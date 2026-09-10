// Dead-reckoning coast: firmware glue around zCoastCore.
//
// Shadow mode (always, when COAST_SHADOW_MODE): the estimator runs against live GNSS and prints
//   $COASTSHADOW,dur_s,dist_m,along_m,cross_m,maxAlong_m,maxCross_m,delta_deg,offset_deg,beta_deg,
//                vStart,vEnd,fixes,Leff_m,k_crab,wasSign,obsFrac,extFrac,extScale
//   one line per window. cross_m is the number that matters for steering.
// Live coast (only when COAST_LIVE_ENABLE): when the RTK fix is lost the raw KSXT/GGA output to AgIO is
//   suppressed and a coasted position is sent every 100 ms instead ($PANDA fix quality 6, or a synthetic
//   $KSXT if COAST_OUTPUT_KSXT), until two RTK fixes return, COAST_MAX_S / COAST_MAX_M is hit, or the
//   sensors fail. On timeout one quality-0 sentence is sent so AgOpenGPS disengages as it does today.
//   $COASTREPORT,reason,dur_s,dist_m,along_m,cross_m,vStart,forced,fallback
//   reason: 1 recovered, 2 timeout, 3 sensor loss. along/cross = first real fix minus the coasted position.
// Field test: "!AOGCO,10" on USB forces a 10 s coast while GNSS is good; the report then holds the true error.
// Optional 10 Hz log: $COAST,... (COAST_LOG_USB) for offline replay with tests/coast/coast_ref.py.
//
// Settings live in the user-settings block of the main sketch. Design: docs/dead_reckoning_coast_design.md

#include "zCoastCore.h"

extern char fixTime[12];    // last GGA time, zHandlers.ino
extern char numSats[4];

coast_t coastState;
bool coastInitDone = false;

// Last KSXT fields kept as strings for the synthetic-KSXT output (filled by KSXT_Handler)
char coastKsxtUtc[24] = "";
char coastKsxtSatsSlave[6] = "0";
char coastKsxtSatsMaster[6] = "0";
char coastKsxtRollField[12] = "";

coast_out_t coastLastOut;
bool coastLastOutValid = false;
uint32_t coastLastEmitMs = 0;

// Vehicle geometry: compile-time defaults < EEPROM (last value received) < AgOpenGPS PGN 209 / "!AOGCG" command
#define COAST_EE_ADDR  100          // steer settings use 0..70
#define COAST_EE_IDENT 0xC0A5
struct CoastGeomEE { uint16_t ident; int16_t wheelbase_cm, pivot_cm, height_cm, offset_cm; };

// offsetAog uses the AgOpenGPS sign (+ = antenna left of centre)
void coastApplyGeometry(float L, float a, float h, float offsetAog, const char *source, bool save)
{
  if (!coastInitDone) coastInit();
  bool ok = coast_set_geometry(&coastState, L, a, h, -offsetAog);
  Serial.print("$COASTMSG,geometry ");
  Serial.print(ok ? (coastState.geom_pending ? "queued (coast running)" : "applied") : "REJECTED");
  Serial.print(" from "); Serial.print(source);
  Serial.print(": L="); Serial.print(L, 2);
  Serial.print(" pivot="); Serial.print(a, 2);
  Serial.print(" height="); Serial.print(h, 2);
  Serial.print(" offset="); Serial.println(offsetAog, 2);
  if (!ok || !save) return;

  CoastGeomEE ee;
  ee.ident = COAST_EE_IDENT;
  ee.wheelbase_cm = (int16_t)lroundf(L * 100.0f);
  ee.pivot_cm     = (int16_t)lroundf(a * 100.0f);
  ee.height_cm    = (int16_t)lroundf(h * 100.0f);
  ee.offset_cm    = (int16_t)lroundf(offsetAog * 100.0f);
  CoastGeomEE cur;
  EEPROM.get(COAST_EE_ADDR, cur);
  if (memcmp(&cur, &ee, sizeof(ee)) != 0) EEPROM.put(COAST_EE_ADDR, ee);   // only write on change
}

// "!AOGCG,L,pivot,height,offset" (metres, AOG signs)
void coastGeometryCommand(const char *args)
{
  // ",L,a,h,o" -> four floats (strtof, not sscanf: %f scanning costs ~28 KB of flash)
  float v[4];
  const char *p = args;
  int n = 0;
  while (n < 4)
  {
    if (*p == ',') p++;
    char *end;
    v[n] = strtof(p, &end);
    if (end == p) break;
    n++;
    p = end;
  }
  if (n == 4) coastApplyGeometry(v[0], v[1], v[2], v[3], "USB !AOGCG", true);
  else Serial.println("$COASTMSG,usage: !AOGCG,wheelbase,antennaPivot,antennaHeight,antennaOffset (m)");
}

// Ground-speed pulse input (Phase 3). Counted in an interrupt, sampled every 50 ms.
#if COAST_SPEED_PULSE_PIN >= 0
volatile uint32_t coastPulseCount = 0;
volatile uint32_t coastPulseLastUs = 0;
void coastPulseIsr()
{
  uint32_t now = micros();
  if ((now - coastPulseLastUs) > 100)      // glitch filter: ISO 11786 tops out around 1.5 kHz
  {
    coastPulseCount++;
    coastPulseLastUs = now;
  }
}
#endif
uint32_t coastPulsePrevCount = 0;
uint32_t coastPulsePrevUs = 0;
float    coastPulseSpeed = 0.0f;
elapsedMillis coastPulseTimer;

void coastInit()
{
  coast_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.wheelbase_m             = COAST_WHEELBASE_M;
  cfg.antenna_fwd_m           = COAST_ANTENNA_FWD_M;
  cfg.antenna_height_m        = COAST_ANTENNA_HEIGHT_M;
  cfg.antenna_right_m         = -(COAST_ANTENNA_OFFSET_M);   // AOG: + = antenna left of centre
  cfg.imu_yaw_sign            = COAST_IMU_YAW_SIGN;
  cfg.imu_roll_sign           = COAST_IMU_ROLL_SIGN;
  cfg.dual_heading_offset_deg = COAST_DUAL_HEADING_OFFSET_DEG;
  cfg.shadow_window_s         = COAST_SHADOW_WINDOW_S;
  cfg.min_speed_mps           = 0.5f;
  cfg.was_sign                = COAST_WAS_SIGN;
  cfg.speed_observer          = COAST_SPEED_OBSERVER;
  cfg.crab_model              = COAST_CRAB_MODEL;
  cfg.ext_speed               = COAST_EXT_SPEED && (COAST_SPEED_PULSE_PIN >= 0);
  cfg.live_enable             = COAST_LIVE_ENABLE;
  cfg.live_max_s              = COAST_MAX_S;
  cfg.live_max_m              = COAST_MAX_M;
  cfg.live_on_float           = COAST_ON_FLOAT;
  coast_init(&coastState, &cfg);
  coastInitDone = true;

  CoastGeomEE ee;
  EEPROM.get(COAST_EE_ADDR, ee);
  if (ee.ident == COAST_EE_IDENT)
  {
    coastApplyGeometry(ee.wheelbase_cm * 0.01f, ee.pivot_cm * 0.01f, ee.height_cm * 0.01f, ee.offset_cm * 0.01f, "EEPROM", false);
  }
  else
  {
    Serial.println("$COASTMSG,geometry from compile-time defaults (no EEPROM value yet)");
  }

#if COAST_SPEED_PULSE_PIN >= 0
  pinMode(COAST_SPEED_PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COAST_SPEED_PULSE_PIN), coastPulseIsr, RISING);
#endif

  Serial.print("Coast: shadow ");
  Serial.print(COAST_SHADOW_MODE ? "ON" : "OFF");
  Serial.print(", LIVE ");
  Serial.print(COAST_LIVE_ENABLE ? "ON" : "OFF");
  if (COAST_LIVE_ENABLE)
  {
    Serial.print(" (");
    Serial.print(COAST_OUTPUT_KSXT ? "KSXT" : "PANDA q6");
    Serial.print(", max ");
    Serial.print(COAST_MAX_S);
    Serial.print(" s / ");
    Serial.print(COAST_MAX_M);
    Serial.print(" m)");
  }
  Serial.print(", USB log ");
  Serial.print(COAST_LOG_USB ? "ON" : "OFF");
  Serial.print(", L=");
  Serial.print(COAST_WHEELBASE_M);
  Serial.print(" a=");
  Serial.print(COAST_ANTENNA_FWD_M);
  Serial.print(" h=");
  Serial.print(COAST_ANTENNA_HEIGHT_M);
  Serial.print(" m, observer ");
  Serial.print(COAST_SPEED_OBSERVER ? "ON" : "OFF");
  Serial.print(", crab ");
  Serial.print(COAST_CRAB_MODEL ? "ON" : "OFF");
  Serial.print(", speed pulse ");
  if (COAST_SPEED_PULSE_PIN >= 0) { Serial.print("pin "); Serial.print(COAST_SPEED_PULSE_PIN); Serial.print(" @ "); Serial.print(COAST_PULSES_PER_M); Serial.print(" p/m"); }
  else Serial.print("none");
  Serial.println();
}

bool coastLive()
{
  return coastInitDone && coastState.live;
}

// Called from TM171.ino for every function-code-35 packet.
void coastOnImu(uint32_t t_us, float yawDeg, float rollDeg, float pitchDeg)
{
  if (!COAST_SHADOW_MODE) return;
  if (!coastInitDone) coastInit();
  coast_imu(&coastState, t_us, yawDeg, rollDeg, pitchDeg);
}

// Called from Autosteer.ino whenever steerAngleActual is refreshed.
void coastOnWas(float steerDeg)
{
  if (!COAST_SHADOW_MODE) return;
  if (!coastInitDone) coastInit();
  coast_was(&coastState, steerDeg);
}

// "!AOGCO,<seconds>" on USB
void coastForce(int seconds)
{
  if (!coastInitDone) coastInit();
  if (!COAST_LIVE_ENABLE)
  {
    Serial.println("$COASTMSG,forced coast ignored: COAST_LIVE_ENABLE is false");
    return;
  }
  coast_force(&coastState, millis(), (float)seconds);
  Serial.print("$COASTMSG,forced coast for ");
  Serial.print(seconds);
  Serial.println(" s");
}

// ----------------------------------------------------------------------------- output sentences
static void coastNmeaChecksum(char *s)
{
  uint8_t sum = 0;
  for (char *p = s + 1; *p && *p != '*'; p++) sum ^= (uint8_t)*p;
  char tail[8];
  snprintf(tail, sizeof(tail), "*%02X\r\n", sum);
  strcat(s, tail);
}

static void coastSend(const char *s)
{
  if (!passThroughGPS && !passThroughGPS2) SerialAOG.print(s);
  if (Ethernet_running)
  {
    Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
    Eth_udpPAOGI.write(s, strlen(s));
    Eth_udpPAOGI.endPacket();
  }
}

// hhmmss.ss + elapsed seconds -> hhmmss.ss
static void coastTimePlus(const char *hhmmss, float plus_s, char *out, size_t n)
{
  int hh = 0, mm = 0; float ss = 0.0f;
  if (strlen(hhmmss) >= 6)
  {
    hh = (hhmmss[0] - '0') * 10 + (hhmmss[1] - '0');
    mm = (hhmmss[2] - '0') * 10 + (hhmmss[3] - '0');
    ss = atof(hhmmss + 4);
  }
  float tot = hh * 3600.0f + mm * 60.0f + ss + plus_s;
  while (tot >= 86400.0f) tot -= 86400.0f;
  int h = (int)(tot / 3600.0f); tot -= h * 3600.0f;
  int m = (int)(tot / 60.0f);   tot -= m * 60.0f;
  snprintf(out, n, "%02d%02d%05.2f", h, m, tot);
}

static void coastLatLonNmea(double lat, double lon, char *latS, char *ns, char *lonS, char *ew)
{
  double alat = fabs(lat), alon = fabs(lon);
  int dlat = (int)alat, dlon = (int)alon;
  snprintf(latS, 24, "%02d%010.7f", dlat, (alat - dlat) * 60.0);
  snprintf(lonS, 24, "%03d%010.7f", dlon, (alon - dlon) * 60.0);
  ns[0] = lat < 0 ? 'S' : 'N'; ns[1] = 0;
  ew[0] = lon < 0 ? 'W' : 'E'; ew[1] = 0;
}

// $PANDA built from coastLastOut, same field encoding as the TM171 path in imuHandler()
// (the builders read the stored output struct: Arduino's auto-prototypes cannot see coast_out_t)
static void coastBuildPanda(char *s, bool q0)
{
  const coast_out_t *o = &coastLastOut;
  char t[12], latS[24], lonS[24], ns[2], ew[2];
  coastTimePlus(fixTime, o->elapsed_ms * 1e-3f, t, sizeof(t));
  coastLatLonNmea(o->lat_deg, o->lon_deg, latS, ns, lonS, ew);

  float hdgField = COAST_PANDA_HEADING_TRUE ? o->heading_deg : coastState.yaw_deg;
  float rollField = o->roll_deg, pitchField = o->pitch_deg;
  if (steerConfig.IsUseY_Axis) { rollField = o->pitch_deg; pitchField = o->roll_deg; }

  snprintf(s, 190, "$PANDA,%s,%s,%s,%s,%s,%d,%s,%.2f,%.2f,%.1f,%.2f,%d,%d,%d,0",
           t, latS, ns, lonS, ew,
           q0 ? 0 : 6,
           numSats,
           o->hdop,
           o->alt_m,
           o->elapsed_ms * 1e-3f,
           o->v_mps * 1.943844f,
           (int)(hdgField * 10.0f),
           (int)(rollField * 10.0f),
           (int)(pitchField * 10.0f));
  coastNmeaChecksum(s);
}

// Synthetic $KSXT (Plan B): keeps AgIO in the same sentence mode it was in before the loss
static void coastBuildKsxt(char *s, bool q0)
{
  const coast_out_t *o = &coastLastOut;
  char utc[24] = "";
  if (strlen(coastKsxtUtc) >= 15)
  {
    char hms[12];
    strncpy(utc, coastKsxtUtc, 8); utc[8] = 0;                       // yyyymmdd
    coastTimePlus(coastKsxtUtc + 8, o->elapsed_ms * 1e-3f, hms, sizeof(hms));
    strcat(utc, hms);
  }
  float hdgRaw = coast_wrap360(o->heading_deg - (float)coastState.quad_offset_deg);
  float dualRoll = coastState.dual_roll_at_loss_deg + (o->roll_deg - coastState.roll_at_loss_deg) * COAST_KSXT_ROLL_SIGN;
  float vk = o->v_mps * 3.6f;
  float tr = o->track_deg * 0.017453292f;

  snprintf(s, 190, "$KSXT,%s,%.8f,%.8f,%.4f,%.2f,%.2f,%.2f,%.3f,%s,%d,%d,%s,%s,,,,%.3f,%.3f,0.000,,",
           utc, o->lon_deg, o->lat_deg, o->alt_m,
           hdgRaw, dualRoll, o->track_deg, vk,
           coastKsxtRollField,
           q0 ? 0 : COAST_KSXT_QUALITY, 3,
           coastKsxtSatsSlave, coastKsxtSatsMaster,
           vk * sinf(tr), vk * cosf(tr));
  coastNmeaChecksum(s);
}

static void coastEmit(bool q0)
{
  if (!q0)
  {
    if (!coast_live_output(&coastState, millis(), &coastLastOut)) return;
    coastLastOutValid = true;
  }
  else if (!coastLastOutValid)
  {
    return;
  }
  char s[200];
  if (COAST_OUTPUT_KSXT) coastBuildKsxt(s, q0); else coastBuildPanda(s, q0);
  coastSend(s);
  coastLastEmitMs = millis();
}

// Called from loop() every iteration.
void coastLoop()
{
  if (!COAST_SHADOW_MODE || !coastInitDone) return;

#if COAST_SPEED_PULSE_PIN >= 0
  if (coastPulseTimer >= 50)
  {
    coastPulseTimer = 0;
    uint32_t cnt, lastUs;
    noInterrupts();
    cnt = coastPulseCount; lastUs = coastPulseLastUs;
    interrupts();
    uint32_t nowUs = micros();
    if (cnt != coastPulsePrevCount)
    {
      uint32_t dp = cnt - coastPulsePrevCount;
      float dtp = (float)(lastUs - coastPulsePrevUs) * 1e-6f;
      if (coastPulsePrevUs != 0 && dtp > 0.0f) coastPulseSpeed = (float)dp / COAST_PULSES_PER_M / dtp;
      coastPulsePrevCount = cnt;
      coastPulsePrevUs = lastUs;
    }
    else if ((nowUs - lastUs) > 500000u)
    {
      coastPulseSpeed = 0.0f;       // no pulse for 0.5 s: stopped
    }
    coast_ext_speed(&coastState, millis(), coastPulseSpeed);
  }
#endif

  coast_tick(&coastState, micros());

  if (coastState.live_report_ready)
  {
    coast_live_report_t *r = &coastState.live_report;
    Serial.print("$COASTREPORT,");
    Serial.print(r->reason);                    Serial.print(",");
    Serial.print(r->duration_ms / 1000.0f, 1);  Serial.print(",");
    Serial.print(r->dist_m, 1);                 Serial.print(",");
    if (r->has_error) { Serial.print(r->along_m, 2); Serial.print(","); Serial.print(r->cross_m, 2); }
    else Serial.print(",");
    Serial.print(",");
    Serial.print(r->v_start_mps, 2);            Serial.print(",");
    Serial.print(r->forced ? 1 : 0);            Serial.print(",");
    Serial.println(r->used_fallback ? 1 : 0);
    coastState.live_report_ready = false;
  }

  if (coastState.q0_pending)
  {
    coastEmit(true);
    coastState.q0_pending = false;
  }

  if (coastState.live && (millis() - coastLastEmitMs) >= 100)
  {
    coastEmit(false);
  }
}

// Called from KSXT_Handler with the parsed fields. Speed already in m/s.
// Returns true if the raw KSXT sentence should still be forwarded to AgIO.
bool coastOnKSXT(uint32_t t_ms, double lat, double lon, float alt, int posQ, int hdgQ,
                 float hdgRaw, float track, float vMps, float dualRoll)
{
  if (!COAST_SHADOW_MODE) return true;
  if (!coastInitDone) coastInit();

  coast_fix_t fx;
  fx.t_ms = t_ms;
  fx.lat_deg = lat;  fx.lon_deg = lon;  fx.alt_m = alt;
  fx.pos_q = posQ;   fx.hdg_q = hdgQ;
  fx.hdg_raw_deg = hdgRaw;  fx.track_deg = track;
  fx.v_mps = vMps;   fx.dual_roll_deg = dualRoll;
  coast_gnss(&coastState, &fx);

  if (coastState.report_ready)
  {
    coast_report_t *r = &coastState.report;
    Serial.print("$COASTSHADOW,");
    Serial.print(r->duration_ms / 1000.0f, 1);  Serial.print(",");
    Serial.print(r->dist_m, 1);                 Serial.print(",");
    Serial.print(r->along_m, 2);                Serial.print(",");
    Serial.print(r->cross_m, 2);                Serial.print(",");
    Serial.print(r->max_abs_along_m, 2);        Serial.print(",");
    Serial.print(r->max_abs_cross_m, 2);        Serial.print(",");
    Serial.print(r->delta_deg, 2);              Serial.print(",");
    Serial.print(r->offset_deg);                Serial.print(",");
    Serial.print(r->beta_deg, 2);               Serial.print(",");
    Serial.print(r->v_start_mps, 2);            Serial.print(",");
    Serial.print(r->v_end_mps, 2);              Serial.print(",");
    Serial.print(r->fix_count);                 Serial.print(",");
    Serial.print(r->leff_m, 3);                 Serial.print(",");
    Serial.print(r->k_crab, 2);                 Serial.print(",");
    Serial.print(r->was_sign, 0);               Serial.print(",");
    Serial.print(r->observer_frac, 2);          Serial.print(",");
    Serial.print(r->ext_frac, 2);               Serial.print(",");
    Serial.println(r->ext_scale, 4);
    coastState.report_ready = false;
  }

  if (COAST_LOG_USB)
  {
    Serial.print("$COAST,");
    Serial.print(t_ms);                      Serial.print(",");
    Serial.print(lat, 8);                    Serial.print(",");
    Serial.print(lon, 8);                    Serial.print(",");
    Serial.print(posQ);                      Serial.print(",");
    Serial.print(hdgQ);                      Serial.print(",");
    Serial.print(hdgRaw, 2);                 Serial.print(",");
    Serial.print(track, 2);                  Serial.print(",");
    Serial.print(vMps, 3);                   Serial.print(",");
    Serial.print(dualRoll, 2);               Serial.print(",");
    Serial.print(coastState.yaw_deg, 3);     Serial.print(",");
    Serial.print(coastState.roll_deg, 2);    Serial.print(",");
    Serial.print(coastState.pitch_deg, 2);   Serial.print(",");
    Serial.print(coastState.was_deg, 2);     Serial.print(",");
    Serial.print(coastState.delta_valid ? coastState.delta_deg : 999.0f, 2); Serial.print(",");
    Serial.print(coastState.quad_valid ? coastState.quad_offset_deg : 999);  Serial.print(",");
    Serial.print(coastState.live ? 2 : (coastState.active ? 1 : 0)); Serial.print(",");
    Serial.print(coastState.along_m, 3);     Serial.print(",");
    Serial.print(coastState.cross_m, 3);     Serial.print(",");
    Serial.print(coastState.leff_valid ? coastState.leff_m : 0.0f, 3); Serial.print(",");
    Serial.print(coastState.k_valid ? coastState.k_crab : 0.0f, 2);    Serial.print(",");
    Serial.println(coastState.ext_valid ? coastState.ext_v_raw_mps : -1.0f, 3);
  }

  return !coastState.live;
}
