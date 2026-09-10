// Dead-reckoning coast: firmware glue around zCoastCore.
//
// Shadow mode (always, when COAST_SHADOW_MODE): the estimator runs against live GNSS and reports
//   $COASTSHADOW,dur_s,dist_m,along_m,cross_m,maxAlong_m,maxCross_m,delta_deg,offset_deg,beta_deg,
//                vStart,vEnd,fixes,Leff_m,k_crab,wasSign,obsFrac,extFrac,extScale
//   one line per window. cross_m is the number that matters for steering.
// Live coast (only when COAST_LIVE_ENABLE): when the RTK fix is lost the raw KSXT/GGA output to AgIO is
//   suppressed and a coasted position is sent every 100 ms instead ($PANDA fix quality 6, or a synthetic
//   $KSXT if COAST_OUTPUT_KSXT), until two RTK fixes return, COAST_MAX_S / COAST_MAX_M is hit, or the
//   sensors fail. On timeout one quality-0 sentence is sent so AgOpenGPS disengages as it does today.
//   $COASTREPORT,reason,dur_s,dist_m,along_m,cross_m,vStart,forced,fallback
//   reason: 1 recovered, 2 timeout, 3 sensor loss. along/cross = first real fix minus the coasted position.
//
// Where the diagnostics go (no USB needed in the tractor):
//   - USB serial, as before
//   - UDP broadcast on COAST_UDP_LOG_PORT (every line; capture with tests/coast/coast_monitor.py)
//   - AgOpenGPS hardware message (PGN 221) for reports, coast start/end and geometry changes: shown on
//     the AgOpenGPS screen when "Hardware Messages" is enabled, and written to its event log
// Commands, any of: USB serial, UDP text to the module's port 8888, or AgOpenGPS PGN 210 (Coast button):
//   "!AOGCO,10"                   force a 10 s coast while GNSS is good (report then holds the true error)
//   "!AOGCG,L,pivot,height,offset" set the vehicle geometry (metres, AgOpenGPS signs)
// Optional 10 Hz log: $COAST,... (COAST_LOG) for offline replay with tests/coast/coast_ref.py.
//
// Settings live in the user-settings block of the main sketch. Design: docs/dead_reckoning_coast_design.md

#include "zCoastCore.h"

extern char fixTime[12];    // last GGA time, zHandlers.ino
extern char numSats[4];

coast_t coastState;
bool coastInitDone = false;
bool coastWasLive = false;

// Last KSXT fields kept as strings for the synthetic-KSXT output (filled by KSXT_Handler)
char coastKsxtUtc[24] = "";
char coastKsxtSatsSlave[6] = "0";
char coastKsxtSatsMaster[6] = "0";
char coastKsxtRollField[12] = "";

coast_out_t coastLastOut;
bool coastLastOutValid = false;
uint32_t coastLastEmitMs = 0;

// ----------------------------------------------------------------------------- transport
// One diagnostic line: USB + UDP broadcast on the log port
void coastSendText(const char *line)
{
  Serial.println(line);
  if (Ethernet_running && COAST_UDP_LOG_PORT > 0)
  {
    IPAddress bcast(networkAddress.ipOne, networkAddress.ipTwo, networkAddress.ipThree, 255);
    Eth_udpPAOGI.beginPacket(bcast, COAST_UDP_LOG_PORT);
    Eth_udpPAOGI.write((const uint8_t *)line, strlen(line));
    Eth_udpPAOGI.write((const uint8_t *)"\r\n", 2);
    Eth_udpPAOGI.endPacket();
  }
}

// AgOpenGPS hardware message, PGN 221: { 0x80, 0x81, 126, 221, len, seconds, colour, text..., crc }
void coastSendDisplay(const char *text, uint8_t seconds, bool alarm)
{
  if (!Ethernet_running) return;
  uint8_t pkt[100];
  size_t n = strlen(text);
  if (n > 80) n = 80;
  pkt[0] = 0x80; pkt[1] = 0x81; pkt[2] = 126; pkt[3] = 221;
  pkt[4] = (uint8_t)(n + 2);
  pkt[5] = seconds;
  pkt[6] = alarm ? 0 : 1;          // 0 = salmon, other = bisque
  memcpy(pkt + 7, text, n);
  int16_t crc = 0;
  for (size_t i = 2; i < 7 + n; i++) crc += pkt[i];
  pkt[7 + n] = (uint8_t)crc;
  SendUdp(pkt, (uint8_t)(8 + n), Eth_ipDestination, portDestination);
}

// ----------------------------------------------------------------------------- geometry
// Vehicle geometry: compile-time defaults < EEPROM (last value received) < AgOpenGPS PGN 209 / "!AOGCG" command
#define COAST_EE_ADDR  100          // steer settings use 0..70
#define COAST_EE_IDENT 0xC0A5
struct CoastGeomEE { uint16_t ident; int16_t wheelbase_cm, pivot_cm, height_cm, offset_cm; };

// offsetAog uses the AgOpenGPS sign (+ = antenna left of centre)
void coastApplyGeometry(float L, float a, float h, float offsetAog, const char *source, bool save)
{
  if (!coastInitDone) coastInit();
  bool ok = coast_set_geometry(&coastState, L, a, h, -offsetAog);
  char msg[120];
  snprintf(msg, sizeof(msg), "$COASTMSG,geometry %s from %s: L=%.2f pivot=%.2f height=%.2f offset=%.2f",
           ok ? (coastState.geom_pending ? "queued" : "applied") : "REJECTED", source, L, a, h, offsetAog);
  coastSendText(msg);
  if (save)
  {
    snprintf(msg, sizeof(msg), "Coast geometry %s: L %.2f pivot %.2f h %.2f offs %.2f",
             ok ? "set" : "REJECTED", L, a, h, offsetAog);
    coastSendDisplay(msg, 5, !ok);
  }
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

// ",L,pivot,height,offset" (metres, AOG signs)
void coastGeometryCommand(const char *args)
{
  // four floats via strtof (sscanf %f costs ~28 KB of flash)
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
  if (n == 4) coastApplyGeometry(v[0], v[1], v[2], v[3], "!AOGCG", true);
  else coastSendText("$COASTMSG,usage: !AOGCG,wheelbase,antennaPivot,antennaHeight,antennaOffset (m)");
}

// ----------------------------------------------------------------------------- commands
void coastForce(int seconds)
{
  if (!coastInitDone) coastInit();
  char msg[80];
  if (!COAST_LIVE_ENABLE)
  {
    coastSendText("$COASTMSG,forced coast ignored: COAST_LIVE_ENABLE is false");
    coastSendDisplay("Coast test ignored: live coast is off in this build", 5, true);
    return;
  }
  coast_force(&coastState, millis(), (float)seconds);
  snprintf(msg, sizeof(msg), "$COASTMSG,forced coast for %d s", seconds);
  coastSendText(msg);
  snprintf(msg, sizeof(msg), "Coast test: %d s forced coast starting", seconds);
  coastSendDisplay(msg, 4, false);
}

// code 'O' = force (args ",seconds"), 'G' = geometry (args ",L,a,h,o"). Shared by USB and UDP.
void coastCommand(char code, const char *args)
{
  if (code == 'O')
  {
    const char *p = args;
    if (*p == ',') p++;
    coastForce(atoi(p));
  }
  else if (code == 'G')
  {
    coastGeometryCommand(args);
  }
}

// "!AOGCO,10" or "!AOGCG,..." arriving as text on the module's UDP port
void coastUdpCommand(const uint8_t *data, int len)
{
  char buf[64];
  int n = len < 63 ? len : 63;
  memcpy(buf, data, n);
  buf[n] = 0;
  for (int i = 0; i < n; i++) if (buf[i] == '\r' || buf[i] == '\n') { buf[i] = 0; break; }
  if (n >= 6 && buf[4] == 'C') coastCommand(buf[5], buf + 6);
}

// AgOpenGPS PGN 210: cmd 1 = forced coast for <value> seconds
void coastOnPgn210(uint8_t cmd, uint16_t value)
{
  if (cmd == 1) coastForce((int)value);
}

// ----------------------------------------------------------------------------- pulse input
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

// ----------------------------------------------------------------------------- init
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
    coastSendText("$COASTMSG,geometry from compile-time defaults (no EEPROM value yet)");
  }

#if COAST_SPEED_PULSE_PIN >= 0
  pinMode(COAST_SPEED_PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COAST_SPEED_PULSE_PIN), coastPulseIsr, RISING);
#endif

  char msg[200];
  snprintf(msg, sizeof(msg), "$COASTMSG,build: shadow %s, live %s (%s, max %.0f s / %.0f m), log %s, udp log port %d, "
           "L=%.2f a=%.2f h=%.2f offs=%.2f, observer %s, crab %s, speed pulse %s",
           COAST_SHADOW_MODE ? "ON" : "OFF", COAST_LIVE_ENABLE ? "ON" : "OFF",
           COAST_OUTPUT_KSXT ? "KSXT" : "PANDA q6", (double)COAST_MAX_S, (double)COAST_MAX_M,
           COAST_LOG ? "ON" : "OFF", (int)COAST_UDP_LOG_PORT,
           (double)COAST_WHEELBASE_M, (double)COAST_ANTENNA_FWD_M, (double)COAST_ANTENNA_HEIGHT_M, (double)COAST_ANTENNA_OFFSET_M,
           COAST_SPEED_OBSERVER ? "ON" : "OFF", COAST_CRAB_MODEL ? "ON" : "OFF",
           COAST_SPEED_PULSE_PIN >= 0 ? "pin " STR(COAST_SPEED_PULSE_PIN) : "none");
  coastSendText(msg);
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

// ----------------------------------------------------------------------------- output sentences
static void coastNmeaChecksum(char *s)
{
  uint8_t sum = 0;
  for (char *p = s + 1; *p && *p != '*'; p++) sum ^= (uint8_t)*p;
  char tail[8];
  snprintf(tail, sizeof(tail), "*%02X\r\n", sum);
  strcat(s, tail);
}

static void coastSendSentence(const char *s)
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
  coastSendSentence(s);
  coastLastEmitMs = millis();
}

// ----------------------------------------------------------------------------- loop
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

  // coast started
  if (coastState.live && !coastWasLive)
  {
    char msg[80];
    snprintf(msg, sizeof(msg), "COAST: %s, dead reckoning from last RTK fix", coastState.live_forced ? "forced test" : "GNSS lost");
    coastSendDisplay(msg, 4, !coastState.live_forced);
    coastSendText(coastState.live_forced ? "$COASTMSG,live coast started (forced)" : "$COASTMSG,live coast started (GNSS lost)");
  }
  coastWasLive = coastState.live;

  if (coastState.live_report_ready)
  {
    coast_live_report_t *r = &coastState.live_report;
    char msg[160];
    if (r->has_error)
      snprintf(msg, sizeof(msg), "$COASTREPORT,%d,%.1f,%.1f,%.2f,%.2f,%.2f,%d,%d",
               r->reason, r->duration_ms / 1000.0f, r->dist_m, r->along_m, r->cross_m, r->v_start_mps,
               r->forced ? 1 : 0, r->used_fallback ? 1 : 0);
    else
      snprintf(msg, sizeof(msg), "$COASTREPORT,%d,%.1f,%.1f,,,%.2f,%d,%d",
               r->reason, r->duration_ms / 1000.0f, r->dist_m, r->v_start_mps,
               r->forced ? 1 : 0, r->used_fallback ? 1 : 0);
    coastSendText(msg);

    const char *why = r->reason == COAST_END_RECOVERED ? "RTK back" : r->reason == COAST_END_TIMEOUT ? "TIMED OUT" : "SENSOR LOST";
    if (r->has_error)
      snprintf(msg, sizeof(msg), "Coast %s after %.1f s / %.0f m: error along %.2f cross %.2f m%s",
               why, r->duration_ms / 1000.0f, r->dist_m, r->along_m, r->cross_m, r->used_fallback ? " (wheel model)" : "");
    else
      snprintf(msg, sizeof(msg), "Coast %s after %.1f s / %.0f m%s",
               why, r->duration_ms / 1000.0f, r->dist_m, r->used_fallback ? " (wheel model)" : "");
    coastSendDisplay(msg, 12, r->reason != COAST_END_RECOVERED);
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

// ----------------------------------------------------------------------------- GNSS event
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
    char msg[220];
    snprintf(msg, sizeof(msg), "$COASTSHADOW,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%lu,%.3f,%.2f,%.0f,%.2f,%.2f,%.4f",
             r->duration_ms / 1000.0f, r->dist_m, r->along_m, r->cross_m, r->max_abs_along_m, r->max_abs_cross_m,
             r->delta_deg, r->offset_deg, r->beta_deg, r->v_start_mps, r->v_end_mps, (unsigned long)r->fix_count,
             r->leff_m, r->k_crab, r->was_sign, r->observer_frac, r->ext_frac, r->ext_scale);
    coastSendText(msg);
    if (COAST_DISPLAY_SHADOW)
    {
      snprintf(msg, sizeof(msg), "Shadow %.0fs/%.0fm: cross %.2f along %.2f (max %.2f/%.2f) L%.2f k%.1f s%.2f",
               r->duration_ms / 1000.0f, r->dist_m, r->cross_m, r->along_m, r->max_abs_cross_m, r->max_abs_along_m,
               r->leff_m, r->k_crab, r->ext_scale);
      coastSendDisplay(msg, 8, false);
    }
    coastState.report_ready = false;
  }

  if (COAST_LOG)
  {
    char msg[240];
    snprintf(msg, sizeof(msg), "$COAST,%lu,%.8f,%.8f,%d,%d,%.2f,%.2f,%.3f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.3f,%.3f,%.3f,%.2f,%.3f",
             (unsigned long)t_ms, lat, lon, posQ, hdgQ, hdgRaw, track, vMps, dualRoll,
             coastState.yaw_deg, coastState.roll_deg, coastState.pitch_deg, coastState.was_deg,
             coastState.delta_valid ? coastState.delta_deg : 999.0f,
             coastState.quad_valid ? coastState.quad_offset_deg : 999,
             coastState.live ? 2 : (coastState.active ? 1 : 0),
             coastState.along_m, coastState.cross_m,
             coastState.leff_valid ? coastState.leff_m : 0.0f,
             coastState.k_valid ? coastState.k_crab : 0.0f,
             coastState.ext_valid ? coastState.ext_v_raw_mps : -1.0f);
    coastSendText(msg);
  }

  return !coastState.live;
}
