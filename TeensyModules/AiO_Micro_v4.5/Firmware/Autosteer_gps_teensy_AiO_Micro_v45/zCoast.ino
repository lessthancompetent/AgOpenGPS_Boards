// Dead-reckoning coast: firmware glue around zCoastCore (Phase 0 shadow mode + Phase 2 models).
//
// Nothing AgIO receives is changed. The firmware parses KSXT, feeds the coast core, runs the
// estimator in shadow mode against live GNSS, and prints its error on USB:
//
//   $COASTSHADOW,dur_s,dist_m,along_m,cross_m,maxAlong_m,maxCross_m,delta_deg,offset_deg,beta_deg,
//                vStart,vEnd,fixes,Leff_m,k_crab,wasSign,obsFrac,extFrac,extScale
//       one line per shadow window (default every 20 s while moving with an RTK fix + TM171).
//       cross_m is the number that matters for steering. Leff/k/wasSign/extScale are 0 until learned.
//   $COAST,t_ms,lat,lon,posq,hdgq,hdg_raw,track,v_mps,dual_roll,yaw,roll,pitch,was_deg,delta,offset,
//          active,along,cross,Leff,k,pulse_v_raw
//       one line per KSXT when COAST_LOG_USB is true (10 Hz), for offline replay with tests/coast/coast_ref.py
//
// Settings live in the user-settings block of the main sketch. Design: docs/dead_reckoning_coast_design.md

#include "zCoastCore.h"

coast_t coastState;
bool coastInitDone = false;

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
  cfg.imu_yaw_sign            = COAST_IMU_YAW_SIGN;
  cfg.imu_roll_sign           = COAST_IMU_ROLL_SIGN;
  cfg.dual_heading_offset_deg = COAST_DUAL_HEADING_OFFSET_DEG;
  cfg.shadow_window_s         = COAST_SHADOW_WINDOW_S;
  cfg.min_speed_mps           = 0.5f;
  cfg.was_sign                = COAST_WAS_SIGN;
  cfg.speed_observer          = COAST_SPEED_OBSERVER;
  cfg.crab_model              = COAST_CRAB_MODEL;
  cfg.ext_speed               = COAST_EXT_SPEED && (COAST_SPEED_PULSE_PIN >= 0);
  coast_init(&coastState, &cfg);
  coastInitDone = true;

#if COAST_SPEED_PULSE_PIN >= 0
  pinMode(COAST_SPEED_PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COAST_SPEED_PULSE_PIN), coastPulseIsr, RISING);
#endif

  Serial.print("Coast: shadow mode ");
  Serial.print(COAST_SHADOW_MODE ? "ON" : "OFF");
  Serial.print(", USB log ");
  Serial.print(COAST_LOG_USB ? "ON" : "OFF");
  Serial.print(", window ");
  Serial.print(COAST_SHADOW_WINDOW_S);
  Serial.print(" s, L=");
  Serial.print(COAST_WHEELBASE_M);
  Serial.print(" a=");
  Serial.print(COAST_ANTENNA_FWD_M);
  Serial.print(" h=");
  Serial.print(COAST_ANTENNA_HEIGHT_M);
  Serial.print(" m, speed observer ");
  Serial.print(COAST_SPEED_OBSERVER ? "ON" : "OFF");
  Serial.print(", crab model ");
  Serial.print(COAST_CRAB_MODEL ? "ON" : "OFF");
  Serial.print(", speed pulse ");
  if (COAST_SPEED_PULSE_PIN >= 0) { Serial.print("pin "); Serial.print(COAST_SPEED_PULSE_PIN); Serial.print(" @ "); Serial.print(COAST_PULSES_PER_M); Serial.print(" p/m"); }
  else Serial.print("none");
  Serial.println(". Output to AgIO is unchanged.");
}

// Called from loop(): converts the pulse count into a raw speed every 50 ms and feeds the core.
void coastLoop()
{
#if COAST_SPEED_PULSE_PIN >= 0
  if (!COAST_SHADOW_MODE || !coastInitDone) return;
  if (coastPulseTimer < 50) return;
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
#endif
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

// Called from KSXT_Handler with the parsed fields. Speed already in m/s.
void coastOnKSXT(uint32_t t_ms, double lat, double lon, float alt, int posQ, int hdgQ,
                 float hdgRaw, float track, float vMps, float dualRoll)
{
  if (!COAST_SHADOW_MODE) return;
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
    Serial.print(coastState.active ? 1 : 0); Serial.print(",");
    Serial.print(coastState.along_m, 3);     Serial.print(",");
    Serial.print(coastState.cross_m, 3);     Serial.print(",");
    Serial.print(coastState.leff_valid ? coastState.leff_m : 0.0f, 3); Serial.print(",");
    Serial.print(coastState.k_valid ? coastState.k_crab : 0.0f, 2);    Serial.print(",");
    Serial.println(coastState.ext_valid ? coastState.ext_v_raw_mps : -1.0f, 3);
  }
}
