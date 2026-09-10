// Dead-reckoning coast core. See zCoastCore.h and the design doc.
#include "zCoastCore.h"
#include <math.h>
#include <string.h>

#define DEG2RAD 0.017453292519943295
#define RAD2DEG 57.29577951308232
#define WGS84_A 6378137.0
#define WGS84_E2 0.00669437999014

// Thresholds shared by learning and the observer
#define TURN_MIN_WAS_DEG     5.0f
#define TURN_MIN_RATE_DPS    2.0f
#define LEARN_MIN_SPEED_MPS  1.0f
#define CRAB_MIN_ROLL_DEG    3.0f
#define CRAB_MAX_ROLL_RATE_DPS 3.0f
#define K_CRAB_LIMIT_DEG     30.0f
#define LEFF_TAU_S           10.0f
#define K_TAU_S              10.0f
#define SLIP_TAU_S           1.0f
#define RES_TAU_S            5.0f
#define OBS_TAU_S            0.25f
#define OBS_MAX_ACCEL_MPS2   1.5f
#define OBS_MAX_SPEED_RATIO  1.25f
#define WAS_SIGN_MIN_VOTES   20
#define LEFF_MIN_SAMPLES     10
#define K_MIN_SAMPLES        50
#define EXT_TIMEOUT_MS       300u
#define EXT_SCALE_TAU_S      30.0f
#define EXT_SCALE_MIN_SAMPLES 50

// Live coast
#define LIVE_SILENCE_MS      250     // no KSXT for this long counts as a loss
#define LIVE_MAX_FIX_AGE_MS  1000    // last RTK fix must be at most this old to start
#define LIVE_MAX_IMU_AGE_MS  100     // TM171 must be at least this fresh to start
#define LIVE_IMU_STALE_US    200000  // switch heading to the wheel model after this
#define LIVE_RECOVER_FIXES   2       // consecutive RTK fixes to end the coast
#define LIVE_HDOP_BASE       0.5f
#define LIVE_HDOP_PER_S      0.05f

float coast_wrap180(float a)
{
  a = fmodf(a + 180.0f, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a - 180.0f;
}

float coast_wrap360(float a)
{
  a = fmodf(a, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a;
}

void coast_earth_radii(double lat_deg, double *rm_m, double *rn_m)
{
  double s = sin(lat_deg * DEG2RAD);
  double d = 1.0 - WGS84_E2 * s * s;
  *rn_m = WGS84_A / sqrt(d);
  *rm_m = WGS84_A * (1.0 - WGS84_E2) / (d * sqrt(d));
}

void coast_init(coast_t *c, const coast_config_t *cfg)
{
  memset(c, 0, sizeof(*c));
  c->cfg = *cfg;
  if (c->cfg.imu_yaw_sign == 0.0f)  c->cfg.imu_yaw_sign = 1.0f;
  if (c->cfg.imu_roll_sign == 0.0f) c->cfg.imu_roll_sign = 1.0f;
  if (c->cfg.shadow_window_s <= 0.0f) c->cfg.shadow_window_s = 20.0f;
  if (c->cfg.min_speed_mps <= 0.0f) c->cfg.min_speed_mps = 0.5f;
  if (c->cfg.live_max_s <= 0.0f) c->cfg.live_max_s = 20.0f;
  if (c->cfg.live_max_m <= 0.0f) c->cfg.live_max_m = 60.0f;
  if (c->cfg.dual_heading_offset_deg != COAST_AUTO_OFFSET)
  {
    c->quad_offset_deg = (int)lroundf(c->cfg.dual_heading_offset_deg);
    c->quad_valid = true;
  }
  if (c->cfg.was_sign > 0.0f)      { c->was_sign = 1.0f;  c->was_sign_valid = true; }
  else if (c->cfg.was_sign < 0.0f) { c->was_sign = -1.0f; c->was_sign_valid = true; }
}

float coast_vehicle_heading(const coast_t *c, float hdg_raw_deg)
{
  if (!c->quad_valid) return -1.0f;
  return coast_wrap360(hdg_raw_deg + (float)c->quad_offset_deg);
}

// ----------------------------------------------------------------------------- helpers
static float imu_roll(const coast_t *c) { return c->cfg.imu_roll_sign * c->roll_deg; }
static float imu_yaw(const coast_t *c)  { return c->cfg.imu_yaw_sign * c->yaw_deg; }
static float lateral_right(const coast_t *c, float roll_deg);

// Lateral position of the antenna relative to the axle centreline: roll lean plus the fixed offset
static float lateral_right(const coast_t *c, float roll_deg)
{
  return c->cfg.antenna_height_m * sinf(roll_deg * (float)DEG2RAD) + c->cfg.antenna_right_m;
}

// Rear axle offset from the antenna, in the current heading/roll: axle = antenna - a*fwd - hr*right
static void axle_from_antenna(const coast_t *c, float psi_deg, float roll_deg, double *dn, double *de)
{
  double s = sin(psi_deg * DEG2RAD), co = cos(psi_deg * DEG2RAD);
  double a = c->cfg.antenna_fwd_m;
  double hr = lateral_right(c, roll_deg);
  *de = -a * s - hr * co;
  *dn = -a * co + hr * s;
}

static bool turning(const coast_t *c, float *delta_deg)
{
  if (!c->was_valid || !c->was_sign_valid) return false;
  float d = c->was_sign * c->was_deg;
  *delta_deg = d;
  return fabsf(d) > TURN_MIN_WAS_DEG && fabsf(c->yaw_rate_dps) > TURN_MIN_RATE_DPS;
}

// Antenna velocity relative to the axle, in the vehicle frame (ignoring slip):
//   along = v_axle - hr*yawrate,  lateral = a*yawrate + h*cos(roll)*rollrate,  hr = h*sin(roll)
// The KSXT speed is the antenna's, so the axle speed is recovered from it, and the geometric
// part of (track - heading) is removed before anything is called "slip".
static float antenna_lateral_mps(const coast_t *c)
{
  float rollc = imu_roll(c) * (float)DEG2RAD;
  return c->cfg.antenna_fwd_m * c->yaw_rate_dps * (float)DEG2RAD
       + c->cfg.antenna_height_m * cosf(rollc) * c->roll_rate_dps * (float)DEG2RAD;
}

static float axle_speed(const coast_t *c, float v_antenna_mps)
{
  float hr = lateral_right(c, imu_roll(c));
  float lat = antenna_lateral_mps(c);
  float along2 = v_antenna_mps * v_antenna_mps - lat * lat;
  float along = along2 > 0.0f ? sqrtf(along2) : 0.0f;
  float v = along + hr * c->yaw_rate_dps * (float)DEG2RAD;
  return v > 0.0f ? v : 0.0f;
}

// Slip crab of the axle: (track - vehicle heading) minus the geometric crab of the antenna
static float slip_crab(const coast_t *c, const coast_fix_t *fix)
{
  float veh = coast_vehicle_heading(c, fix->hdg_raw_deg);
  float beta_total = coast_wrap180(fix->track_deg - veh);
  float hr = lateral_right(c, imu_roll(c));
  float along = axle_speed(c, fix->v_mps) - hr * c->yaw_rate_dps * (float)DEG2RAD;
  float beta_kin = (float)(atan2((double)antenna_lateral_mps(c), (double)along) * RAD2DEG);
  return coast_wrap180(beta_total - beta_kin);
}

// External speed, scaled and projected to the horizontal: the pulse counts distance along the slope.
static bool ext_speed_fresh(const coast_t *c, uint32_t now_ms)
{
  return c->cfg.ext_speed && c->ext_valid && (int32_t)(now_ms - c->ext_t_ms) <= (int32_t)EXT_TIMEOUT_MS;
}

static float ext_speed_h(const coast_t *c)
{
  float s = c->ext_scale_valid ? c->ext_scale : 1.0f;
  return c->ext_v_raw_mps * s * cosf(c->pitch_deg * (float)DEG2RAD);
}

// ----------------------------------------------------------------------------- inputs
void coast_was(coast_t *c, float steer_deg)
{
  c->was_deg = steer_deg; c->was_valid = true;
}

void coast_ext_speed(coast_t *c, uint32_t t_ms, float v_raw_mps)
{
  c->ext_v_raw_mps = v_raw_mps < 0.0f ? 0.0f : v_raw_mps;
  c->ext_t_ms = t_ms; c->ext_valid = true;
}

// One integration step with a given heading. Used by the IMU path and by the wheel-model fallback.
static void integrate(coast_t *c, float dt, float psi_deg, float roll_c, uint32_t now_ms)
{
  c->psi_deg = psi_deg;

  // crab: k*sin(roll_now) + residual (which decays toward the model), or the frozen entry value
  if (c->cfg.crab_model && c->k_valid)
  {
    c->beta_res_deg -= c->beta_res_deg * (dt / (RES_TAU_S + dt));
    c->beta_deg = c->k_crab * sinf(roll_c * (float)DEG2RAD) + c->beta_res_deg;
  }
  else
    c->beta_deg = c->beta_res_deg;

  // speed: external pulse when fresh, else observer on turns, else hold
  float d;
  if (ext_speed_fresh(c, now_ms))
  {
    c->v_mps = ext_speed_h(c);
    c->ext_samples++;
  }
  else if (c->cfg.speed_observer && c->leff_valid && turning(c, &d))
  {
    float v_obs = fabsf(c->yaw_rate_dps * (float)DEG2RAD * c->leff_m / tanf(d * (float)DEG2RAD));
    float a_obs = dt / (dt + OBS_TAU_S);
    float dv = (v_obs - c->v_mps) * a_obs;
    float lim = OBS_MAX_ACCEL_MPS2 * dt;
    if (dv > lim) dv = lim; else if (dv < -lim) dv = -lim;
    c->v_mps += dv;
    if (c->v_mps < 0.0f) c->v_mps = 0.0f;
    if (c->v_mps > OBS_MAX_SPEED_RATIO * c->v_start_mps) c->v_mps = OBS_MAX_SPEED_RATIO * c->v_start_mps;
    c->obs_samples++;
  }
  c->int_samples++;

  float chi = (c->psi_deg + c->beta_deg) * (float)DEG2RAD;
  c->n_m += (double)(c->v_mps * cosf(chi) * dt);
  c->e_m += (double)(c->v_mps * sinf(chi) * dt);
  c->dist_m += c->v_mps * dt;
  c->alt_m += c->v_mps * sinf(c->pitch_deg * (float)DEG2RAD) * dt;
}

void coast_imu(coast_t *c, uint32_t t_us, float yaw_deg, float roll_deg, float pitch_deg)
{
  if (c->imu_valid)
  {
    float dt = (float)(t_us - c->imu_t_us) * 1e-6f;
    if (dt > 0.0f && dt < 0.5f)
    {
      float rate = c->cfg.imu_yaw_sign * coast_wrap180(yaw_deg - c->yaw_deg) / dt;
      float alpha = dt / (dt + 0.05f);           // 50 ms low-pass
      c->yaw_rate_dps += (rate - c->yaw_rate_dps) * alpha;
      float rrate = c->cfg.imu_roll_sign * (roll_deg - c->roll_deg) / dt;
      c->roll_rate_dps += (rrate - c->roll_rate_dps) * alpha;

      c->yaw_deg = yaw_deg; c->roll_deg = roll_deg; c->pitch_deg = pitch_deg;
      if (c->active)
      {
        integrate(c, dt, coast_wrap360(c->cfg.imu_yaw_sign * yaw_deg + c->delta_frozen_deg),
                  c->cfg.imu_roll_sign * roll_deg, t_us / 1000u);
        c->t_last_us = t_us;
        c->imu_fallback = false;
      }
    }
    else if (dt >= 0.5f && c->active && !c->live)
    {
      c->active = false;   // IMU gap in a shadow window: abandon it silently (a live coast is handled by coast_tick)
    }
  }
  c->yaw_deg = yaw_deg; c->roll_deg = roll_deg; c->pitch_deg = pitch_deg;
  c->imu_t_us = t_us; c->imu_valid = true;
}

bool coast_predict_antenna(const coast_t *c, double *lat_deg, double *lon_deg)
{
  if (!c->active) return false;
  double s = sin(c->psi_deg * DEG2RAD), co = cos(c->psi_deg * DEG2RAD);
  double a = c->cfg.antenna_fwd_m;
  double hr = lateral_right(c, imu_roll(c));
  double e = c->e_m + a * s + hr * co;
  double n = c->n_m + a * co - hr * s;
  *lat_deg = c->lat0_deg + (n / c->rm_m) * RAD2DEG;
  *lon_deg = c->lon0_deg + (e / (c->rn_m * cos(c->lat0_deg * DEG2RAD))) * RAD2DEG;
  return true;
}

// ----------------------------------------------------------------------------- learning
static void learn_quadrant(coast_t *c, const coast_fix_t *fix)
{
  if (c->cfg.dual_heading_offset_deg != COAST_AUTO_OFFSET) return;
  if (fix->hdg_q != 3 || fix->v_mps < LEARN_MIN_SPEED_MPS) return;
  float r = coast_wrap180(fix->track_deg - fix->hdg_raw_deg);
  int q = (int)lroundf(r / 90.0f);
  q = ((q % 4) + 4) % 4;
  c->quad_votes[q]++;
  int best = 0;
  for (int i = 1; i < 4; i++) if (c->quad_votes[i] > c->quad_votes[best]) best = i;
  int total = c->quad_votes[0] + c->quad_votes[1] + c->quad_votes[2] + c->quad_votes[3];
  if (c->quad_votes[best] >= 20 && c->quad_votes[best] * 10 >= total * 8)
  {
    int off = best * 90;
    if (!c->quad_valid || off != c->quad_offset_deg) c->delta_valid = false;  // offset changed: relearn delta
    c->quad_offset_deg = off;
    c->quad_valid = true;
  }
}

static void learn_delta(coast_t *c, const coast_fix_t *fix)
{
  if (!c->quad_valid || !c->imu_valid || fix->hdg_q != 3) return;
  float veh = coast_vehicle_heading(c, fix->hdg_raw_deg);
  float raw = coast_wrap180(veh - imu_yaw(c));
  if (!c->delta_valid)
  {
    c->delta_deg = raw; c->delta_valid = true;
  }
  else
  {
    float dt = (float)(fix->t_ms - c->delta_t_ms) * 1e-3f;
    if (dt <= 0.0f || dt > 2.0f) dt = 0.1f;
    float alpha = dt / (dt + 2.0f);              // 2 s low-pass
    c->delta_deg = coast_wrap180(c->delta_deg + coast_wrap180(raw - c->delta_deg) * alpha);
  }
  c->delta_t_ms = fix->t_ms;
}

// WAS sign + effective wheelbase from turns: L = v * tan(delta) / yawrate
static void learn_wheelbase(coast_t *c, const coast_fix_t *fix)
{
  if (!c->imu_valid || !c->was_valid || fix->v_mps < LEARN_MIN_SPEED_MPS) return;
  if (fabsf(c->was_deg) <= TURN_MIN_WAS_DEG || fabsf(c->yaw_rate_dps) <= TURN_MIN_RATE_DPS) return;

  if (c->cfg.was_sign == 0.0f)
  {
    bool same = (c->was_deg > 0.0f) == (c->yaw_rate_dps > 0.0f);
    if (same) c->was_votes_pos++; else c->was_votes_neg++;
    int hi = c->was_votes_pos > c->was_votes_neg ? c->was_votes_pos : c->was_votes_neg;
    int total = c->was_votes_pos + c->was_votes_neg;
    if (hi >= WAS_SIGN_MIN_VOTES && hi * 10 >= total * 8)
    {
      float s = c->was_votes_pos >= c->was_votes_neg ? 1.0f : -1.0f;
      if (!c->was_sign_valid || s != c->was_sign) { c->leff_valid = false; c->leff_n = 0; }
      c->was_sign = s; c->was_sign_valid = true;
    }
  }
  if (!c->was_sign_valid) return;

  float d = c->was_sign * c->was_deg;
  float l_obs = axle_speed(c, fix->v_mps) * tanf(d * (float)DEG2RAD) / (c->yaw_rate_dps * (float)DEG2RAD);
  float L = c->cfg.wheelbase_m;
  if (!(l_obs > 0.3f * L && l_obs < 3.0f * L)) return;   // wrong sign or nonsense: skip

  if (c->leff_n == 0)
  {
    c->leff_m = l_obs;
  }
  else
  {
    float dt = (float)(fix->t_ms - c->leff_t_ms) * 1e-3f;
    if (dt <= 0.0f || dt > 2.0f) dt = 0.1f;
    float alpha = dt / (dt + LEFF_TAU_S);
    c->leff_m += (l_obs - c->leff_m) * alpha;
  }
  if (c->leff_m < 0.5f * L) c->leff_m = 0.5f * L;
  if (c->leff_m > 2.0f * L) c->leff_m = 2.0f * L;
  c->leff_t_ms = fix->t_ms;
  c->leff_n++;
  if (c->leff_n >= LEFF_MIN_SAMPLES) c->leff_valid = true;
}

// External speed scale: GNSS axle speed / raw pulse speed, 30 s LPF. Absorbs tyre radius, radar
// mounting angle and a wrong pulses-per-metre constant.
static void learn_ext_scale(coast_t *c, const coast_fix_t *fix)
{
  if (!c->cfg.ext_speed || !c->imu_valid || fix->v_mps < LEARN_MIN_SPEED_MPS) return;
  if (!ext_speed_fresh(c, fix->t_ms)) return;
  float raw_h = c->ext_v_raw_mps * cosf(c->pitch_deg * (float)DEG2RAD);
  if (raw_h < 0.5f) return;
  float ratio = axle_speed(c, fix->v_mps) / raw_h;
  if (!(ratio > 0.5f && ratio < 2.0f)) return;
  if (c->ext_scale_n == 0)
  {
    c->ext_scale = ratio;
  }
  else
  {
    float dt = (float)(fix->t_ms - c->ext_scale_t_ms) * 1e-3f;
    if (dt <= 0.0f || dt > 2.0f) dt = 0.1f;
    float alpha = dt / (dt + EXT_SCALE_TAU_S);
    c->ext_scale += (ratio - c->ext_scale) * alpha;
  }
  c->ext_scale_t_ms = fix->t_ms;
  c->ext_scale_n++;
  if (c->ext_scale_n >= EXT_SCALE_MIN_SAMPLES) c->ext_scale_valid = true;
}

// Low-passed slip crab (1 s): the KSXT track is noisy sample to sample, the crab is not.
static void update_slip(coast_t *c, const coast_fix_t *fix)
{
  if (!c->quad_valid || !c->imu_valid || fix->hdg_q != 3 || fix->v_mps < LEARN_MIN_SPEED_MPS) return;
  float raw = slip_crab(c, fix);
  if (!c->beta_slip_valid)
  {
    c->beta_slip_filt_deg = raw; c->beta_slip_valid = true;
  }
  else
  {
    float dt = (float)(fix->t_ms - c->beta_slip_t_ms) * 1e-3f;
    if (dt <= 0.0f || dt > 2.0f) dt = 0.1f;
    float alpha = dt / (dt + SLIP_TAU_S);
    c->beta_slip_filt_deg += coast_wrap180(raw - c->beta_slip_filt_deg) * alpha;
  }
  c->beta_slip_t_ms = fix->t_ms;
}

// Crab gain k = slip crab / sin(roll) on side slopes
static void learn_crab(coast_t *c, const coast_fix_t *fix)
{
  if (!c->beta_slip_valid || fix->hdg_q != 3 || fix->v_mps < LEARN_MIN_SPEED_MPS) return;
  float rollc = imu_roll(c);
  if (fabsf(rollc) <= CRAB_MIN_ROLL_DEG) return;
  if (fabsf(c->roll_rate_dps) > CRAB_MAX_ROLL_RATE_DPS) return;   // antenna swinging: track is not crab
  float k_obs = c->beta_slip_filt_deg / sinf(rollc * (float)DEG2RAD);
  if (k_obs > K_CRAB_LIMIT_DEG) k_obs = K_CRAB_LIMIT_DEG;
  if (k_obs < -K_CRAB_LIMIT_DEG) k_obs = -K_CRAB_LIMIT_DEG;

  if (c->k_n == 0)
  {
    c->k_crab = k_obs;
  }
  else
  {
    float dt = (float)(fix->t_ms - c->k_t_ms) * 1e-3f;
    if (dt <= 0.0f || dt > 2.0f) dt = 0.1f;
    float alpha = dt / (dt + K_TAU_S);
    c->k_crab += (k_obs - c->k_crab) * alpha;
  }
  c->k_t_ms = fix->t_ms;
  c->k_n++;
  if (c->k_n >= K_MIN_SAMPLES) c->k_valid = true;
}

// ----------------------------------------------------------------------------- integrator windows
static void start_window(coast_t *c, const coast_fix_t *fix)
{
  c->delta_frozen_deg = c->delta_deg;
  c->psi_deg = coast_wrap360(imu_yaw(c) + c->delta_frozen_deg);
  c->v_mps = ext_speed_fresh(c, fix->t_ms) ? ext_speed_h(c) : axle_speed(c, fix->v_mps);
  c->v_start_mps = c->v_mps;

  c->beta_entry_deg = c->beta_slip_valid ? c->beta_slip_filt_deg : slip_crab(c, fix);
  if (c->cfg.crab_model && c->k_valid)
    c->beta_res_deg = c->beta_entry_deg - c->k_crab * sinf(imu_roll(c) * (float)DEG2RAD);
  else
    c->beta_res_deg = c->beta_entry_deg;
  c->beta_deg = c->beta_entry_deg;

  c->lat0_deg = fix->lat_deg; c->lon0_deg = fix->lon_deg;
  c->alt_m = fix->alt_m;
  coast_earth_radii(c->lat0_deg, &c->rm_m, &c->rn_m);
  axle_from_antenna(c, c->psi_deg, imu_roll(c), &c->n_m, &c->e_m);

  c->dist_m = 0.0f;
  c->t_start_ms = fix->t_ms;
  c->t_last_us = c->imu_t_us;
  c->along_m = c->cross_m = 0.0f;
  c->max_abs_along_m = c->max_abs_cross_m = 0.0f;
  c->fix_count = 0;
  c->int_samples = c->obs_samples = c->ext_samples = 0;
  c->active = true;
  c->live = false;
}

static void compare_fix(coast_t *c, const coast_fix_t *fix)
{
  double plat, plon;
  if (!coast_predict_antenna(c, &plat, &plon)) return;
  double dn = (fix->lat_deg - plat) * DEG2RAD * c->rm_m;
  double de = (fix->lon_deg - plon) * DEG2RAD * c->rn_m * cos(c->lat0_deg * DEG2RAD);
  double s = sin(c->psi_deg * DEG2RAD), co = cos(c->psi_deg * DEG2RAD);
  c->along_m = (float)(de * s + dn * co);
  c->cross_m = (float)(de * co - dn * s);
  if (fabsf(c->along_m) > c->max_abs_along_m) c->max_abs_along_m = fabsf(c->along_m);
  if (fabsf(c->cross_m) > c->max_abs_cross_m) c->max_abs_cross_m = fabsf(c->cross_m);
  c->fix_count++;
}

static void finish_window(coast_t *c, const coast_fix_t *fix)
{
  coast_report_t *r = &c->report;
  r->duration_ms = fix->t_ms - c->t_start_ms;
  r->dist_m = c->dist_m;
  r->along_m = c->along_m; r->cross_m = c->cross_m;
  r->max_abs_along_m = c->max_abs_along_m; r->max_abs_cross_m = c->max_abs_cross_m;
  r->delta_deg = c->delta_frozen_deg;
  r->offset_deg = c->quad_offset_deg;
  r->beta_deg = c->beta_entry_deg;
  r->v_start_mps = c->v_start_mps; r->v_end_mps = fix->v_mps;
  r->fix_count = c->fix_count;
  r->leff_m = c->leff_valid ? c->leff_m : 0.0f;
  r->k_crab = c->k_valid ? c->k_crab : 0.0f;
  r->was_sign = c->was_sign_valid ? c->was_sign : 0.0f;
  r->observer_frac = c->int_samples ? (float)c->obs_samples / (float)c->int_samples : 0.0f;
  r->ext_frac = c->int_samples ? (float)c->ext_samples / (float)c->int_samples : 0.0f;
  r->ext_scale = c->ext_scale_valid ? c->ext_scale : 0.0f;
  c->report_ready = true;
  c->active = false;
}

// ----------------------------------------------------------------------------- live coast
static void apply_geometry(coast_t *c, float L, float a, float h, float o);

static void end_live(coast_t *c, uint32_t now_ms, int reason)
{
  coast_live_report_t *r = &c->live_report;
  r->reason = reason;
  r->duration_ms = now_ms - c->t_start_ms;
  r->dist_m = c->dist_m;
  r->has_error = c->fix_count > 0;
  r->along_m = c->along_m; r->cross_m = c->cross_m;
  r->v_start_mps = c->v_start_mps;
  r->forced = c->live_forced;
  r->used_fallback = c->live_used_fallback;
  c->live_report_ready = true;
  c->q0_pending = (reason == COAST_END_TIMEOUT || reason == COAST_END_SENSOR);
  c->live = false;
  c->active = false;
  c->imu_fallback = false;
  if (c->geom_pending)
  {
    c->geom_pending = false;
    apply_geometry(c, c->geom_L, c->geom_a, c->geom_h, c->geom_o);
  }
}

// Start a live coast from the last RTK fix, if everything needed is fresh.
static void try_start_live(coast_t *c, uint32_t now_ms)
{
  if (!c->cfg.live_enable || c->live) return;
  if (!c->last_valid || (int32_t)(now_ms - c->last.t_ms) > LIVE_MAX_FIX_AGE_MS) return;
  if (!c->imu_valid || (int32_t)(now_ms - c->imu_t_us / 1000u) > LIVE_MAX_IMU_AGE_MS) return;
  if (!c->delta_valid || !c->quad_valid) return;

  start_window(c, &c->last);          // origin = last RTK fix, heading from the IMU now
  c->live = true;
  c->live_good = 0;
  c->live_forced = c->forced;
  c->live_used_fallback = false;
  c->imu_fallback = false;
  c->roll_at_loss_deg = c->roll_deg;
  c->dual_roll_at_loss_deg = c->last.dual_roll_deg;
  c->tick_t_us = c->imu_t_us; c->tick_valid = true;

  // motion since that fix (up to LIVE_MAX_FIX_AGE_MS): straight at the current heading
  float gap = (float)(int32_t)(now_ms - c->last.t_ms) * 1e-3f;
  if (gap > 0.0f) integrate(c, gap, c->psi_deg, imu_roll(c), now_ms);
}

static void apply_geometry(coast_t *c, float L, float a, float h, float o)
{
  bool L_changed = fabsf(L - c->cfg.wheelbase_m) > 0.01f;
  c->cfg.wheelbase_m = L;
  c->cfg.antenna_fwd_m = a;
  c->cfg.antenna_height_m = h;
  c->cfg.antenna_right_m = o;
  if (L_changed)
  {
    // L_eff is learned around L and clamped to it: start again
    c->leff_valid = false; c->leff_n = 0;
  }
  // the antenna geometry enters the crab and slip estimates: relearn them
  c->beta_slip_valid = false; c->k_valid = false; c->k_n = 0;
  c->ext_scale_valid = false; c->ext_scale_n = 0;
  if (c->active && !c->live) c->active = false;   // shadow window no longer consistent
}

bool coast_set_geometry(coast_t *c, float wheelbase_m, float antenna_fwd_m, float antenna_height_m, float antenna_right_m)
{
  if (!(wheelbase_m > 0.5f && wheelbase_m < 10.0f)) return false;
  if (!(antenna_fwd_m > -10.0f && antenna_fwd_m < 10.0f)) return false;
  if (!(antenna_height_m >= 0.0f && antenna_height_m < 10.0f)) return false;
  if (!(antenna_right_m > -5.0f && antenna_right_m < 5.0f)) return false;
  if (c->live)
  {
    c->geom_pending = true;
    c->geom_L = wheelbase_m; c->geom_a = antenna_fwd_m; c->geom_h = antenna_height_m; c->geom_o = antenna_right_m;
    return true;
  }
  apply_geometry(c, wheelbase_m, antenna_fwd_m, antenna_height_m, antenna_right_m);
  return true;
}

void coast_force(coast_t *c, uint32_t now_ms, float seconds)
{
  if (seconds <= 0.0f) { c->forced = false; return; }
  c->forced = true;
  c->force_until_ms = now_ms + (uint32_t)(seconds * 1000.0f);
}

void coast_tick(coast_t *c, uint32_t t_us)
{
  uint32_t now_ms = t_us / 1000u;

  // receiver gone quiet: treat as a loss
  if (c->cfg.live_enable && c->last_any_valid && !c->gnss_lost &&
      (int32_t)(now_ms - c->last_any_ms) > LIVE_SILENCE_MS)
  {
    c->gnss_lost = true;
    if (!c->live) try_start_live(c, now_ms);
  }

  if (c->live)
  {
    // IMU stale: carry the heading with the wheel model, integrate on the tick
    if ((int32_t)(t_us - c->imu_t_us) > LIVE_IMU_STALE_US)
    {
      if (!c->was_valid || !c->was_sign_valid)
      {
        end_live(c, now_ms, COAST_END_SENSOR);
      }
      else if (c->tick_valid)
      {
        float dt = (float)(t_us - c->tick_t_us) * 1e-6f;
        if (dt > 0.0f && dt < 0.5f)
        {
          float L = c->leff_valid ? c->leff_m : c->cfg.wheelbase_m;
          float d = c->was_sign * c->was_deg;
          float rate_dps = c->v_mps * tanf(d * (float)DEG2RAD) / L * (float)RAD2DEG;
          c->yaw_rate_dps = rate_dps;
          integrate(c, dt, coast_wrap360(c->psi_deg + rate_dps * dt), imu_roll(c), now_ms);
          c->imu_fallback = true;
          c->live_used_fallback = true;
        }
      }
    }
    if (c->live && ((float)(int32_t)(now_ms - c->t_start_ms) * 1e-3f > c->cfg.live_max_s ||
                    c->dist_m > c->cfg.live_max_m))
    {
      end_live(c, now_ms, COAST_END_TIMEOUT);
    }
  }
  c->tick_t_us = t_us; c->tick_valid = true;
}

bool coast_live_output(const coast_t *c, uint32_t now_ms, coast_out_t *out)
{
  if (!c->live) return false;
  if (!coast_predict_antenna(c, &out->lat_deg, &out->lon_deg)) return false;
  out->alt_m = c->alt_m;
  out->heading_deg = c->psi_deg;
  out->track_deg = coast_wrap360(c->psi_deg + c->beta_deg);
  out->roll_deg = c->roll_deg;
  out->pitch_deg = c->pitch_deg;
  out->v_mps = c->v_mps;
  out->elapsed_ms = now_ms - c->t_start_ms;
  out->hdop = LIVE_HDOP_BASE + LIVE_HDOP_PER_S * (float)out->elapsed_ms * 1e-3f;
  out->dist_m = c->dist_m;
  out->imu_fallback = c->imu_fallback;
  return true;
}

// ----------------------------------------------------------------------------- GNSS event
void coast_gnss(coast_t *c, const coast_fix_t *fix)
{
  c->last_any_ms = fix->t_ms; c->last_any_valid = true;

  bool forced_now = c->forced && (int32_t)(c->force_until_ms - fix->t_ms) > 0;
  if (c->forced && !forced_now) c->forced = false;
  if (forced_now && !c->cfg.live_enable) { c->forced = false; forced_now = false; }

  bool good = (fix->pos_q == 3);
  bool flt  = (fix->pos_q == 2) && !c->cfg.live_on_float;
  bool lost = !good && !flt;

  if (good && forced_now)
  {
    // field test: the fix is real, so measure against it, but behave as if it were lost
    if (c->live) compare_fix(c, fix);
    good = false; lost = true;
  }

  if (good)
  {
    c->gnss_lost = false;
    learn_quadrant(c, fix);
    learn_delta(c, fix);
    learn_wheelbase(c, fix);
    learn_ext_scale(c, fix);
    update_slip(c, fix);
    learn_crab(c, fix);

    if (c->live)
    {
      c->live_good++;
      compare_fix(c, fix);
      if (c->live_good >= LIVE_RECOVER_FIXES) end_live(c, fix->t_ms, COAST_END_RECOVERED);
    }
    else if (c->active)
    {
      compare_fix(c, fix);
      if ((float)(fix->t_ms - c->t_start_ms) * 1e-3f >= c->cfg.shadow_window_s) finish_window(c, fix);
    }

    c->last = *fix; c->last_valid = true;

    if (!c->active && !c->live && c->delta_valid && c->quad_valid && c->imu_valid &&
        fix->v_mps >= c->cfg.min_speed_mps)
    {
      start_window(c, fix);
    }
  }
  else if (flt)
  {
    // float: position exists but is not good enough to start or end a coast
    c->gnss_lost = false;
    if (c->live) c->live_good = 0;
    else if (c->active && c->last_valid && (fix->t_ms - c->last.t_ms) > 1000) c->active = false;
  }
  else if (lost)
  {
    c->gnss_lost = true;
    if (c->live)
    {
      c->live_good = 0;
    }
    else
    {
      if (c->active && c->last_valid && (fix->t_ms - c->last.t_ms) > 1000) c->active = false;
      try_start_live(c, fix->t_ms);
    }
  }
}
