// Dead-reckoning coast core (Phase 0). See zCoastCore.h and the design doc.
#include "zCoastCore.h"
#include <math.h>
#include <string.h>

#define DEG2RAD 0.017453292519943295
#define RAD2DEG 57.29577951308232
#define WGS84_A 6378137.0
#define WGS84_E2 0.00669437999014

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
  if (c->cfg.dual_heading_offset_deg != COAST_AUTO_OFFSET)
  {
    c->quad_offset_deg = (int)lroundf(c->cfg.dual_heading_offset_deg);
    c->quad_valid = true;
  }
}

float coast_vehicle_heading(const coast_t *c, float hdg_raw_deg)
{
  if (!c->quad_valid) return -1.0f;
  return coast_wrap360(hdg_raw_deg + (float)c->quad_offset_deg);
}

// Rear axle offset from the antenna, in the current heading/roll: axle = antenna - a*fwd - h*sin(roll)*right
static void axle_from_antenna(const coast_t *c, float psi_deg, float roll_deg, double *dn, double *de)
{
  double s = sin(psi_deg * DEG2RAD), co = cos(psi_deg * DEG2RAD);
  double a = c->cfg.antenna_fwd_m;
  double hr = c->cfg.antenna_height_m * sin(roll_deg * DEG2RAD);
  *de = -a * s - hr * co;
  *dn = -a * co + hr * s;
}

static float imu_roll(const coast_t *c) { return c->cfg.imu_roll_sign * c->roll_deg; }
static float imu_yaw(const coast_t *c)  { return c->cfg.imu_yaw_sign * c->yaw_deg; }

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

      if (c->active)
      {
        c->psi_deg = coast_wrap360(c->cfg.imu_yaw_sign * yaw_deg + c->delta_frozen_deg);
        float chi = (c->psi_deg + c->beta_deg) * (float)DEG2RAD;
        c->n_m += (double)(c->v_mps * cosf(chi) * dt);
        c->e_m += (double)(c->v_mps * sinf(chi) * dt);
        c->dist_m += c->v_mps * dt;
        c->t_last_us = t_us;
      }
    }
    else if (dt >= 0.5f && c->active)
    {
      c->active = false;   // IMU gap: abandon the window silently
    }
  }
  c->yaw_deg = yaw_deg; c->roll_deg = roll_deg; c->pitch_deg = pitch_deg;
  c->imu_t_us = t_us; c->imu_valid = true;
}

void coast_was(coast_t *c, float steer_deg)
{
  c->was_deg = steer_deg; c->was_valid = true;
}

bool coast_predict_antenna(const coast_t *c, double *lat_deg, double *lon_deg)
{
  if (!c->active) return false;
  double s = sin(c->psi_deg * DEG2RAD), co = cos(c->psi_deg * DEG2RAD);
  double a = c->cfg.antenna_fwd_m;
  double hr = c->cfg.antenna_height_m * sin(imu_roll(c) * DEG2RAD);
  double e = c->e_m + a * s + hr * co;
  double n = c->n_m + a * co - hr * s;
  *lat_deg = c->lat0_deg + (n / c->rm_m) * RAD2DEG;
  *lon_deg = c->lon0_deg + (e / (c->rn_m * cos(c->lat0_deg * DEG2RAD))) * RAD2DEG;
  return true;
}

static void learn_quadrant(coast_t *c, const coast_fix_t *fix)
{
  if (c->cfg.dual_heading_offset_deg != COAST_AUTO_OFFSET) return;
  if (fix->hdg_q != 3 || fix->v_mps < 1.0f) return;
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

static void start_window(coast_t *c, const coast_fix_t *fix)
{
  c->delta_frozen_deg = c->delta_deg;
  c->psi_deg = coast_wrap360(imu_yaw(c) + c->delta_frozen_deg);
  c->v_mps = fix->v_mps;

  // slip crab = (track - vehicle heading) - geometric crab of the antenna while turning
  float veh = coast_vehicle_heading(c, fix->hdg_raw_deg);
  float beta_total = coast_wrap180(fix->track_deg - veh);
  float beta_kin = (float)(atan2((double)(c->cfg.antenna_fwd_m * c->yaw_rate_dps * (float)DEG2RAD), (double)fix->v_mps) * RAD2DEG);
  c->beta_deg = coast_wrap180(beta_total - beta_kin);

  c->lat0_deg = fix->lat_deg; c->lon0_deg = fix->lon_deg;
  coast_earth_radii(c->lat0_deg, &c->rm_m, &c->rn_m);
  axle_from_antenna(c, c->psi_deg, imu_roll(c), &c->n_m, &c->e_m);

  c->dist_m = 0.0f;
  c->t_start_ms = fix->t_ms;
  c->t_last_us = c->imu_t_us;
  c->along_m = c->cross_m = 0.0f;
  c->max_abs_along_m = c->max_abs_cross_m = 0.0f;
  c->fix_count = 0;
  c->active = true;
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
  r->beta_deg = c->beta_deg;
  r->v_start_mps = c->v_mps; r->v_end_mps = fix->v_mps;
  r->fix_count = c->fix_count;
  c->report_ready = true;
  c->active = false;
}

void coast_gnss(coast_t *c, const coast_fix_t *fix)
{
  bool good = (fix->pos_q == 3);
  if (!good)
  {
    // Phase 0: a real loss just interrupts the shadow window. Abandon it if the gap grows.
    if (c->active && c->last_valid && (fix->t_ms - c->last.t_ms) > 1000) c->active = false;
    return;
  }

  learn_quadrant(c, fix);
  learn_delta(c, fix);

  if (c->active)
  {
    compare_fix(c, fix);
    if ((float)(fix->t_ms - c->t_start_ms) * 1e-3f >= c->cfg.shadow_window_s) finish_window(c, fix);
  }

  c->last = *fix; c->last_valid = true;

  if (!c->active && c->delta_valid && c->quad_valid && c->imu_valid && fix->v_mps >= c->cfg.min_speed_mps)
  {
    start_window(c, fix);
  }
}
