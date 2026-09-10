// Dead-reckoning coast core (Phase 0 shadow estimator + Phase 2 curve/slope models).
// Pure C11, no Arduino dependencies, no heap. Also compiled by the host test in
// Firmware/tests/coast/. See Firmware/docs/dead_reckoning_coast_design.md.
//
// Frames: N/E metres, heading in degrees clockwise from north.
//   fwd(psi)   = (E: sin psi, N: cos psi)
//   right(psi) = (E: cos psi, N: -sin psi)
// Antenna = axle + a*fwd(psi) + h*sin(roll)*right(psi), roll positive right-side-down.
//
// Phase 2 additions, all learned while GNSS is good and used while integrating:
//   - WAS sign and effective wheelbase L_eff from v*tan(delta)/yawrate on turns
//   - speed observer on turns: v_obs = yawrate * L_eff / tan(delta)
//   - crab gain k [deg per unit sin(roll)]: slip crab = k*sin(roll) + residual
#ifndef ZCOASTCORE_H
#define ZCOASTCORE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COAST_AUTO_OFFSET 999.0f   // dual_heading_offset_deg: learn the 0/90/180/270 quadrant from track vs heading

typedef struct {
  float wheelbase_m;             // L nominal (L_eff is learned around it)
  float antenna_fwd_m;           // a, antenna ahead of the rear axle (= AOG antenna pivot)
  float antenna_height_m;        // h (= AOG antenna height)
  float imu_yaw_sign;            // +1 if TM171 yaw grows clockwise like a compass heading, else -1
  float imu_roll_sign;           // +1 if TM171 roll is positive right-side-down (AOG convention), else -1
  float dual_heading_offset_deg; // added to KSXT heading to get vehicle heading, or COAST_AUTO_OFFSET
  float shadow_window_s;         // length of each shadow window (design: 20 s)
  float min_speed_mps;           // do not start a window below this speed
  float was_sign;                // +1 if positive WAS = right turn, -1 if left, 0 = learn from yaw rate
  bool  speed_observer;          // Phase 2: use yawrate*L_eff/tan(delta) to track speed on turns
  bool  crab_model;              // Phase 2: slip crab follows k*sin(roll)
} coast_config_t;

typedef struct {
  uint32_t t_ms;
  double   lat_deg, lon_deg;
  float    alt_m;
  int      pos_q;        // KSXT position quality: 0 none, 1 single, 2 float, 3 fixed
  int      hdg_q;        // KSXT heading quality, same codes
  float    hdg_raw_deg;  // KSXT heading (master -> slave antenna)
  float    track_deg;    // KSXT course over ground
  float    v_mps;        // KSXT horizontal speed converted to m/s
  float    dual_roll_deg;// KSXT "pitch" field, which AgIO uses as roll for left/right antennas
} coast_fix_t;

typedef struct {
  uint32_t duration_ms;
  float    dist_m;
  float    along_m, cross_m;            // final error (fix - prediction), along = +ahead, cross = +right
  float    max_abs_along_m, max_abs_cross_m;
  float    delta_deg;                   // heading offset used
  int      offset_deg;                  // dual heading quadrant offset used
  float    beta_deg;                    // slip crab measured at window start
  float    v_start_mps, v_end_mps;      // speed at start vs true speed at the end of the window
  uint32_t fix_count;
  float    leff_m;                      // effective wheelbase in use (0 if not learned)
  float    k_crab;                      // crab gain in use, deg per unit sin(roll) (0 if not learned)
  float    was_sign;                    // WAS sign in use (0 if not learned)
  float    observer_frac;               // fraction of integration steps where the speed observer was active
} coast_report_t;

typedef struct {
  coast_config_t cfg;

  // learned while GNSS is good
  float    delta_deg;    bool delta_valid;  uint32_t delta_t_ms;
  int      quad_votes[4];
  int      quad_offset_deg; bool quad_valid;
  int      was_votes_pos, was_votes_neg; float was_sign; bool was_sign_valid;
  float    leff_m;       bool leff_valid;   int leff_n;    uint32_t leff_t_ms;
  float    k_crab;       bool k_valid;      int k_n;       uint32_t k_t_ms;
  float    beta_slip_filt_deg; bool beta_slip_valid; uint32_t beta_slip_t_ms;   // 1 s low-passed slip crab

  // IMU
  float    yaw_deg, roll_deg, pitch_deg;  // raw TM171 values as received
  float    yaw_rate_dps;                  // sign-corrected, low-passed
  float    roll_rate_dps;                 // sign-corrected, low-passed
  uint32_t imu_t_us;      bool imu_valid;

  // WAS
  float    was_deg;       bool was_valid;

  // last good fix
  coast_fix_t last;       bool last_valid;

  // integrator (shadow now, live coast in Phase 1)
  bool     active;
  double   n_m, e_m;          // rear axle relative to the start antenna position
  double   lat0_deg, lon0_deg;// start antenna position
  double   rm_m, rn_m;        // earth radii at lat0
  float    psi_deg;           // current vehicle heading estimate
  float    v_mps;             // speed estimate (held, or observed on turns)
  float    v_start_mps;
  float    beta_deg;          // current crab in use
  float    beta_res_deg;      // residual crab at window start (after removing k*sin(roll))
  float    beta_entry_deg;    // measured slip crab at window start
  float    delta_frozen_deg;
  float    dist_m;
  uint32_t t_start_ms;
  uint32_t t_last_us;
  uint32_t int_samples, obs_samples;

  // shadow statistics
  float    along_m, cross_m, max_abs_along_m, max_abs_cross_m;
  uint32_t fix_count;
  coast_report_t report;  bool report_ready;
} coast_t;

void  coast_init(coast_t *c, const coast_config_t *cfg);
void  coast_imu(coast_t *c, uint32_t t_us, float yaw_deg, float roll_deg, float pitch_deg);
void  coast_was(coast_t *c, float steer_deg);
void  coast_gnss(coast_t *c, const coast_fix_t *fix);
bool  coast_predict_antenna(const coast_t *c, double *lat_deg, double *lon_deg);
float coast_vehicle_heading(const coast_t *c, float hdg_raw_deg);   // KSXT heading + offset, or <0 if unknown

float coast_wrap180(float a);
float coast_wrap360(float a);
void  coast_earth_radii(double lat_deg, double *rm_m, double *rn_m);

#ifdef __cplusplus
}
#endif
#endif
