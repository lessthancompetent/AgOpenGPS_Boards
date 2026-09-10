// Host test for zCoastCore.c: replays a CSV log (see coast_ref.py) and prints shadow reports in the same
// format as the Python reference model, so the two can be diffed. With --live it also exercises the live
// coast against withheld ground truth.
//   see run_tests.sh in this folder.
//   Flags: --no-observer --no-crab --no-ext
//          --live                 enable the live coast (20 s / 60 m caps)
//          --outage T0 T1         GNSS rows in [T0,T1) s report quality 0 (position withheld as truth)
//          --silence              with --outage: drop the rows entirely instead (receiver goes quiet)
//          --force T S            call coast_force at T s for S s (GNSS stays good)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "zCoastCore.h"

int main(int argc, char **argv)
{
  if (argc < 2) { fprintf(stderr, "usage: %s log.csv [flags]\n", argv[0]); return 2; }
  FILE *f = fopen(argv[1], "r");
  if (!f) { perror(argv[1]); return 2; }

  coast_config_t cfg;
  memset(&cfg, 0, sizeof cfg);
  cfg.wheelbase_m = 2.6f; cfg.antenna_fwd_m = 1.2f; cfg.antenna_height_m = 2.8f; cfg.antenna_right_m = 0.35f;
  cfg.imu_yaw_sign = 1.0f; cfg.imu_roll_sign = 1.0f; cfg.dual_heading_offset_deg = COAST_AUTO_OFFSET;
  cfg.shadow_window_s = 20.0f; cfg.min_speed_mps = 0.5f; cfg.was_sign = 0.0f;
  cfg.speed_observer = true; cfg.crab_model = true; cfg.ext_speed = true;
  cfg.live_enable = false; cfg.live_max_s = 20.0f; cfg.live_max_m = 60.0f; cfg.live_on_float = false;

  double out_t0 = -1, out_t1 = -1; bool silence = false; double force_t = -1; float force_s = 0;
  for (int i = 2; i < argc; i++)
  {
    if (!strcmp(argv[i], "--no-observer")) cfg.speed_observer = false;
    else if (!strcmp(argv[i], "--no-crab")) cfg.crab_model = false;
    else if (!strcmp(argv[i], "--no-ext")) cfg.ext_speed = false;
    else if (!strcmp(argv[i], "--live")) cfg.live_enable = true;
    else if (!strcmp(argv[i], "--silence")) silence = true;
    else if (!strcmp(argv[i], "--outage") && i + 2 < argc) { out_t0 = atof(argv[i + 1]); out_t1 = atof(argv[i + 2]); i += 2; }
    else if (!strcmp(argv[i], "--force") && i + 2 < argc) { force_t = atof(argv[i + 1]); force_s = (float)atof(argv[i + 2]); i += 2; }
  }
  coast_t c;
  coast_init(&c, &cfg);

  char line[512];
  int reports = 0;
  bool forced_done = false;
  double max_along = 0, max_cross = 0; int live_samples = 0; double last_print_s = -10;
  while (fgets(line, sizeof line, f))
  {
    char *p[16]; int n = 0;
    for (char *tok = strtok(line, ",\r\n"); tok && n < 16; tok = strtok(NULL, ",\r\n")) p[n++] = tok;
    if (n < 2) continue;
    uint32_t t_us = (uint32_t)strtoul(p[0], NULL, 10);
    double t_s = t_us * 1e-6;

    if (force_t >= 0 && !forced_done && t_s >= force_t) { coast_force(&c, t_us / 1000u, force_s); forced_done = true; }

    if (p[1][0] == 'I' && n >= 5)
    {
      coast_imu(&c, t_us, strtof(p[2], NULL), strtof(p[3], NULL), strtof(p[4], NULL));
      coast_tick(&c, t_us);
    }
    else if (p[1][0] == 'W' && n >= 3)
    {
      coast_was(&c, strtof(p[2], NULL));
    }
    else if (p[1][0] == 'S' && n >= 3)
    {
      coast_ext_speed(&c, t_us / 1000u, strtof(p[2], NULL));
    }
    else if (p[1][0] == 'G' && n >= 11)
    {
      coast_fix_t fx;
      fx.t_ms = t_us / 1000u;
      fx.lat_deg = strtod(p[2], NULL); fx.lon_deg = strtod(p[3], NULL); fx.alt_m = strtof(p[4], NULL);
      fx.pos_q = atoi(p[5]); fx.hdg_q = atoi(p[6]);
      fx.hdg_raw_deg = strtof(p[7], NULL); fx.track_deg = strtof(p[8], NULL);
      fx.v_mps = strtof(p[9], NULL); fx.dual_roll_deg = strtof(p[10], NULL);

      bool withheld = (out_t0 >= 0 && t_s >= out_t0 && t_s < out_t1);
      double true_lat = fx.lat_deg, true_lon = fx.lon_deg;
      if (withheld)
      {
        if (!silence)
        {
          coast_fix_t lost = fx;
          lost.pos_q = 0; lost.hdg_q = 0; lost.lat_deg = 0; lost.lon_deg = 0; lost.v_mps = 0;
          coast_gnss(&c, &lost);
        }
      }
      else
      {
        coast_gnss(&c, &fx);
      }
      coast_tick(&c, t_us);

      // truth comparison while the live coast runs (truth is available for withheld rows and forced coasts)
      coast_out_t o;
      if (c.live && coast_live_output(&c, t_us / 1000u, &o) && (withheld || c.forced))
      {
        double rm, rn; coast_earth_radii(true_lat, &rm, &rn);
        double dn = (true_lat - o.lat_deg) * 0.017453292519943295 * rm;
        double de = (true_lon - o.lon_deg) * 0.017453292519943295 * rn * cos(true_lat * 0.017453292519943295);
        double s = sin(o.heading_deg * 0.017453292519943295), co = cos(o.heading_deg * 0.017453292519943295);
        double along = de * s + dn * co, cross = de * co - dn * s;
        if (fabs(along) > max_along) max_along = fabs(along);
        if (fabs(cross) > max_cross) max_cross = fabs(cross);
        live_samples++;
        if (t_s - last_print_s >= 1.0)
        {
          printf("LIVE,%.1f,%.3f,%.3f,%.1f,%.1f,%d\n", t_s, along, cross, o.elapsed_ms * 1e-3, o.dist_m, o.imu_fallback ? 1 : 0);
          last_print_s = t_s;
        }
      }

      if (c.report_ready)
      {
        coast_report_t *r = &c.report;
        printf("REPORT,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%d,%.2f,%.3f,%.3f,%u,%.3f,%.3f,%.0f,%.3f,%.3f,%.4f\n",
               (unsigned)r->duration_ms, r->dist_m, r->along_m, r->cross_m, r->max_abs_along_m, r->max_abs_cross_m,
               r->delta_deg, r->offset_deg, r->beta_deg, r->v_start_mps, r->v_end_mps, (unsigned)r->fix_count,
               r->leff_m, r->k_crab, r->was_sign, r->observer_frac, r->ext_frac, r->ext_scale);
        c.report_ready = false;
        reports++;
      }
    }

    if (c.live_report_ready)
    {
      coast_live_report_t *r = &c.live_report;
      printf("LIVEREPORT,%d,%u,%.1f,%.3f,%.3f,%d,%d,%.3f,%.3f,%d\n", r->reason, (unsigned)r->duration_ms, r->dist_m,
             r->has_error ? r->along_m : 0.0f, r->has_error ? r->cross_m : 0.0f,
             r->forced ? 1 : 0, r->used_fallback ? 1 : 0, max_along, max_cross, live_samples);
      c.live_report_ready = false;
      max_along = max_cross = 0; live_samples = 0;
    }
    if (c.q0_pending) { printf("Q0SENT,%.1f\n", t_s); c.q0_pending = false; }
  }
  fclose(f);
  return reports > 0 ? 0 : 1;
}
