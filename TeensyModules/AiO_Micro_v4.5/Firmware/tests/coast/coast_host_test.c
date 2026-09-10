// Host test for zCoastCore.c: replays a CSV log (see coast_ref.py) and prints shadow reports
// in the same format as the Python reference model, so the two can be diffed.
//
//   see run_tests.sh in this folder
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "zCoastCore.h"

int main(int argc, char **argv)
{
  if (argc < 2) { fprintf(stderr, "usage: %s log.csv\n", argv[0]); return 2; }
  FILE *f = fopen(argv[1], "r");
  if (!f) { perror(argv[1]); return 2; }

  coast_config_t cfg = {
    .wheelbase_m = 2.6f, .antenna_fwd_m = 1.2f, .antenna_height_m = 2.8f,
    .imu_yaw_sign = 1.0f, .imu_roll_sign = 1.0f, .dual_heading_offset_deg = COAST_AUTO_OFFSET,
    .shadow_window_s = 20.0f, .min_speed_mps = 0.5f,
  };
  coast_t c;
  coast_init(&c, &cfg);

  char line[512];
  int reports = 0;
  while (fgets(line, sizeof line, f))
  {
    char *p[16]; int n = 0;
    for (char *tok = strtok(line, ",\r\n"); tok && n < 16; tok = strtok(NULL, ",\r\n")) p[n++] = tok;
    if (n < 2) continue;
    uint32_t t_us = (uint32_t)strtoul(p[0], NULL, 10);
    if (p[1][0] == 'I' && n >= 5)
    {
      coast_imu(&c, t_us, strtof(p[2], NULL), strtof(p[3], NULL), strtof(p[4], NULL));
    }
    else if (p[1][0] == 'W' && n >= 3)
    {
      coast_was(&c, strtof(p[2], NULL));
    }
    else if (p[1][0] == 'G' && n >= 11)
    {
      coast_fix_t fx = {
        .t_ms = t_us / 1000u,
        .lat_deg = strtod(p[2], NULL), .lon_deg = strtod(p[3], NULL), .alt_m = strtof(p[4], NULL),
        .pos_q = atoi(p[5]), .hdg_q = atoi(p[6]),
        .hdg_raw_deg = strtof(p[7], NULL), .track_deg = strtof(p[8], NULL),
        .v_mps = strtof(p[9], NULL), .dual_roll_deg = strtof(p[10], NULL),
      };
      coast_gnss(&c, &fx);
      if (c.report_ready)
      {
        coast_report_t *r = &c.report;
        printf("REPORT,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%d,%.2f,%.3f,%.3f,%u\n",
               (unsigned)r->duration_ms, r->dist_m, r->along_m, r->cross_m, r->max_abs_along_m, r->max_abs_cross_m,
               r->delta_deg, r->offset_deg, r->beta_deg, r->v_start_mps, r->v_end_mps, (unsigned)r->fix_count);
        c.report_ready = false;
        reports++;
      }
    }
  }
  fclose(f);
  return reports > 0 ? 0 : 1;
}
