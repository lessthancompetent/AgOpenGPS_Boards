#!/usr/bin/env bash
# Cross-checks zCoastCore.c against the Python reference model on a synthetic sloped-curve drive.
# Needs: python3, gcc. Run from this folder.
set -e
SK=../../Autosteer_gps_teensy_AiO_Micro_v45
python coast_ref.py gen scenario.csv
python coast_ref.py run scenario.csv > py.out
gcc -std=c11 -O2 -Wall -Wextra -I"$SK" coast_host_test.c "$SK/zCoastCore.c" -o coast_host_test -lm
./coast_host_test scenario.csv > c.out
echo "== python =="; cat py.out
echo "== c =="; cat c.out
# Compare numerically: every field within 2 mm / 0.01 deg.
python - <<'EOF'
import sys
py = [l.split(',') for l in open('py.out') if l.startswith('REPORT')]
c  = [l.split(',') for l in open('c.out')  if l.startswith('REPORT')]
assert len(py) == len(c) and len(py) > 0, "report count mismatch"
for a, b in zip(py, c):
    for x, y in zip(a[1:], b[1:]):
        assert abs(float(x) - float(y)) <= 0.002 + 1e-6, f"mismatch {x} vs {y} in {a} / {b}"
print(f"OK: {len(py)} shadow reports match between Python and C")
EOF
