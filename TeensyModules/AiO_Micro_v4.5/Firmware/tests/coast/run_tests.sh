#!/usr/bin/env bash
# 1. Cross-checks zCoastCore.c against the Python reference model on a synthetic sloped-curve drive,
#    with the Phase 2/3 models on and off (shadow mode).
# 2. Exercises the Phase 1 live coast (C only) against withheld ground truth: outage on the second curve,
#    receiver silence, timeout, and a forced coast.
# Needs: python3, gcc. Run from this folder.
set -e
SK=../../Autosteer_gps_teensy_AiO_Micro_v45
python coast_ref.py gen scenario.csv
gcc -std=c11 -O2 -Wall -Wextra -I"$SK" coast_host_test.c "$SK/zCoastCore.c" -o coast_host_test -lm

for flags in "" "--no-ext" "--no-ext --no-observer" "--no-ext --no-crab" "--no-ext --no-observer --no-crab"; do
  python coast_ref.py run scenario.csv $flags > py.out
  ./coast_host_test scenario.csv $flags > c.out
  echo "== shadow, flags: [${flags:-all models on}] =="
  echo "   fields: dur_ms,dist,along,cross,maxAlong,maxCross,delta,offset,beta,vStart,vEnd,fixes,Leff,k,wasSign,obsFrac,extFrac,extScale"
  sed 's/^/   /' c.out
  python - <<'EOF'
py = [l.split(',') for l in open('py.out') if l.startswith('REPORT')]
c  = [l.split(',') for l in open('c.out')  if l.startswith('REPORT')]
assert len(py) == len(c) and len(py) > 0, "report count mismatch"
for a, b in zip(py, c):
    for x, y in zip(a[1:], b[1:]):
        assert abs(float(x) - float(y)) <= 0.003 + 1e-6, f"mismatch {x} vs {y} in {a} / {b}"
print(f"   OK: {len(py)} shadow reports match between Python and C")
EOF
done

live_check() {   # $1 = label, $2.. = flags ; then python asserts read from stdin
  local label="$1"; shift
  ./coast_host_test scenario.csv "$@" > live.out || true
  echo "== live: $label  [$*] =="
  echo "   LIVEREPORT fields: reason,dur_ms,dist,alongAtEnd,crossAtEnd,forced,fallback,maxAlongTruth,maxCrossTruth,samples"
  grep -E "LIVE,|LIVEREPORT|Q0SENT" live.out | sed 's/^/   /'
}

live_check "15 s outage on the second curve (all models + pulse)" --live --outage 42 57
python - <<'EOF'
r = [l.strip().split(',') for l in open('live.out') if l.startswith('LIVEREPORT')]
assert len(r) == 1, f"expected one live report, got {len(r)}"
reason, dur, dist, along, cross, forced, fb, ma, mc, ns = r[0][1:]
assert reason == '1', "coast should end by recovery"
assert 14500 <= int(dur) <= 15500, f"duration {dur}"
assert float(ma) < 0.5 and float(mc) < 0.5, f"truth error too large: along {ma} cross {mc}"
print(f"   OK: recovered after {int(dur)/1000:.1f} s, max truth error along {ma} m cross {mc} m")
EOF

live_check "same outage, receiver silent (rows dropped)" --live --outage 42 57 --silence
python - <<'EOF'
r = [l.strip().split(',') for l in open('live.out') if l.startswith('LIVEREPORT')]
assert len(r) == 1 and r[0][1] == '1'
assert float(r[0][8]) < 0.5 and float(r[0][9]) < 0.5
print("   OK: silence detected, recovered, truth error within 0.5 m")
EOF

live_check "outage longer than the 20 s cap (no pulse, observer only)" --live --no-ext --outage 42 70
python - <<'EOF'
r = [l.strip().split(',') for l in open('live.out') if l.startswith('LIVEREPORT')]
q = [l for l in open('live.out') if l.startswith('Q0SENT')]
assert len(r) == 1 and r[0][1] == '2', f"expected timeout, got {r}"
assert 19900 <= int(r[0][2]) <= 20200, f"duration {r[0][2]}"
assert len(q) == 1, "one quality-0 sentence expected"
print(f"   OK: timed out at {int(r[0][2])/1000:.1f} s, quality-0 sentence sent once, max truth error along {r[0][8]} m cross {r[0][9]} m")
EOF

live_check "forced 10 s coast with GNSS good" --live --force 45 10
python - <<'EOF'
r = [l.strip().split(',') for l in open('live.out') if l.startswith('LIVEREPORT')]
assert len(r) == 1 and r[0][1] == '1' and r[0][6] == '1', f"expected forced recovery, got {r}"
assert abs(float(r[0][4])) < 0.5 and abs(float(r[0][5])) < 0.5
print(f"   OK: forced coast ended by recovery with true error along {r[0][4]} m cross {r[0][5]} m")
EOF

live_check "live disabled: outage must not start a coast" --outage 42 57
python - <<'EOF'
r = [l for l in open('live.out') if l.startswith('LIVEREPORT') or l.startswith('LIVE,')]
assert len(r) == 0, "no live output expected when live_enable is false"
print("   OK: nothing sent")
EOF
