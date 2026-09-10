# Dead-Reckoning Coast — Design

Firmware feature for the AiO Micro v4.5 port (UM982 + TM171 + Keya). 2026-09-10.

**Status: all phases implemented.** Phases 0, 2, 3 (shadow mode, curve/slope models, ground-speed pulse) run whenever `COAST_SHADOW_MODE` is on and never change what AgIO receives. Phase 1, the live coast, is compiled in but **off by default** (`COAST_LIVE_ENABLE false`) and must only be enabled after the shadow numbers from real drives look right and AgOpenGPS has been checked against the chosen sentence.

**Phase 1 as built.** Entry: a KSXT with position quality 0/1 (2 = float too if `COAST_ON_FLOAT`), or no KSXT for 250 ms, while the last RTK-fixed fix is at most 1 s old, the TM171 packet at most 100 ms old, and the heading offset is learned. The integrator starts from that last fix and first integrates the gap since it. While live the receiver's own KSXT/GGA-derived sentences are held back and a coasted sentence goes out every 100 ms: `$PANDA` with fix quality 6, growing HDOP and the coast age in the DGPS-age field (default), or a synthetic `$KSXT` with `COAST_KSXT_QUALITY` (Plan B, `COAST_OUTPUT_KSXT`). Exit on the second consecutive RTK-fixed fix (AgOpenGPS steps to the true position; `$COASTREPORT` prints the true error), or on `COAST_MAX_S` / `COAST_MAX_M` (one quality-0 sentence is sent so AgOpenGPS disengages), or on sensor loss. If the TM171 stops for 200 ms the heading is carried by the wheel model `v·tan δ / L_eff`; if the WAS is invalid too the coast ends. Float fixes neither start nor end a coast. `!AOGCO,10` on USB forces a 10 s coast while GNSS is good, so the report holds the true error. Host tests (`tests/coast/run_tests.sh`) replay a 15 s outage on the sloped curve with the position withheld as truth: max error 5 cm along / 14 cm cross with the pulse, recovery on the second fix; receiver silence, the 20 s cap with the quality-0 sentence, and the forced coast are covered too.

**Prebuilt hexes** (`build_coast_hexes.sh`, geometry defaults from the "6480" profile, overridable at runtime per §10a): `…_coast-shadow.hex` (measure only), `…_coast-live.hex` (coast on GNSS loss, PANDA quality 6, no speed pulse), `…_coast-live-pulse37.hex` (same, ground-speed pulse on pin 37).

**Sharing the tractor speed signal.** The ISO 11786 speed pin is an output; a seed drill controller and the AiO board can both listen to it. Tap it as a parallel branch through the resistor divider (about 1 mA load) with a common ground, and the drill sees no difference.

**Phase 3 as built.** `COAST_SPEED_PULSE_PIN` (default −1 = off) counts pulses in an interrupt (100 µs glitch filter) and converts them to a raw speed every 50 ms from the pulse timestamps, using `COAST_PULSES_PER_M` (130 for ISO 11786). The core learns a scale factor against the GNSS axle speed (30 s low-pass, valid after 50 samples), projects the speed to the horizontal with `cos(pitch)`, and uses it whenever it is under 300 ms old; otherwise it falls back to the observer/hold. When the pin is 37 (`REMOTE_PIN`) the remote-switch read reports "open" and the encoder kickout is disabled automatically. **Hardware:** Teensy 4.1 pins are 3.3 V only; an ISO 11786 signal swings to battery voltage, so put a resistor divider (e.g. 10 kΩ / 3.3 kΩ) or an optocoupler in front of the pin, and check the tractor's connector pinout (pin 1 radar ground speed, pin 2 wheel speed). On the synthetic drive the pulse removes the straight-line speed hold entirely: the post-curve window goes from 7.9 m to 0.09 m along-track. Code: `zCoastCore.h/.c` (pure C core, also built by `tests/coast/run_tests.sh` against the Python reference model), `zCoast.ino` (glue), settings in the main sketch's user-settings block.

**Findings from the synthetic drive (see §11 for how to read real numbers):**

- The KSXT track angle and speed belong to the *antenna*, not the axle. On a rolled turn the antenna sits `h·sin φ` inside the turn and, whenever the roll changes, swings sideways at `h·cos φ·φ̇`. Both terms are now removed before anything is called crab or used to learn the wheelbase; without that the learned crab gain was off by 60 % after every roll ramp.
- The measured slip crab is low-passed over 1 s before it is used at window entry or for learning `k`; a single 10 Hz KSXT sample is too noisy.
- With a valid `k`, the entry residual decays toward the `k·sin φ` model with τ = 5 s, so a noisy entry value does not get carried onto flat ground.
- On a straight there is nothing to observe speed from, so an acceleration after a curve shows up as several metres of along-track error. It converts to cross-track only when the next curve begins (`s²/2R`). This is the Phase 3 case: only a speed pulse fixes it.

## 1. Problem

Under one section of trees the UM982 loses the fix completely. That section is a curve — you drive around the trees — on side slopes up to about 15°. Helical antennas did not help. When the fix goes, AgOpenGPS drops autosteer.

**Goal.** Keep AgOpenGPS supplied with a position good enough to hold the line through the gap *and* through RTK reconvergence afterwards, for a bounded time, while making it unmistakable to AgOpenGPS and the driver that the position is dead-reckoned.

**Non-goals.** This is not an INS. No accelerometer integration, no Kalman filter, no attempt to survive beyond the configured maximum. It is a kinematic coast on the inputs the Teensy already has.

## 2. Inputs already on the Teensy

| Signal | Source | Rate | Note |
|---|---|---|---|
| Position, fix quality, sats, HDOP | GGA on Serial7 | 10 Hz | parsed today |
| Dual-antenna heading ψ, track angle χ, ground speed, roll, E/N velocity, quality | KSXT on Serial7 | 10 Hz | **forwarded verbatim today, not parsed** — parsing must be added |
| Yaw ψ_imu, roll φ, pitch θ | TM171 code 35 on Serial2 | as configured | free-integrating yaw |
| Steer angle δ | `steerAngleActual` (WAS via ADS1115) | steer loop | already global |
| Ground-speed pulse | — | — | **not available.** The board outputs a speed pulse on pin 36; it does not read one. Optional Phase 3. |

## 3. Vehicle model

A kinematic bicycle model in the **rear-axle frame**. The one physical assumption that makes this work is the non-holonomic constraint: the rear axle has no lateral velocity except the slope crab.

Geometry (all also exist as AgOpenGPS settings and must match):

- `L` wheelbase
- `a` antenna forward of the rear axle (AOG "antenna pivot")
- `h` antenna height above ground (AOG "antenna height")
- `φ` roll, positive right-side-down (AOG convention; check `invertRoll`)
- `ψ` heading, `β` crab angle, `v` axle ground speed

Rear-axle propagation:

```
dN = v · cos(ψ + β) · dt
dE = v · sin(ψ + β) · dt
```

Antenna position for output:

```
antenna = axle + a · fwd(ψ) + h · sin(φ) · right(ψ)
fwd(ψ)   = [sin ψ, cos ψ]      (E, N)
right(ψ) = [cos ψ, −sin ψ]
```

**Why the axle frame and not the antenna.** On a curve the antenna has a lateral velocity `a·ψ̇` that the GNSS track angle includes. Coasting the antenna with a frozen track angle is wrong the moment the steer angle changes. And on a slope the roll changes through the curve: with `h = 3 m` a 5° roll change moves the antenna 26 cm sideways. Coasting the axle and re-adding the *current* roll offset means AgOpenGPS's own roll correction cancels it exactly.

## 4. Continuous calibration while GNSS is good

Every good KSXT (RTK fixed, `v > 1 m/s`) updates four things. All plain floats, no heap.

| Quantity | Formula | Filter | Purpose |
|---|---|---|---|
| Heading offset `Δ` | `wrap(ψ_gnss − ψ_imu)` | LPF τ ≈ 2 s | pins the free-running TM171 yaw to true north; frozen at loss |
| Slip crab `β_slip` | `wrap(χ − ψ_gnss) − atan2(a·ψ̇, v)` | — | total crab minus the geometric part from turning |
| Crab gain `k` | `β_slip / sin φ` when `|φ| > 3°` | LPF τ ≈ 10 s, clamp [0, 1] | on a side slope the tractor crabs downhill; `β = k·sin φ` follows the roll through the curve instead of holding a stale crab |
| Effective wheelbase `L_eff` | `v · tan δ / ψ̇` when `|δ| > 5°` and `|ψ̇| > 2°/s` | LPF τ ≈ 10 s, clamp [0.5L, 2L] | absorbs WAS scale error, Ackermann and understeer; used by the speed observer and the IMU-fallback heading |

Plus the last good `lat, lon, alt, v, sats, φ, θ` and their timestamps.

## 5. Loss detection and entry

Enter the coast when either:

- no GGA/KSXT for more than 250 ms, or
- GGA fix quality is in the "lost" set — default `{0, 1, 2}`; float (5) is included only if `COAST_ON_FLOAT`.

The UM982 keeps emitting GGA and KSXT with quality 0 and possibly empty position fields during an outage. Those must be **intercepted before forwarding**, otherwise AgOpenGPS sees them and drops out regardless.

Entry is refused (behaviour stays exactly as today) unless: the last good fix was RTK fixed no more than 1.0 s ago, the last TM171 packet is under 100 ms old, and `Δ` is valid. `k` and `L_eff` fall back to defaults (`k = 0`, `L_eff = L`) if not yet learned.

## 6. Coast propagation

Runs on every TM171 packet, `dt` from `micros()`, position kept in metres from the entry point (double).

1. **Heading.** `ψ = ψ_imu + Δ`. Yaw rate `ψ̇` by differencing yaw, LPF 50 ms. If the TM171 goes stale (> 200 ms): `ψ̇ = v · tan δ / L_eff`, integrate. If the WAS is invalid too: terminate.
2. **Speed.** Hold `v_last`. *Speed observer:* when `|δ| > 5°` and `|ψ̇| > 2°/s`, `v_obs = ψ̇ · L_eff / tan δ`; blend `v += α (v_obs − v)`, `α = 0.05` per sample, rate-limited to 1.5 m/s², clamped to `[0, 1.25·v_last]`. On a curve the driver usually slows; the gyro sees it through the curvature. On a straight there is nothing to observe and the hold stands. If a Phase-3 speed pulse exists: `v = v_pulse · cos θ` (slope-projected) and the observer is off.
3. **Crab.** `β = k · sin φ` using the *current* roll; if `k` was never learned, hold `β_slip` from entry.
4. **Integrate the axle** (section 3).
5. **Metres → degrees** with the meridional and prime-vertical radii `R_M`, `R_N` at the current latitude. A single mean radius carries up to 0.3 % scale error, which is 15 cm over a 50 m coast.
6. **Antenna** from the axle (section 3).
7. **Altitude.** `alt += v · sin θ · dt`. Small, harmless, keeps the field plausible.

## 7. Output to AgOpenGPS

Every 100 ms while coasting: suppress raw KSXT forwarding and the GGA-triggered `BuildNmea()`, and emit a `$PANDA` built from the coast state.

| Field | Value |
|---|---|
| 1 time | last fix time + coast elapsed |
| 2–5 lat/lon | `ddmm.mmmmmmm` from the coasted antenna position |
| 6 fix quality | **6 = estimated (dead reckoning)** — the NMEA code already listed in the PANDA table in `zHandlers.ino` |
| 7 sats | last good |
| 8 HDOP | `0.5 + 0.05 · t_coast` — grows so AOG's display shows the degradation |
| 9 altitude | coasted |
| 10 age | coast seconds |
| 11 speed | knots |
| 12–15 heading, roll, pitch, yaw rate | same encoding as today's TM171 path |

**Why PANDA and not a synthetic KSXT.** KSXT has no dead-reckoning quality code; PANDA quality 6 is standard and visible. **Open point to test before Phase 1:** how AgOpenGPS behaves when the sentence type switches KSXT → PANDA mid-session (heading source), and which fix-quality values its RTK-loss alarm / steer lockout accept. If the switch misbehaves, Plan B is a synthetic KSXT with coasted fields and a quality code chosen after reading AOG's `ParseKSXT`.

## 8. Exit

- **GNSS returns.** Two consecutive GGA with quality ≥ 4 (RTK fixed): step to the true position, no blending. A step is honest, and the steering loop handles a small one. Print a **coast report** to USB: duration, distance, final error split into along-track and cross-track relative to heading. That report is the field-performance measurement.
- **Timeout.** `t > COAST_MAX_S` (default 20) or distance `> COAST_MAX_M` (default 60): emit a quality-0 PANDA so AgOpenGPS disengages exactly as it does today.
- **Sensor loss.** TM171 and WAS both invalid: immediate quality 0.
- Never extend a coast, never coast from a coast.

## 9. Error budget

Scenario: curve radius 25 m, 2 m/s, 20 s coast (40 m), 15° side slope, `h = 3 m`.

| Source | Assumption | Cross-track effect |
|---|---|---|
| Heading offset `Δ` at entry | 0.2° (dual-antenna + filter) | 14 cm |
| TM171 yaw drift | 0.1° over 20 s, enters gradually | ~7 cm |
| Along-track error `s` from speed | converts as `s² / 2R`: `s = 2 m` (10 %) | 8 cm; 4 m → 32 cm; on a straight ≈ 0 |
| Crab change, uncorrected | 1° | **70 cm** |
| Crab change, with `k·sin φ` | residual 0.3° | 21 cm |
| Roll change 5°, antenna-frame coast | `h · Δsin φ` | 26 cm |
| Roll change 5°, axle-frame coast | `h` known within 10 cm | ~1 cm |
| Antenna pivot `a` error | 0.2 m over a 20° heading change | 7 cm |
| Earth radius | mean radius vs `R_M/R_N` | 15 cm → < 1 cm |

Realistic total: **20–40 cm at 20 s on a sloped curve, 5–15 cm at 10 s, under 10 cm at 20 s on flat straight ground.** The crab model and the speed observer are the two items the log replay must validate; everything else is arithmetic.

## 10. Settings

Compile-time constants in the user-settings block of the main sketch, matching the style of the rest of the firmware.

| Constant | Default | Meaning |
|---|---|---|
| `COAST_ENABLE` | `false` | master switch for coast output |
| `COAST_SHADOW_MODE` | `true` | run the estimator against live GNSS and report error (section 11) |
| `COAST_MAX_S` | 20 | hard time cap |
| `COAST_MAX_M` | 60 | hard distance cap |
| `COAST_ON_FLOAT` | `false` | treat RTK float as lost |
| `COAST_WHEELBASE_M` | — | nominal `L`; refined online |
| `COAST_ANTENNA_FWD_M` | — | `a`, must equal AOG antenna pivot |
| `COAST_ANTENNA_HEIGHT_M` | — | `h`, must equal AOG antenna height |
| `COAST_SPEED_PULSE_PIN` | −1 | Phase 3 input, −1 = none |
| `COAST_PULSES_PER_M` | 130 | ISO 11786 ground-speed signal |

### 10a. Vehicle geometry at runtime

The geometry no longer has to be compiled in. Precedence: compile-time defaults (from the "6480" profile) < EEPROM (last value received) < a live update. Two ways to update:

- **AgOpenGPS PGN 209 (0xD1)** over the usual UDP port 8888, to be sent whenever the vehicle settings are saved and once after connecting. Sent by `FormGPS.SendVehicleGeometry()` in the AgOpenGPS-farm fork (with the other module settings, and on every geometry edit). 0xD0 was avoided because AgOpenGPS already reserves it for a latitude/longitude message.

  | byte | value |
  |---|---|
  | 0–2 | `0x80 0x81 0x7F` |
  | 3 | `0xD1` (209) |
  | 4 | length = 8 |
  | 5–6 | wheelbase, cm, uint16 LE |
  | 7–8 | antenna pivot, cm, int16 LE (positive = antenna ahead of the rear axle) |
  | 9–10 | antenna height, cm, uint16 LE |
  | 11–12 | antenna offset, cm, int16 LE, AgOpenGPS sign (+ = antenna left of centre) |
  | 13 | CRC = sum of bytes 2–12, low byte (same as the other PGNs) |

  Fill from `setVehicle_wheelbase`, `setVehicle_antennaPivot`, `setVehicle_antennaHeight`, `setVehicle_antennaOffset`.

- **USB command** `!AOGCG,2.8,0.1,3.0,-0.35` (metres, AgOpenGPS signs) from any serial monitor.

Both print `$COASTMSG,geometry applied …` and store the value in EEPROM (only when it changed). A change that arrives during a live coast is applied when that coast ends. Changing the wheelbase restarts the `L_eff` learning; any change restarts the crab, slip and pulse-scale learning, since the antenna geometry enters all of them.

### 10b. Diagnostics and commands without a USB cable

The AiO's enclosure does not expose USB, so everything also runs over the Ethernet link AgIO already uses:

| Channel | Carries | Where it shows |
|---|---|---|
| AgOpenGPS hardware message (PGN 221) | coast start, `$COASTREPORT` summary, each shadow window (`COAST_DISPLAY_SHADOW`), geometry changes | on the AgOpenGPS screen (tick **Hardware Messages** in the display settings) and in `Documents/AgOpenGPS/Logs/AgOpenGPS_Events_Log.txt` |
| UDP broadcast on `COAST_UDP_LOG_PORT` (5125) | every diagnostic line, including the 10 Hz `$COAST` log when `COAST_LOG` is on | `tests/coast/coast_monitor.py` on any laptop on the tractor subnet |
| AgOpenGPS PGN 210 (0xD2) | commands to the module; the **Coast 10 s test** button in the steer settings form sends cmd 1 | — |
| UDP text to the module's port 8888 | `!AOGCO,<s>` and `!AOGCG,L,pivot,height,offset` as plain text | `coast_monitor.py --module <ip> --force 10` |
| USB serial | all of the above, when a cable is available | serial monitor |

Alarm-coloured messages (salmon) are used for a real GNSS loss, a timeout, a sensor loss or a rejected geometry; a forced test and normal reports are bisque.

## 11. Shadow mode and validation

**Shadow mode** is the key tool. The coast estimator runs continuously while GNSS is good, restarting every 20 s from the current fix and integrating in the background. Every 100 ms it compares itself with the live fix; at the end of each window it prints max along-track and cross-track error. This measures coast accuracy on every drive, on every field, with no GPS loss and no risk. Turn it on first, drive the sloped curves in the open, read the numbers. Only then enable output.

**Bench.** A Python reference model with the same equations, fed by a USB log line added to the firmware (`t, lat, lon, q, ψ_gnss, χ, v, ψ_imu, φ, θ, δ`). Mask 5/10/20/30 s windows in logged drives, compute errors, tune `τ`, `α` and the thresholds. Log the real tree section too: outage duration and time-to-refix decide `COAST_MAX_S`.

**AgOpenGPS.** Before enabling `COAST_LIVE_ENABLE`, run one forced coast on open ground with autosteer engaged on a straight AB line: `!AOGCO,10` on the USB monitor. Watch three things: that the fix-quality display shows 6 (or float, for the KSXT variant), whether autosteer stays engaged, and whether the tractor holds the line. If AgOpenGPS drops steering on quality 6, switch to `COAST_OUTPUT_KSXT true`. If the heading jumps at the KSXT → PANDA switch, try `COAST_PANDA_HEADING_TRUE false`.

**Field acceptance.** Phase 1 in an open field with a forced coast (`!AOGCOAST,10` forces a 10 s coast while GNSS is still received, so the report contains the true error). Then the tree section with `COAST_MAX_S = 5`, extended only as the reports justify.

## 12. Phasing

0. KSXT parsing, USB logging, shadow mode. No behaviour change.
1. Coast output: heading offset, speed hold, axle-frame geometry, PANDA quality 6, timeouts, exit report.
2. Curve and slope: speed observer, `L_eff` self-calibration, crab-gain model.
3. Optional ground-speed pulse input. Most tractors expose radar ground speed on the ISO 11786 signal connector (pin 1, 130 pulses/m). `REMOTE_PIN` 37 (H_TEENSY header row 20) is a candidate input.

## 13. Code map

| File | Change |
|---|---|
| `zCoast.ino` (new) | state struct; `coastRecordGood()`, `coastUpdate()`, `coastEmit()`, `coastShadow()`, `coastReport()` |
| `zHandlers.ino` | `KSXT_Handler` parses lon, lat, height, heading, pitch, track, speed, roll, position quality, heading quality, E/N velocity before forwarding (**indices and quality codes to be verified** against the UM982 reference and AOG's `ParseKSXT`); forwarding and `BuildNmea()` gated by coast state; `GGA_Handler` feeds quality and timing |
| `TM171.ino` | expose yaw timestamp (`micros`) and yaw rate |
| main sketch | settings block; `coastUpdate()` in `loop()`; `!AOGCOAST` in the serial command parser |

About 350 lines, float math only, no heap. Negligible load on a 600 MHz Teensy 4.1.

## 14. Open questions

- Which AgOpenGPS version is in use, whether it keeps steering on quality 6, and how it handles the KSXT → PANDA switch.
- KSXT field indices and quality codes on the UM982 firmware in use.
- TM171 yaw sign, wrap and output rate (affects the `Δ` filter and `ψ̇`).
- Whether the crab model needs hysteresis at the transition into and out of the slope.
- A WAS zero error is absorbed in the primary path (the gyro carries heading) and only biases the IMU-fallback path; acceptable.
