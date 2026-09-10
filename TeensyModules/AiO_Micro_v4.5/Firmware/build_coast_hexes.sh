#!/usr/bin/env bash
# Builds the AiO Micro v4.5 coast firmware variants from one source. Run from this folder.
#   shadow       : COAST_LIVE_ENABLE false, no speed pulse   (measure only; output to AgIO unchanged)
#   live         : COAST_LIVE_ENABLE true,  no speed pulse   (coast on GNSS loss, PANDA fix quality 6)
#   live-pulse37 : COAST_LIVE_ENABLE true,  speed pulse on pin 37
# The geometry defaults in the sketch are overridden at runtime by EEPROM / AgOpenGPS PGN 208 / !AOGCG.
set -e
FQBN="teensy:avr:teensy41:usb=serial,speed=600,opt=o2std"
SK=Autosteer_gps_teensy_AiO_Micro_v45
BASE=AOG-Keya-CANBUS_lazykickoff95_TM171_KSXT_AiO-Micro-v45_coast

# Teensy's recipe has no extra_flags hook, so the defines ride on build.flags.cpp (default value kept).
CPPFLAGS="-std=gnu++17 -fno-exceptions -fpermissive -fno-rtti -fno-threadsafe-statics -felide-constructors -Wno-error=narrowing"

build() {   # $1 = suffix, $2 = extra -D flags
  echo "== $1  [$2]"
  rm -rf "build_$1"
  arduino-cli compile --fqbn "$FQBN" --output-dir "build_$1" \
    --build-property "build.flags.cpp=$CPPFLAGS $2" "$SK" 2>&1 | grep -E "error|FLASH:" || true
  cp "build_$1/$SK.ino.hex" "${BASE}-$1.hex"
  rm -rf "build_$1"
}

build shadow       "-DCOAST_LIVE_ENABLE=false -DCOAST_SPEED_PULSE_PIN=-1"
build live         "-DCOAST_LIVE_ENABLE=true  -DCOAST_SPEED_PULSE_PIN=-1"
build live-pulse37 "-DCOAST_LIVE_ENABLE=true  -DCOAST_SPEED_PULSE_PIN=37"
ls -la ${BASE}-*.hex
