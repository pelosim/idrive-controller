#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════
# Flash any of the car's ESP32 boards without touching the car.
#
# Compiles on this Mac, copies the app binary to the Pi, and flashes over
# the Pi's USB. The ESP32-S3's USB-Serial/JTAG is a ROM peripheral rather
# than something the sketch provides, so esptool can force download mode
# even if the firmware currently on the board crashes on boot — you
# cannot lock yourself out remotely the way you can with a board that
# uses a separate USB-UART bridge.
#
#   ./tools/flash-via-pi.sh idrive     iDrive controller
#   ./tools/flash-via-pi.sh lighting   interior lighting output board
#   ./tools/flash-via-pi.sh gauges1    aux gauge panel A / master
#   ./tools/flash-via-pi.sh gauges2    panel B / slave — no sketch yet
#   ./tools/flash-via-pi.sh            lists targets
#
# ── BOARD TABLE — edit here to add a board ─────────────────────────
# Each row is: repo-relative-to-~/Desktop/Claude | sketch dir | toolchain
# | FQBN | /dev symlink. The toolchains are deliberately separate: the
# lighting firmware uses ledcSetup/ledcAttachPin and the old ESP-NOW
# callback signature, both of which core 3.x removed, so it must build
# against 2.0.14. Flashing it with the v3 toolchain fails to compile.
#
# ONE-TIME PI SETUP (done): pip install esptool --break-system-packages,
# and /etc/udev/rules.d/99-espressif.rules for the stable symlinks —
# every board is pinned by its MAC, because adding the USB hub already
# reshuffled ttyACM numbering once.
#
# Only the app partition is written (0x10000). Bootloader and partition
# table do not change between builds. If you ever change the partition
# scheme, do one full flash over local USB first.
# ═══════════════════════════════════════════════════════════════════
set -euo pipefail

CLAUDE_DIR="${CLAUDE_DIR:-$HOME/Desktop/Claude}"
PI="${PI_HOST:-pi944}"
APP_OFFSET="0x10000"

board_config() {
  EXTRA_PROPS=()          # per-board extra --build-property args
  case "$1" in
    idrive)
      REPO="$CLAUDE_DIR/idrive-controller"; SKETCH="idrive_controller"
      TOOLCHAIN="$HOME/.arduino-cli-esp32v3"; PORT="/dev/idrive"
      FQBN="esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,FlashSize=16M,PSRAM=opi" ;;
    lighting)
      REPO="$CLAUDE_DIR/Automotive-Lighting-Controller"; SKETCH="firmware/pwm_controller"
      TOOLCHAIN="$HOME/.arduino-cli-esp32v2"; PORT="/dev/lighting"
      FQBN="esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,FlashSize=16M,PSRAM=opi,PartitionScheme=huge_app" ;;
    gauges1|gauges)
      # LilyGo T-Display-S3-Long panel A / master, board #1
      # (MAC 68:EE:8F:48:19:9C). Owns the ADS1115 and the ESP-NOW TX.
      # Settings taken from the project's own flash.sh / BUILD.md — note the
      # partition scheme differs from every other board here, and LVGL needs
      # the include-simple flag or the build fails on lvgl.h resolution.
      REPO="$CLAUDE_DIR/944_gauges"; SKETCH="build/gauge_bench"
      TOOLCHAIN="$HOME/.arduino-cli-esp32v2"; PORT="/dev/gauges"
      FQBN="esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,CDCOnBoot=cdc,USBMode=hwcdc,PartitionScheme=app3M_fat9M_16MB"
      EXTRA_PROPS=(--build-property "compiler.c.extra_flags=-DLV_LVGL_H_INCLUDE_SIMPLE"
                   --build-property "compiler.cpp.extra_flags=-DLV_LVGL_H_INCLUDE_SIMPLE") ;;
    gauges2)
      # Panel B / slave, board #2 (MAC 68:EE:8F:48:18:D4) — AFR + battery,
      # no sensors of its own, everything arrives over ESP-NOW.
      #
      # DELIBERATELY NOT FLASHABLE YET. slave_app.h is an app layer meant to
      # be dropped into a copy of LilyGo's lvgl_demo; that sketch has not
      # been assembled. Pointing this at gauge_bench would flash the master's
      # ADS1115 readout onto a board with no ADS1115 attached.
      echo "gauges2 has no sketch yet — slave_app.h still needs assembling" >&2
      echo "  into a gauges_slave sketch (see 944_gauges/CLAUDE.md)." >&2
      return 1 ;;
    *) return 1 ;;
  esac
}

KNOWN_TARGETS="idrive lighting gauges gauges1 gauges2"
TARGET="${1:-}"
if [ -z "$TARGET" ] || ! board_config "$TARGET"; then
  # A known-but-unbuildable target has already explained itself; only an
  # unrecognised name needs the usage line.
  case " $KNOWN_TARGETS " in
    *" $TARGET "*) exit 1 ;;
  esac
  echo "usage: $(basename "$0") <idrive|lighting|gauges1|gauges2>" >&2
  [ -n "$TARGET" ] && echo "  unknown target: $TARGET" >&2
  exit 1
fi

BIN_NAME="$(basename "$SKETCH")"
BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT

export ARDUINO_DIRECTORIES_DATA="$TOOLCHAIN/data"
export ARDUINO_DIRECTORIES_USER="$TOOLCHAIN/user"
export ARDUINO_DIRECTORIES_DOWNLOADS="$TOOLCHAIN/downloads"

[ -d "$REPO/$SKETCH" ] || { echo "no such sketch: $REPO/$SKETCH" >&2; exit 1; }

echo "── $TARGET ────────────────────────────────────────────────"
echo "   sketch    $SKETCH"
echo "   toolchain $(basename "$TOOLCHAIN")"
echo "   port      $PORT"
echo
echo "── compiling ─────────────────────────────────────────────"
arduino-cli compile --warnings all --fqbn "$FQBN" \
  "${EXTRA_PROPS[@]}" \
  --output-dir "$BUILD" "$REPO/$SKETCH" 2>&1 \
  | grep -E "Sketch uses|Global variables|error|warning:" \
  | grep -v "libraries/" || true

BIN="$BUILD/$BIN_NAME.ino.bin"
[ -f "$BIN" ] || { echo "compile produced no binary — aborting" >&2; exit 1; }
echo "   binary: $(du -h "$BIN" | cut -f1)"

echo
echo "── checking the board is visible on $PI ──────────────────"
if ! ssh -4 -o ConnectTimeout=10 "$PI" "test -e $PORT"; then
  cat >&2 <<EOF

   $PORT not present on $PI.

   Either the board is unplugged, or its udev rule is missing. Check with:
     ssh $PI 'ls -l /dev/ttyACM* /dev/idrive /dev/lighting /dev/gauges'

   Every board is pinned by MAC in /etc/udev/rules.d/99-espressif.rules —
   a new board needs a row there before it gets a stable name.
EOF
  exit 1
fi

# The HVAC backend holds /dev/lighting (and would hold any other board it
# talks to) the moment udev creates the symlink. esptool and a reader on the
# same character device fight, and the flash dies partway with "No more data
# to read from the serial port" — leaving a half-written app partition.
# Release the port for the duration, then put it back.
HELD_BY_BACKEND=0
if ssh -4 "$PI" "sudo fuser $PORT 2>/dev/null | grep -q ." ; then
  echo
  echo "── $PORT is open by the backend — stopping it for the flash ──"
  ssh -4 "$PI" "sudo systemctl stop hvac-backend" && HELD_BY_BACKEND=1
  sleep 1
fi
restore_backend() {
  if [ "$HELD_BY_BACKEND" = "1" ]; then
    echo "── restarting hvac-backend ───────────────────────────────"
    ssh -4 "$PI" "sudo systemctl start hvac-backend" || \
      echo "   !! backend did NOT restart — start it by hand" >&2
  fi
}
trap 'restore_backend; rm -rf "$BUILD"' EXIT

echo
echo "── uploading + flashing ──────────────────────────────────"
scp -q -4 "$BIN" "$PI:/tmp/$BIN_NAME.bin"
ssh -4 "$PI" "python3 -m esptool --chip esp32s3 --port $PORT --baud 921600 \
  write-flash $APP_OFFSET /tmp/$BIN_NAME.bin && rm -f /tmp/$BIN_NAME.bin" \
  2>&1 | grep -viE "^esptool|^$" | tail -12

echo
echo "✓ $TARGET flashed on $PI — the board has rebooted."
