#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════
# Flash the iDrive controller without touching the car.
#
# Compiles on this Mac, copies the app binary to the Pi, and flashes the
# ESP32 over the Pi's USB. The ESP32-S3's USB-Serial/JTAG is a ROM
# peripheral rather than something the sketch provides, so esptool can
# force download mode even if the firmware currently on the board crashes
# on boot — you cannot lock yourself out remotely the way you can with a
# board that uses a separate USB-UART bridge.
#
#   ./tools/flash-via-pi.sh              flash idrive_controller
#   ./tools/flash-via-pi.sh ir_capture   flash a different sketch
#
# ONE-TIME SETUP ON THE PI (already done):
#   pip install esptool --break-system-packages
#   /etc/udev/rules.d/99-espressif.rules  — keeps ModemManager away from
#   the board and provides the stable /dev/idrive symlink.
#
# Only the app partition is written (0x10000). The bootloader and
# partition table do not change between builds. If you ever change the
# partition scheme, do one full flash over local USB first.
# ═══════════════════════════════════════════════════════════════════
set -euo pipefail

SKETCH="${1:-idrive_controller}"
PI="${PI_HOST:-pi944}"
PORT="${ESP_PORT:-/dev/idrive}"
FQBN="esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,FlashSize=16M,PSRAM=opi"
APP_OFFSET="0x10000"

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT

export ARDUINO_DIRECTORIES_DATA="$HOME/.arduino-cli-esp32v3/data"
export ARDUINO_DIRECTORIES_USER="$HOME/.arduino-cli-esp32v3/user"
export ARDUINO_DIRECTORIES_DOWNLOADS="$HOME/.arduino-cli-esp32v3/downloads"

[ -d "$REPO/$SKETCH" ] || { echo "no such sketch: $REPO/$SKETCH" >&2; exit 1; }

echo "── compiling $SKETCH ─────────────────────────────────────"
arduino-cli compile --warnings all --fqbn "$FQBN" \
  --output-dir "$BUILD" "$REPO/$SKETCH" 2>&1 \
  | grep -E "Sketch uses|Global variables|error|warning:" \
  | grep -v "libraries/" || true

BIN="$BUILD/$SKETCH.ino.bin"
[ -f "$BIN" ] || { echo "compile produced no binary — aborting" >&2; exit 1; }
echo "   binary: $(du -h "$BIN" | cut -f1)"

echo
echo "── checking the board is visible on $PI ──────────────────"
if ! ssh -4 -o ConnectTimeout=10 "$PI" "test -e $PORT"; then
  cat >&2 <<EOF

   $PORT not present on $PI.

   The ESP32 is not plugged into the Pi's USB, or it has not enumerated.
   Check with:  ssh $PI 'ls -l /dev/ttyACM* /dev/idrive'

   Note this is the USB cable, not the 3-wire UART link — the UART
   carries data but cannot flash.
EOF
  exit 1
fi

echo
echo "── uploading + flashing ──────────────────────────────────"
scp -q -4 "$BIN" "$PI:/tmp/$SKETCH.bin"
ssh -4 "$PI" "python3 -m esptool --chip esp32s3 --port $PORT --baud 921600 \
  write-flash $APP_OFFSET /tmp/$SKETCH.bin && rm -f /tmp/$SKETCH.bin" \
  2>&1 | grep -viE "^esptool|^$" | tail -14

echo
echo "✓ $SKETCH flashed to the board on $PI"
echo "  The ESP32 has rebooted. If the backend was mid-read on the UART it"
echo "  will simply see the link go quiet and resume — no restart needed."
