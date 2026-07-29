# CLAUDE.md — idrive-controller

## Why
Retrofit a BMW F-Series iDrive rotary controller into the 1987 Porsche 944S as the
cabin input device, and use it to drive a **Power Acoustik CP-71W** head unit
(volume, mute, track change). Same car as the 944 HVAC / lighting / gauge projects.

The 944S is pre-CAN, so the iDrive and the ESP32 form a **private two-node CAN bus**
that touches nothing else in the car. There is no gateway and no safety-critical
traffic on it — unlike a live vehicle bus, this one is safe to experiment on freely.
Because it is a dedicated 2-node bench-style bus, the 120Ω termination **is** required
here (the opposite of the "never terminate when tapping a live car" rule in the
canclaude project).

## What
Single Arduino sketch: `idrive_controller/idrive_controller.ino` (ESP32-S3, TWAI).
- Decodes all 14 iDrive inputs from CAN ID `0x25B` @ 500 kbps
- Sends the wake + keep-alive frame set the controller needs to stay awake
- Drives the head unit through a **switched resistor ladder** on 5 GPIOs

Web tools in `tools/` (Chrome, Web Serial API) for reverse-engineering:
`idrive-mapper.html` (map buttons, export code) and `can-analyzer.html` (frame diff).

## Hardware
| Part | Detail |
|---|---|
| MCU | Lonely Binary ESP32-S3 Dev Module (16MB flash, OPI PSRAM) |
| Transceiver | **SN65HVD230** — 3.3V native. NOT MCP2551 (5V, needs level shifting) |
| iDrive | BMW F-Series Preh, part #6582 6829079-03 |
| Head unit | Power Acoustik CP-71W — resistive wired-remote input |

Pins: CTX=GPIO4, CRX=GPIO8, NeoPixel=GPIO48.
SWC ladder: GPIO5/6/7/15/16 → 560Ω/1k/1.5k/2.2k/3.9k → head unit remote line.

Pins to avoid on this board: 19/20 (USB), 26–37 (flash + OPI PSRAM),
0/45/46 (strapping), 43/44 (UART0).

## Key design decision: resistance, not voltage
The head unit measures **resistance to ground** against its own internal pull-up.
Version 1.0.0 planned a PWM + RC filter to synthesize a voltage — that cannot work:
a 10kΩ source is the same order as the radio's pull-up (unpredictable divider that
drifts with its reference), and 10k × 10µF is a 100 ms time constant, far too slow
to step volume. Do not reintroduce that approach.

**Before wiring:** measure the head unit's remote line to ground. If it is above
3.3V, insert a 2N7000 per leg (gate←GPIO, drain→resistor→line, source→GND) so the
ESP32 never contacts the line.

## Build / verify
Isolated toolchain (esp32 core 3.3.10, Adafruit NeoPixel installed):

    export ARDUINO_DIRECTORIES_DATA=~/.arduino-cli-esp32v3/data \
           ARDUINO_DIRECTORIES_USER=~/.arduino-cli-esp32v3/user \
           ARDUINO_DIRECTORIES_DOWNLOADS=~/.arduino-cli-esp32v3/downloads
    arduino-cli compile --warnings all \
      --fqbn esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,FlashSize=16M,PSRAM=opi \
      idrive_controller

Baseline 1.1.0: 324182 bytes flash (24%), 22740 bytes RAM (6%), zero warnings.

Arduino IDE equivalent: ESP32S3 Dev Module, USB CDC On Boot **Enabled**,
USB Mode "Hardware CDC and JTAG", Flash 16MB, PSRAM OPI.
Upload hangs at "Connecting..." → hold BOOT, tap RESET, release BOOT.

## Gotcha: USB CDC blocks when nothing is listening
`Serial.setTxTimeoutMs(0)` in `setup()` is load-bearing — do not drop it.
Hardware USB CDC blocks on write when no host drains the FIFO: the core uses
`tx_timeout_ms=100` with up to 20 consecutive timeouts, so a single `printf`
can stall ~2 s once the buffer backs up. Since this sketch prints on every
input event, running **without** a serial monitor — i.e. in the car — stalls
`loop()` on every press and makes the NeoPixel lag and drop events.

The trap is that opening a terminal makes it disappear, so it reads as flaky
hardware or a bad encoder. Found 2026-07-28 exactly that way: the LED was
erratic, then became perfect the moment a `cat` on the port started.

## Conventions
- **Full deployable `.ino` files, never diffs or snippets** — Mark flashes whole files.
- Nothing blocking in `loop()`. The 20 ms keep-alive cadence is load-bearing: if it
  slips, the iDrive sleeps. LED flash and SWC presses are both deadline-driven state
  machines for this reason; do not reintroduce `delay()` into the event path.
- Exactly one ladder leg asserted at a time; release is high-Z (`pinMode(pin, INPUT)`),
  never `OUTPUT`+`HIGH`.
- Never send `0x440` with byte1=`0xFF` — that is a sleep request.
- Keep the "KNOWN ISSUES" and init-sequence tables in the README intact on regen.

## Output path: IR is primary, SWC ladder is the fallback
**The CP-71W has no SWC learn/study mode** (confirmed on the unit). Its resistive
ladder is therefore a fixed, undocumented, per-radio map — the thing PAC sells
programming for — so using it means first discovering the values by sweeping ~0–5 kΩ.

The bundled IR remote sidesteps that entirely: volume, mute, and track buttons are
physically on it, so the codes provably exist and capture is deterministic. Hence
`ir_capture/ir_capture.ino` (IRremoteESP8266 2.9.0, verified to build on S3/core 3.3.10).

The ladder stays behind `SWC_ENABLE` in case a sweep later finds a usable map.

⚠️ When wiring IR *send* into the main sketch: a full NEC frame is ~67 ms of blocking
carrier work. Run `IRsend` in a FreeRTOS task pinned to **core 0** (Arduino `loop()`
runs on core 1) fed by a queue. Putting it inline in `loop()` would reintroduce the
exact frame-dropping bug fixed in 1.1.0.

## Status: v1.3.0 is the in-car build
Tagged `v1.3.0`. Everything on the ESP32 side is verified on hardware; the only
untested link is whether the radio physically responds to the IR.

Verified: 339 rotation events over 95 s with no spurious drops · 157 CAN events
expanding to exactly 190 IR sends (per-detent volume exact) · ir_loopback 15/15 ·
both host test suites pass · zero warnings across all four flag combinations.

**Known limit — IR is rate-bound.** A NEC frame is ~67 ms, so at the 112 ms cadence
the ceiling is ~9 volume steps/second. A fast flick queues more than that and the
bounded queue drops the excess (deliberately — the alternative is the LED and volume
trailing seconds behind the knob). The fix, once the radio is available to test
against, is NEC **hold-repeat** frames: a repeat is ~11 ms versus ~67 ms, so holding
ramps roughly 6x faster. The captured remote does emit clean repeats.

## ⏭️ NEXT UP — wire the UART to the Pi and test
**Mark asked to be reminded of this.** The mode layer emits NDJSON but nothing
consumes it yet.

Physical plan: ESP32 + SN65HVD230 mount **inside the HVAC module enclosure**, so the
UART run to the Pi is inches and the CAN leads stay short to the iDrive.

    ESP32 TX GPIO43 ──→ Pi GPIO15 / RXD (pin 10)
    ESP32 RX GPIO44 ←── Pi GPIO14 / TXD (pin  8)
    ESP32 GND       ─── Pi GND         (pin  6)     ← mandatory
    optional: Pi 5V (pin 2) → ESP32 VIN, keeps USB free for flashing

Pi config: `raspi-config` → Serial Port → login shell **No**, hardware **Yes**. Then
in `/boot/firmware/config.txt` add `enable_uart=1` and **`dtoverlay=disable-bt`** —
without that, `/dev/serial0` is the mini-UART whose baud tracks the VPU core clock and
corrupts under load, which the 10 Hz control loop will absolutely produce. Verify
`/dev/serial0` points at `ttyAMA0`, not `ttyS0`.

Backend: feed events into the **same handler the dashboard commands use**, so the
touchscreen updates for free and there is no second state path.

## Open next steps
- **In-car test.** Does the CP-71W respond to the LED, and at what range? If it works
  close up but not from the dash, that is drive current, not code — put the LED behind
  a 2N3904 rather than straight off the GPIO.
- **Does the radio honour NEC hold-repeats?** This decides whether volume stays as
  discrete presses or becomes assert-and-hold during sustained rotation.
- Decide the IR emit method: direct injection at the head unit's IR receiver output
  pin (preferred — hidden, no line of sight, set `IR_CARRIER_BYPASS 1`) vs the LED at
  the faceplate window (`IR_CARRIER_BYPASS 0`, currently).
- **Unresolved:** the bench loopback monitor went to 100% UNKNOWN echoes after the
  emitter was repositioned (223 garbled from 487 sends, previously 144 good / 33 bad).
  Almost certainly geometry — the discriminator is to re-run `ir_loopback`, which
  sends at a gentle 250 ms cadence and passed 15/15 before. Not blocking the in-car
  test, since the transmit path itself is verified.
- If the ladder is ever revisited: sweep 0–5 kΩ to find the fixed map, then tune
  `SWC_HOLD_MS`/`SWC_GAP_MS` down from 140/60 (current 200 ms cadence means a
  10-detent spin takes ~2 s to play out).
- Solve the bench auto-wake: capture `0x5E7`/`0x273` on the 2014 428i with canclaude
  `sniff`. F-series put the iDrive on K-CAN2 behind the ZGW, so tap a module
  connector, not the OBD port.
- Map the 9 unused inputs (UP, DOWN, MENU, BACK, OPTION, COM, MEDIA, NAV, MAP).
