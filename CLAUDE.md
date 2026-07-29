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

## Conventions
- **Full deployable `.ino` files, never diffs or snippets** — Mark flashes whole files.
- Nothing blocking in `loop()`. The 20 ms keep-alive cadence is load-bearing: if it
  slips, the iDrive sleeps. LED flash and SWC presses are both deadline-driven state
  machines for this reason; do not reintroduce `delay()` into the event path.
- Exactly one ladder leg asserted at a time; release is high-Z (`pinMode(pin, INPUT)`),
  never `OUTPUT`+`HIGH`.
- Never send `0x440` with byte1=`0xFF` — that is a sleep request.
- Keep the "KNOWN ISSUES" and init-sequence tables in the README intact on regen.

## Open next steps
- Bench-test the ladder against the real CP-71W. First question: does it have an SWC
  "study"/"learn" screen? If yes, values are arbitrary. If no, sweep to find them.
- Tune `SWC_HOLD_MS`/`SWC_GAP_MS` down from 140/60. Current 200 ms cadence means a
  10-detent spin takes ~2 s to play out. If the radio auto-repeats on a held button,
  switch volume to assert-and-hold during sustained rotation instead of N presses.
- Solve the bench auto-wake: capture `0x5E7`/`0x273` on the 2014 428i with canclaude
  `sniff`. F-series put the iDrive on K-CAN2 behind the ZGW, so tap a module
  connector, not the OBD port.
- Map the 9 unused inputs (UP, DOWN, MENU, BACK, OPTION, COM, MEDIA, NAV, MAP).
