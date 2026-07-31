# BMW F-Series iDrive — Universal CAN Input Controller

Decode all inputs from a BMW F-Series iDrive controller over CAN bus using an ESP32-S3, and route them to any output you want — an aftermarket head unit's SWC resistor ladder (implemented), GPIO, BLE HID, MQTT, or anything else.

---

## Hardware

| Component | Detail |
|---|---|
| MCU | Lonely Binary ESP32-S3 Dev Module |
| CAN Transceiver | SN65HVD230 breakout (3.3V native — do NOT use MCP2551) |
| iDrive Controller | BMW F-Series Preh, part #6582 6829079-03 |
| CAN Termination | 120Ω resistor across CANH/CANL — mandatory |
| Head Unit | Power Acoustik CP-71W (resistive wired-remote input) |
| SWC Ladder | 560Ω, 1.0kΩ, 1.5kΩ, 2.2kΩ, 3.9kΩ — one per function |
| Power | 12V bench supply or car battery |

---

## Wiring

```
iDrive Green wire        →  SN65HVD230 CANH
iDrive Green/Orange wire →  SN65HVD230 CANL
120Ω resistor            →  CANH to CANL

SN65HVD230 VCC   →  ESP32-S3 3.3V
SN65HVD230 GND   →  Common GND
SN65HVD230 CRX   →  ESP32-S3 GPIO8
SN65HVD230 CTX   →  ESP32-S3 GPIO4

12V supply GND   →  Common GND  ← critical, floating ground = no data
```

> ⚠️ **Do not use MCP2551** — it is a 5V device and incompatible with the ESP32-S3 without level shifting. Use SN65HVD230 (3.3V native).

> ⚠️ **120Ω termination is mandatory** — the ESP32 will crash on button press without it.

> ⚠️ **All grounds must be common** — bench supply, SN65HVD230, and ESP32 must share ground.

---

## Build Configuration

`idrive_controller.ino` ships as the **in-car build**. Compile-time flags near the
top of the file:

| Flag | In-car | Bench | Purpose |
|---|---|---|---|
| `OUT_IR` | `1` | `1` | IR remote replay — the working output path |
| `OUT_SWC` | `0` | `0` | Resistive ladder — needs a resistance sweep first |
| `IR_LOOPBACK_MONITOR` | `0` | `1` | Verify each send via a receiver on GPIO18 |
| `DEBUG_ROTATION` | `0` | `1` | Log every rotation the gate refuses, and `b2` changes |
| `IR_CARRIER_BYPASS` | `0` | `0` | `1` only when injecting at the radio's IR receiver pin |

> ⚠️ **`Serial.setTxTimeoutMs(0)` in `setup()` is load-bearing — do not remove it.**
> Hardware USB CDC blocks on write when no host drains the FIFO (`tx_timeout_ms=100`,
> up to 20 consecutive timeouts ≈ **2 s per write**). This sketch prints on every
> input event, so with no serial monitor attached — i.e. in the car — `loop()` stalls
> on every press and the NeoPixel visibly lags and drops events. Opening a terminal
> makes the symptom vanish, which makes it very easy to misdiagnose as flaky hardware.

## Arduino IDE Settings

| Setting | Value |
|---|---|
| Board | ESP32S3 Dev Module |
| USB CDC On Boot | **Enabled** ← critical |
| USB Mode | Hardware CDC and JTAG |
| Upload Mode | UART0 / Hardware CDC |
| Flash Size | 16MB |
| PSRAM | OPI PSRAM |

### Upload Sequence

If upload hangs at "Connecting...":
1. Hold **BOOT** button
2. Tap **RESET** button
3. Release **BOOT**

---

## Dependencies

Install via Arduino IDE Library Manager:
- **Adafruit NeoPixel**

---

## Complete Button Map

All inputs arrive on CAN ID `0x25B` at 500 kbps.

> ⚠️ **Do not gate rotation on `b2`.** `b2` is not the two-state flag it looks like.
> It takes at least `0x7F`, `0x80` and `0x81`, and it **latches at `0x81` during
> sustained rotation** until a knob press resets it. Versions before 1.3.0 gated
> rotation on `b2 == 0x80`, so after roughly 20 seconds of continuous turning every
> rotation event was silently discarded while the buttons kept working. Measured on
> the bench: 84 of 89 dropped rotations were `b2 == 0x81`, across only 5 `b2`
> transitions in 90 seconds. Rotation is decoded purely from the `b1` delta, and is
> suppressed only while the knob is physically pressed.
>
> ⚠️ **`0xFF` is a real encoder position, not a sentinel.** The old code used
> `lastB1 == 0xFF` to mean "no reference", which threw away a genuine detent every
> time the counter wrapped through it. Use the explicit `lastB1Valid` flag.

| Input | Byte | Detection |
|---|---|---|
| KNOB_PRESS | b3 bit 0 | Rising edge |
| KNOB_CW | b1 | Signed delta > 0 |
| KNOB_CCW | b1 | Signed delta < 0 |
| UP | b3 upper nibble | == 0x10 |
| RIGHT | b3 upper nibble | == 0x40 |
| DOWN | b3 upper nibble | == 0x70 |
| LEFT | b3 upper nibble | == 0xA0 |
| MENU | b4 | bit 2 = 0x04 |
| BACK | b4 | bit 5 = 0x20 |
| OPTION | b5 | bit 0 = 0x01 |
| COM | b5 | bit 3 = 0x08 |
| MEDIA | b6 | bit 0 (idle C0→C1) |
| NAV | b6 | bit 3 (idle C0→C8) |
| MAP | b7 | bit 0 (idle F8→F9) |

---

## Head Unit Control — SWC Resistor Ladder

Implemented as of 1.1.0. Target: **Power Acoustik CP-71W**.

### How it works

An aftermarket head unit's wired-remote input is a **resistance-to-ground ladder**.
The radio holds the line up through its own internal pull-up and measures the
resistance you place between that line and ground; each distinct resistance is one
button. A PAC SWI-RC is nothing more than this, with a CAN decoder in front — so we
skip the box and drive the ladder from the iDrive directly.

> ⚠️ **Do not use a PWM + RC filter to make a voltage** (the approach sketched in
> 1.0.0). It cannot work: a 10kΩ source impedance is the same order as the radio's
> own pull-up, so the node voltage becomes an unpredictable three-way divider that
> drifts with the unit's reference — and 10kΩ × 10µF is a 100 ms time constant,
> far too slow to step volume. Present **resistance**, not voltage.

### Wiring

```
GPIO5  ─[560Ω] ─┐
GPIO6  ─[1.0kΩ]─┤
GPIO7  ─[1.5kΩ]─┼── head unit wired-remote line
GPIO15 ─[2.2kΩ]─┤   (3.5mm TIP, or the Blue/Yellow "W/R" / "REM" wire)
GPIO16 ─[3.9kΩ]─┘
Head unit remote GND (3.5mm SLEEVE) → Common GND
```

A "press" is `pinMode(pin, OUTPUT); digitalWrite(pin, LOW)` — that leg's resistor is
tied to ground. Release is `pinMode(pin, INPUT)` — high-Z, leg disconnected. Exactly
one leg is ever asserted at a time.

> ⚠️ **Measure the remote line to ground before wiring anything.** If it sits above
> 3.3 V (many units pull up to 5 V), do **not** connect ESP32 pins to it directly —
> a pin in INPUT mode would be over-spec and its clamp diode would conduct. Put a
> 2N7000 in each leg instead: gate ← GPIO, drain → resistor → line, source → GND.
> The ESP32 then never touches the line.

### Default mapping

| iDrive input | Head unit function |
|---|---|
| Knob CW / CCW | Volume up / down (one step **per detent**) |
| Knob press | Mute |
| Right / Left | Next / Previous track |

This table is the head-unit layer only. The mode layer added since then re-targets
the same knob and d-pad depending on mode, and every labelled button now selects one:

| Button | Mode | Drives |
|---|---|---|
| MEDIA | RADIO | head unit over IR (the mapping above) |
| MENU | HVAC | the Pi climate control |
| MAP | ILLUM | the ESP-NOW lighting controller |
| NAV | GAUGE | backup gauge cluster (T-Display panels) — no Pi protocol yet |
| OPTION | TSDASH | TunerStudio dash on the TSDash Pi, via the dash_bridge board |
| BACK | — | global: SYSTEM_TOGGLE, the link status page on the HVAC screen |
| COM | — | global: AUX_SWAP, round screen clock ↔ G-meter |

GAUGE and TSDASH are two different screens and deliberately two different modes.
Nothing is unassigned now; UP and DOWN remain free *within* individual modes.

BACK stopped being a second way home in 1.8.0 — MEDIA carries the printed label for
the default mode and gets there just as fast, so one button doing it is enough.

The firmware also emits a liveness heartbeat every 2 s: `{"hb":1,"mode":...,"up":...}`.
Deliberately a different shape from an event so the Pi can drop it before the action
path. Without it the Pi cannot tell an idle knob from a cut UART, which would make
any link-health display lie.

### Resistor values

Taken from PAC's proven set (47/100/150/560/1000/1500/3900 Ω), skipping the low end
where the ESP32's ~30 Ω output-LOW impedance is a large error term — at 560 Ω and up
it is under 6%. Adjacent values are separated by ≥1.4×, wider than any radio's
detection window.

> ⚠️ **The CP-71W has no SWC learn/study mode.** Confirmed on the unit. That means it
> expects a **fixed factory ladder** whose values are undocumented and specific to this
> radio — exactly the per-radio map PAC sells programming for. The values above are
> therefore a starting guess, not a known-good map.
>
> To use the ladder you must first **discover** the real values: hand-tie single
> resistors between the remote line and ground across roughly 0–5 kΩ (these inputs are
> ADC-read, typically 0–255 over that span) and note which resistances trigger which
> function. Then put the values that hit into `SWC_MAP[]`, measuring each leg pin-to-GND
> with a DMM while that pin is driven LOW.
>
> **Because of this, IR is now the recommended path — see below.** The ladder stays
> available behind `SWC_ENABLE` in case a sweep later turns up a usable map.

### Tuning

`SWC_HOLD_MS` (140) + `SWC_GAP_MS` (60) gives a **200 ms cadence**, so a 10-detent
spin takes ~2 s to play out — the queue absorbs it, but volume visibly lags a fast
spin. Shortening both is the first thing to tune on the bench; how far you can go
depends on how often the radio samples its remote input. If it turns out the CP-71W
**auto-repeats** volume while a button is held, a better approach is to assert and
*hold* during sustained rotation rather than emitting discrete presses.

### Adding your own output

All input events flow through `onButtonPress()`, which is the integration point.
`count` is >1 only for knob rotation, where one CAN frame can carry several detents:

```cpp
void onButtonPress(const char* name, uint8_t r, uint8_t g, uint8_t b,
                   uint8_t count = 1) {
  flashLED(r, g, b);
  if (!strcmp(name, "KNOB_CW")) swcPush(SWC_VOL_UP, count);
  // ... your output here
}
```

Build with `SWC_ENABLE 0` for CAN decode only, with no ladder output.

---

## Head Unit Control — IR (recommended)

Because the CP-71W has no SWC learn mode, its resistive ladder is an undocumented
fixed map that has to be reverse-engineered. The bundled **IR remote does not have
that problem**: volume, mute, and track buttons are physically on it, so the codes
provably exist and capturing them is deterministic rather than a search.

### Step 1 — capture the codes

`ir_capture/ir_capture.ino` is a standalone sketch that reads the remote and prints a
paste-ready table.

```
IR receiver module (VS1838B / TSOP38238 / KY-022):
  OUT → ESP32 GPIO18
  VCC → 3.3V
  GND → Common GND
```

Flash it, open Serial Monitor at 115200, point the remote at the receiver, and press
VOL+, VOL−, MUTE, NEXT, PREV in turn. Press each several times — a well-behaved remote
reports the same hex code every time. Send `d` over serial to dump the table.

Almost certainly NEC at 38 kHz. If it decodes as `UNKNOWN`, the sketch falls back to
printing raw timings, which are replayable with `sendRaw()`.

### Step 2 — emit the codes

Two ways to get IR into the head unit, in order of cleanliness:

1. **Direct injection (preferred).** Tack-solder onto the *output pin* of the head
   unit's own IR receiver and drive the demodulated logic-level signal there,
   open-drain (small MOSFET or a 4066) so the original receiver still works. No line
   of sight, immune to sunlight, completely hidden, no aiming.
2. **IR LED at the faceplate.** A KY-005 / bare IR LED epoxied ~3 mm from the unit's
   IR window. Less elegant, no disassembly.

> ⚠️ **IR transmission must not run on the CAN core.** A full NEC frame is ~67 ms of
> blocking carrier work — long enough to drop several inbound `0x25B` rotation frames,
> the exact failure mode fixed in 1.1.0. Run `IRsend` in its own FreeRTOS task pinned
> to core 0 (the Arduino `loop()` runs on core 1), fed by a queue, and keep the CAN
> loop untouched.

---

## Initialization Sequence

The iDrive requires several CAN frames to wake up and activate all inputs:

| Frame | Interval | Purpose |
|---|---|---|
| 0x202: FD | On startup (twice) | Wake backlight |
| 0x0AA: 45... | Every 20ms | CAS ignition signal |
| 0x130: 45... | Every 20ms | KOMBI network alive |
| 0x440: 00 02... | Every 20ms | BMW NM alive (byte1=0x02=ALIVE) |
| 0x560: 00 02... | Every 20ms | Secondary heartbeat |
| 0x202: FD | Every 20ms | Backlight maintain |
| 0x563: 63 | Every 1000ms | iDrive primary keep-alive |
| 0x273: reply | On each 0x5E7 | Init handshake response |

> ⚠️ **Never send 0x440 with byte1=0xFF** — this is a sleep request and will cause the controller to sleep faster.

---

## Known Issues

| Issue | Status |
|---|---|
| Physical button press required to wake on bench | Open — requires live BMW CAN capture to solve |
| 0x5E7 handshake (0x273 reply) not confirmed | Open — byte4 never reaches 0x01 without live car |
| Directional pad needs knob wake on bench | Expected to work without in car |

The definitive fix for auto-wake requires sniffing `0x5E7` and `0x273` on a running BMW F-Series to capture the exact reply format the iDrive expects.

**Capture path:** the 2014 428i (F32) is the same iDrive generation as this controller.
Use the `sniff` command in [canclaude](https://github.com/pelosim) (local tool at
`~/Desktop/Claude/OBD and CAN Tool`) — its changed-bytes highlighter is built for
exactly this. Caveat: F-series route the iDrive on K-CAN2 behind the ZGW gateway, so
the OBD-II port exposes only diagnostic CAN. Expect to tap at a module connector
(head unit or ZGW) rather than at the OBD port.

---

## Web Tools

Two browser-based tools are included for development (Chrome required — uses Web Serial API):

| Tool | Purpose |
|---|---|
| `tools/idrive-mapper.html` | Capture and map button presses, export Arduino code |
| `tools/can-analyzer.html` | Full frame stream with capture, diff, and compare |

**Usage:** Close Arduino IDE Serial Monitor first, then open the tool in Chrome and click Connect.

---

## CAN Bus Notes

- **Speed:** 500 kbps (PT-CAN, not K-CAN)
- **CANH:** Green wire
- **CANL:** Green/Orange wire
- **Transceiver:** SN65HVD230 (3.3V) — not MCP2551 (5V)
- **Termination:** 120Ω across CANH/CANL required on bench setup

---

## License

MIT License — see [LICENSE](LICENSE)
