# BMW F-Series iDrive — Universal CAN Input Controller

Decode all inputs from a BMW F-Series iDrive controller over CAN bus using an ESP32-S3, and route them to any output you want — GPIO, SWC voltage, BLE HID, MQTT, or anything else.

---

## Hardware

| Component | Detail |
|---|---|
| MCU | Lonely Binary ESP32-S3 Dev Module |
| CAN Transceiver | SN65HVD230 breakout (3.3V native — do NOT use MCP2551) |
| iDrive Controller | BMW F-Series Preh, part #6582 6829079-03 |
| CAN Termination | 120Ω resistor across CANH/CANL — mandatory |
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

## Adding Your Own Output

All button events flow through `onButtonPress()`. Add your output logic there:

```cpp
void onButtonPress(const char* name, uint8_t r, uint8_t g, uint8_t b) {
  Serial.printf("PRESS: %s\n", name);
  flashLED(r, g, b);

  if (strcmp(name, "KNOB_CW") == 0)  { /* volume up   */ }
  if (strcmp(name, "KNOB_CCW") == 0) { /* volume down */ }
  if (strcmp(name, "MENU") == 0)     { /* menu        */ }
  if (strcmp(name, "BACK") == 0)     { /* back        */ }
  // ... etc
}
```

### SWC Voltage Output (for aftermarket head units)

Wire an RC filter on GPIO2:
```
GPIO2 → [10kΩ] → SWC wire on head unit harness
                      |
                   [10µF]
                      |
                     GND
```

Then add PWM output:
```cpp
#define SWC_PIN  2
#define PWM_FREQ 5000
#define PWM_RES  8

void outputSWC(uint8_t duty, int holdMs = 150) {
  ledcWrite(SWC_PIN, duty);
  delay(holdMs);
  ledcWrite(SWC_PIN, 0);
}

// In setup():
ledcAttach(SWC_PIN, PWM_FREQ, PWM_RES);
ledcWrite(SWC_PIN, 0);
```

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
