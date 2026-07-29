// ═══════════════════════════════════════════════════════════════════
// ir_loopback — prove the IR TRANSMIT path before touching the radio
// Hardware: ESP32-S3 + IR LED (GPIO17) + IR receiver (GPIO18)
// Version:  1.0.0
// Author:   Mark Pelosi
// License:  MIT
// Repo:     https://github.com/pelosim/idrive-controller
// ═══════════════════════════════════════════════════════════════════
//
// WHY THIS EXISTS:
//   ir_capture proved we can READ the CP-71W remote. This proves we can
//   WRITE the same codes back out. It transmits each entry in IR_CODES,
//   listens on the receiver, and checks the decoded value matches what
//   was sent — so a wiring fault, a dead LED, a wrong resistor, or a
//   broken send path shows up here instead of being misdiagnosed as
//   "the head unit ignores us" later.
//
//   It also exercises the exact IRsend call the main sketch uses, on the
//   same core-0 task arrangement, so a working run here means the only
//   remaining unknown is whether the radio physically sees the light.
//
// WIRING:
//   IR LED:       GPIO17 → 220Ω → LED anode (longer leg)
//                 LED cathode → GND
//   IR receiver:  OUT → GPIO18,  VCC → 3.3V,  GND → GND
//
//   Point the LED at the receiver from roughly 5–30 cm. Too close can
//   saturate the receiver's AGC and cause dropouts — if you get FAILs,
//   pull them further apart before assuming anything is broken. Bouncing
//   the light off a wall or your hand also works fine.
//
// ARDUINO IDE SETTINGS: same as idrive_controller —
//   ESP32S3 Dev Module / USB CDC On Boot: Enabled / Flash 16MB / PSRAM OPI
//
// DEPENDENCIES:
//   - IRremoteESP8266
// ═══════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>

// ── Configuration ────────────────────────────────────────────────
const uint16_t kIrLedPin          = 17;
const uint16_t kRecvPin           = 18;
const uint16_t kCaptureBufferSize = 1024;
const uint8_t  kTimeout           = 50;
const uint16_t kRxWindowMs        = 600;   // how long to wait for the echo
const uint8_t  kRounds            = 3;     // full passes over the table

IRsend irsend(kIrLedPin);
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
decode_results results;

// ── The captured CP-71W codes — must match idrive_controller.ino ──
enum : uint8_t { KEY_VOL_UP = 0, KEY_VOL_DOWN, KEY_MUTE, KEY_NEXT, KEY_PREV, KEY_COUNT };

struct IrCode { decode_type_t proto; uint64_t code; uint16_t bits; };

static const char* KEY_NAME[KEY_COUNT] = {
  "VOL_UP", "VOL_DOWN", "MUTE", "NEXT", "PREV"
};
static const IrCode IR_CODES[KEY_COUNT] = {
  /* KEY_VOL_UP   */ { decode_type_t::NEC, 0xFF02FD, 32 },
  /* KEY_VOL_DOWN */ { decode_type_t::NEC, 0xFF9867, 32 },
  /* KEY_MUTE     */ { decode_type_t::NEC, 0xFFE21D, 32 },
  /* KEY_NEXT     */ { decode_type_t::NEC, 0xFF906F, 32 },
  /* KEY_PREV     */ { decode_type_t::NEC, 0xFFE01F, 32 },
};

// ── Results ──────────────────────────────────────────────────────
uint16_t passCount = 0, failCount = 0;
uint16_t perKeyPass[KEY_COUNT] = {0};
uint16_t perKeyFail[KEY_COUNT] = {0};

// Send one code, then listen for our own transmission coming back.
// Returns true when the decoded value matches exactly.
bool testKey(uint8_t key) {
  const IrCode& c = IR_CODES[key];

  // Flush anything already buffered so we can't match a stale frame.
  while (irrecv.decode(&results)) irrecv.resume();

  irsend.send(c.proto, c.code, c.bits);

  uint32_t deadline = millis() + kRxWindowMs;
  while ((int32_t)(millis() - deadline) < 0) {
    if (!irrecv.decode(&results)) { delay(2); continue; }
    if (results.repeat) { irrecv.resume(); continue; }   // hold-repeat, ignore

    uint64_t got   = results.value;
    bool     match = (got == c.code) && (results.decode_type == c.proto);

    Serial.printf("  %-9s sent 0x%llX  got 0x%llX  [%s]",
                  KEY_NAME[key],
                  (unsigned long long)c.code,
                  (unsigned long long)got,
                  match ? "PASS" : "FAIL");
    if (!match)
      Serial.printf("  (decoded as %s)", typeToString(results.decode_type).c_str());
    Serial.println();

    irrecv.resume();
    return match;
  }

  Serial.printf("  %-9s sent 0x%llX  got NOTHING  [FAIL — no echo received]\n",
                KEY_NAME[key], (unsigned long long)c.code);
  return false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }

  irsend.begin();
  irrecv.enableIRIn();

  Serial.println();
  Serial.println("═══════════════════════════════════════════════════════");
  Serial.println(" ir_loopback — transmit path verification");
  Serial.printf ("   IR LED on GPIO%u   receiver on GPIO%u\n", kIrLedPin, kRecvPin);
  Serial.printf ("   %u rounds x %u codes\n", kRounds, KEY_COUNT);
  Serial.println("═══════════════════════════════════════════════════════");
  Serial.println("Point the LED at the receiver, 5-30cm apart.");
  Serial.println();
  delay(1500);

  for (uint8_t round = 1; round <= kRounds; round++) {
    Serial.printf("── Round %u ──────────────────────────────────\n", round);
    for (uint8_t k = 0; k < KEY_COUNT; k++) {
      if (testKey(k)) { passCount++; perKeyPass[k]++; }
      else            { failCount++; perKeyFail[k]++; }
      delay(250);   // let the receiver's AGC settle between frames
    }
    Serial.println();
  }

  Serial.println("═══════════════════ SUMMARY ═══════════════════");
  for (uint8_t k = 0; k < KEY_COUNT; k++)
    Serial.printf("  %-9s %u/%u passed\n",
                  KEY_NAME[k], perKeyPass[k], perKeyPass[k] + perKeyFail[k]);
  Serial.printf("\n  TOTAL: %u passed, %u failed\n", passCount, failCount);
  if (failCount == 0) {
    Serial.println("  ✓ Transmit path verified — every code went out and");
    Serial.println("    came back byte-identical. Wire it to the radio.");
  } else {
    Serial.println("  ✗ Check: LED polarity (long leg = anode, to the resistor),");
    Serial.println("    LED on GPIO17, receiver OUT on GPIO18, both grounded,");
    Serial.println("    and try changing the LED-to-receiver distance.");
  }
  Serial.println("═══════════════════════════════════════════════");
}

void loop() { }
