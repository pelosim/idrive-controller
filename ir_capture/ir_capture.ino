// ═══════════════════════════════════════════════════════════════════
// ir_capture — read the codes off the head unit's own IR remote
// Hardware: ESP32-S3 + any 38 kHz IR receiver (VS1838B / TSOP38238 /
//           KY-022 module from the Kuman kit)
// Version:  1.0.0
// Author:   Mark Pelosi
// License:  MIT
// Repo:     https://github.com/pelosim/idrive-controller
// ═══════════════════════════════════════════════════════════════════
//
// WHY THIS EXISTS:
//   The Power Acoustik CP-71W has no SWC "learn" mode, so its resistive
//   wired-remote ladder uses a fixed, undocumented, per-radio map that
//   would have to be found by blind sweeping. The bundled IR remote has
//   no such problem: the volume, mute, and track buttons are physically
//   on it, so the codes provably exist. Capture them once here, paste
//   the table into idrive_controller.ino, and replay them.
//
// WIRING (IR receiver module — 3 pins):
//   OUT / S / DAT  → ESP32 GPIO18
//   VCC            → ESP32 3.3V   (VS1838B and TSOP382 both run at 3.3V)
//   GND            → Common GND
//
//   If using a bare VS1838B (not a module), looking at the FRONT (domed
//   face toward you), pins left→right are OUT, GND, VCC. Add a 10kΩ
//   pull-up from OUT to 3.3V and a 100nF cap across VCC/GND if the
//   readings are noisy.
//
// HOW TO USE:
//   1. Flash this sketch, open Serial Monitor at 115200.
//   2. Point the CP-71W remote at the receiver from ~10cm.
//   3. Press ONE button. Note what it prints.
//   4. Repeat for: VOL+, VOL-, MUTE, NEXT/>>|, PREV/|<<.
//   5. Copy the emitted `IR_CODES[]` lines into idrive_controller.ino.
//
//   Press each button several times. A well-behaved remote reports the
//   SAME hex code every time, plus "REPEAT" frames if you hold it. If
//   the code changes on every press the remote is rolling-code (very
//   unlikely on a head unit) and IR replay will not work.
//
// ARDUINO IDE SETTINGS: same as idrive_controller —
//   ESP32S3 Dev Module / USB CDC On Boot: Enabled / Flash 16MB / PSRAM OPI
//
// DEPENDENCIES:
//   - IRremoteESP8266 (works on ESP32/S3 despite the name)
// ═══════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

// ── Configuration ────────────────────────────────────────────────
const uint16_t kRecvPin           = 18;    // IR receiver OUT
const uint32_t kBaudRate          = 115200;
const uint16_t kCaptureBufferSize = 1024;  // plenty for any remote
const uint8_t  kTimeout           = 50;    // ms of silence = end of frame
const uint16_t kMinUnknownSize    = 12;    // ignore shorter noise bursts

IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
decode_results results;

// ── Seen-code tracking, so repeats are obvious ───────────────────
#define MAX_SEEN 24
struct Seen { uint64_t code; uint16_t count; decode_type_t proto; uint16_t bits; };
Seen seen[MAX_SEEN];
uint8_t seenCount = 0;

// Returns how many times this code has now been observed.
uint16_t noteCode(uint64_t code, decode_type_t proto, uint16_t bits) {
  for (uint8_t i = 0; i < seenCount; i++)
    if (seen[i].code == code && seen[i].proto == proto) return ++seen[i].count;
  if (seenCount < MAX_SEEN) {
    seen[seenCount] = { code, 1, proto, bits };
    seenCount++;
  }
  return 1;
}

void dumpTable() {
  Serial.println();
  Serial.println("════════ PASTE THIS INTO idrive_controller.ino ════════");
  Serial.println("// Rows are in first-seen order. Reorder them to match the");
  Serial.println("// KEY_* order below, which is what the main sketch indexes by.");
  Serial.println("static const IrCode IR_CODES[KEY_COUNT] = {");
  static const char* slot[] = { "KEY_VOL_UP  ", "KEY_VOL_DOWN", "KEY_MUTE    ",
                                "KEY_NEXT    ", "KEY_PREV    " };
  for (uint8_t i = 0; i < seenCount; i++) {
    // Emit exactly the IrCode layout the main sketch declares — no label
    // field — so this pastes in and compiles as-is.
    Serial.printf("  /* %s */ { decode_type_t::%s, 0x%llX, %u },  // seen %u time%s\n",
                  i < 5 ? slot[i] : "??????????? ",
                  typeToString(seen[i].proto).c_str(),
                  (unsigned long long)seen[i].code,
                  seen[i].bits,
                  seen[i].count,
                  seen[i].count == 1 ? "" : "s");
  }
  Serial.println("};");
  Serial.println("═══════════════════════════════════════════════════════");
  Serial.println();
}

void setup() {
  Serial.begin(kBaudRate);
  while (!Serial && millis() < 3000) { }   // let USB CDC come up

  irrecv.setUnknownThreshold(kMinUnknownSize);
  irrecv.enableIRIn();

  Serial.println();
  Serial.println("═══════════════════════════════════════════════════════");
  Serial.println(" ir_capture — CP-71W remote code reader");
  Serial.printf ("   IR receiver OUT on GPIO%u\n", kRecvPin);
  Serial.println("═══════════════════════════════════════════════════════");
  Serial.println("Point the remote at the receiver and press one button.");
  Serial.println("Capture in this order: VOL+, VOL-, MUTE, NEXT, PREV.");
  Serial.println("Send 'd' over serial to dump the paste-ready table,");
  Serial.println("     'c' to clear and start over.");
  Serial.println();
}

void loop() {
  // Serial commands
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'd' || ch == 'D') dumpTable();
    if (ch == 'c' || ch == 'C') {
      seenCount = 0;
      Serial.println(">> cleared, capture again");
    }
  }

  if (!irrecv.decode(&results)) return;

  // A held button emits NEC "repeat" frames — real code, no new value.
  if (results.repeat) {
    Serial.println("  (repeat frame — button held)");
    irrecv.resume();
    return;
  }

  if (results.overflow)
    Serial.printf("WARNING: buffer overflow, raise kCaptureBufferSize above %u\n",
                  kCaptureBufferSize);

  String proto = typeToString(results.decode_type, results.repeat);

  if (results.decode_type == decode_type_t::UNKNOWN) {
    // Not a protocol the library knows — fall back to raw timings, which
    // can still be replayed with sendRaw().
    Serial.println("──────────────────────────────────────────────");
    Serial.println("UNKNOWN protocol — raw timing capture below.");
    Serial.printf ("  %u raw entries\n", getCorrectedRawLength(&results));
    Serial.println(resultToSourceCode(&results));
    Serial.println("──────────────────────────────────────────────");
  } else {
    uint16_t n = noteCode(results.value, results.decode_type, results.bits);
    Serial.println("──────────────────────────────────────────────");
    Serial.printf("  Protocol : %s\n", proto.c_str());
    Serial.printf("  Code     : 0x%llX\n", (unsigned long long)results.value);
    Serial.printf("  Bits     : %u\n", results.bits);
    Serial.printf("  Seen     : %u time%s\n", n, n == 1 ? "" : "s");
    if (results.decode_type == decode_type_t::NEC) {
      // A 32-bit NEC frame is addr | ~addr | cmd | ~cmd, MSB first. Both
      // inverted halves must XOR to 0xFF — that check is what separates a
      // real capture from noise or a half-seen frame.
      uint8_t addr  = (uint8_t)((results.value >> 24) & 0xFF);
      uint8_t naddr = (uint8_t)((results.value >> 16) & 0xFF);
      uint8_t cmd   = (uint8_t)((results.value >>  8) & 0xFF);
      uint8_t ncmd  = (uint8_t)( results.value        & 0xFF);
      bool valid = ((addr ^ naddr) == 0xFF) && ((cmd ^ ncmd) == 0xFF);
      Serial.printf("  NEC addr : 0x%02X   cmd: 0x%02X   checksum: %s\n",
                    addr, cmd, valid ? "VALID" : "BAD — re-capture this one");
    }
    Serial.println("──────────────────────────────────────────────");
  }

  irrecv.resume();
}
