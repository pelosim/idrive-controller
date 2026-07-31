// ═══════════════════════════════════════════════════════════════════
// BMW F-Series iDrive — Universal CAN Input Controller
// Hardware: Lonely Binary ESP32-S3 + SN65HVD230 CAN transceiver
// Version:  1.8.0  — BACK selects the SYSTEM STATUS screen; link heartbeat
// Author:   Mark Pelosi
// License:  MIT
// Repo:     https://github.com/pelosim/idrive-controller
// ═══════════════════════════════════════════════════════════════════
//
// DESCRIPTION:
//   Decodes all inputs from a BMW F-Series iDrive controller (Preh,
//   part #6582 6829079-03) over CAN bus and drives an aftermarket head
//   unit. Target: Power Acoustik CP-71W.
//
//   Two output backends, selected at compile time below:
//     OUT_IR   — replay the head unit's own IR remote codes  (PRIMARY)
//     OUT_SWC  — resistive wired-remote ladder               (FALLBACK)
//
//   IR is primary because the CP-71W has NO SWC learn mode: its ladder
//   is a fixed, undocumented, per-radio map that would have to be found
//   by blind sweeping. The bundled remote's buttons provably exist, so
//   capturing them is a measurement rather than a search.
//
// ── BUILD CONFIGURATION ─────────────────────────────────────────────
//   This is the IN-CAR build. Flags, and what to change for bench work:
//
//     OUT_IR              1   IR remote replay — the working path
//     OUT_SWC             0   resistive ladder — needs a sweep first
//     IR_LOOPBACK_MONITOR 0   set 1 on the bench (needs receiver GPIO18)
//     DEBUG_ROTATION      0   set 1 if rotation ever misbehaves again
//     IR_CARRIER_BYPASS   0   set 1 only if injecting at the radio's
//                             IR receiver pin instead of using an LED
//
//   IR_CODES[] is populated with real codes captured from the CP-71W
//   remote on 2026-07-28 and verified by loopback (15/15).
//
//   Serial.setTxTimeoutMs(0) in setup() is LOAD-BEARING for the in-car
//   build — see the comment there before touching it.
// ────────────────────────────────────────────────────────────────────
//
// WIRING — CAN side:
//   iDrive Green        → SN65HVD230 CANH
//   iDrive Green/Orange → SN65HVD230 CANL
//   120Ω resistor       → CANH to CANL  ← mandatory, crash without it
//   SN65HVD230 VCC      → ESP32 3.3V
//   SN65HVD230 GND      → Common GND
//   SN65HVD230 CRX      → ESP32 GPIO8
//   SN65HVD230 CTX      → ESP32 GPIO4
//   12V bench/car GND   → Common GND    ← floating ground = no data
//
// WIRING — IR output (OUT_IR):
//   GPIO17 → 220Ω → IR LED anode;  LED cathode → GND
//   For more range drive the LED through a 2N3904: GPIO17 → 1kΩ → base,
//   LED + 100Ω from 3.3V to collector, emitter → GND.
//
//   Two ways to get the light into the radio, best first:
//     1. DIRECT INJECTION — tack onto the OUTPUT pin of the head unit's
//        own IR receiver and drive the demodulated logic-level signal
//        there, open-drain (small MOSFET or 4066) so the original
//        receiver still works. No line of sight, immune to sunlight,
//        fully hidden. Note: injecting demodulated signal means you are
//        bypassing the 38 kHz carrier — see IR_CARRIER_BYPASS below.
//     2. LED AT THE FACEPLATE — IR LED epoxied ~3mm from the unit's IR
//        window. No disassembly, slightly less tidy.
//
// WIRING — SWC ladder output (OUT_SWC, fallback only):
//   GPIO5  ─[560Ω] ─┐
//   GPIO6  ─[1.0kΩ]─┤
//   GPIO7  ─[1.5kΩ]─┼── head unit wired-remote line (3.5mm TIP,
//   GPIO15 ─[2.2kΩ]─┤    or the Blue/Yellow "W/R"/"REM" wire)
//   GPIO16 ─[3.9kΩ]─┘
//   Head unit remote GND (3.5mm SLEEVE) → Common GND
//
//   ⚠ Those values are a GUESS. The CP-71W has no learn mode, so the
//     real map must be discovered by sweeping ~0–5kΩ first.
//   ⚠ Measure the remote line to ground before wiring. If it sits above
//     3.3V, put a 2N7000 in each leg (gate←GPIO, drain→resistor→line,
//     source→GND) so the ESP32 never touches the line.
//
// ARDUINO IDE SETTINGS:
//   Board:           ESP32S3 Dev Module
//   USB CDC On Boot: Enabled  ← critical for Serial output
//   USB Mode:        Hardware CDC and JTAG
//   Upload Mode:     UART0 / Hardware CDC
//   Flash Size:      16MB
//   PSRAM:           OPI PSRAM
//
// DEPENDENCIES:
//   - Adafruit NeoPixel
//   - IRremoteESP8266   (only when OUT_IR is 1; works fine on ESP32-S3)
//
// COMPLETE BUTTON MAP (CAN ID 0x25B, 500 kbps):
//   b0  — rolling counter (ignore)
//   b1  — knob absolute position (signed delta = rotation)
//   b2  — NOT a simple flag. Observed 0x7F, 0x80 and 0x81; latches at 0x81
//         during sustained rotation and is reset by a knob press. Looks like
//         -1/0/+1 around a signed centre. Logged but NOT used for gating —
//         see the rotation section for why that mattered.
//   b3  — bit0: KNOB_PRESS | upper nibble: UP=0x10 RIGHT=0x40 DOWN=0x70 LEFT=0xA0
//   b4  — MENU=0x04, BACK=0x20
//   b5  — OPTION=0x01, COM=0x08
//   b6  — MEDIA=0x01 (idle C0→C1), NAV=0x08 (idle C0→C8)
//   b7  — MAP=0x01 (idle F8→F9)
//
// INITIALIZATION SEQUENCE:
//   1. Send 0x202:FD twice      — wakes controller + enables backlight
//   2. Send 0x563:63 @1000ms    — primary iDrive keep-alive
//   3. Send 0x0AA, 0x130, 0x440, 0x560 @20ms — network keep-alive
//   4. iDrive replies with 0x5E7 — respond with 0x273 each time
//   5. 0x5E7 byte4=0x01 = fully initialized (handshake under investigation)
//
// KNOWN ISSUES:
//   - iDrive requires one physical button press to wake on bench
//   - 0x5E7 handshake reply (0x273) format not yet confirmed —
//     definitive fix requires live BMW CAN capture
//   - Directional pad may need knob press first on bench
//     (expected to work without in car)
//
// CHANGES IN 1.2.0:
//   - Added the IR output backend, now the primary path. IRsend runs in
//     its own FreeRTOS task pinned to CORE 0, fed by a queue. This is
//     not optional: a full NEC frame is ~67 ms of blocking carrier work,
//     which inline in loop() would drop several inbound 0x25B rotation
//     frames — the exact failure mode fixed in 1.1.0.
//   - Unified both backends behind one KEY_* enum and outPush(), so a
//     mapping change applies to whichever backend is compiled in.
//
// CHANGES IN 1.1.0:
//   - SWC output implemented as a switched RESISTOR LADDER, replacing
//     the planned PWM+RC voltage output (which cannot work — the head
//     unit measures resistance against its own pull-up, so a 10k/10µF
//     source is both too high-impedance and far too slow).
//   - LED flash and SWC key presses are now fully non-blocking.
//   - Knob rotation emits one volume step PER DETENT.
// ═══════════════════════════════════════════════════════════════════

#include "driver/twai.h"
#include <Adafruit_NeoPixel.h>

// ═══════════════════════════════════════════════════════════════════
// OUTPUT BACKEND SELECTION
// ═══════════════════════════════════════════════════════════════════
// Normally exactly one of these is 1. Both at once is legal (every
// input fires both backends) but only useful while bench-comparing.
#define OUT_IR   1    // IR remote replay          — primary
#define OUT_SWC  0    // resistive wired-remote     — needs a sweep first

// Bench aid: with an IR receiver wired to IR_RX_PIN, every transmitted
// code is read back and verified. OFF for the in-car install — there is
// no receiver in the car, so it would report 100% failures and burn
// cycles formatting output nobody reads. Set to 1 on the bench.
#define IR_LOOPBACK_MONITOR 0

#if OUT_IR
  #include <IRremoteESP8266.h>
  #include <IRsend.h>
  #if IR_LOOPBACK_MONITOR
    #include <IRrecv.h>
    #include <IRutils.h>
  #endif
#endif

// ── Pin definitions ──────────────────────────────────────────────
#define CTX_PIN   4    // SN65HVD230 CTX
#define CRX_PIN   8    // SN65HVD230 CRX
#define LED_PIN   48   // Onboard NeoPixel (Lonely Binary ESP32-S3)
#define IR_TX_PIN 17   // IR LED (or injection line into the head unit)
#define IR_RX_PIN 18   // IR receiver — bench loopback monitor only

// ── Tunable parameters ───────────────────────────────────────────
// LED blink timing. ON+GAP is ~115 ms, deliberately close to the IR frame
// period, so a fast knob spin produces one visible blink per detent rather
// than one continuous glow. The GAP is the important part: without an
// enforced dark period, two consecutive same-colour flashes merge into one
// and the LED stops reporting individual presses.
#define PULSE_MS      70   // LED lit duration ms
#define LED_GAP_MS    45   // enforced dark period between blinks
#define LED_QUEUE_LEN  8   // pending blinks; bounded so it cannot lag far behind
#define KEEPALIVE_MS 20    // Fast keepalive interval ms
#define SLOW_KA_MS   1000  // Slow keepalive interval ms (0x563)
#define ILLUM_LEVEL  0xFD  // Backlight brightness: 0x00=off, 0xFD=full
#define MAX_STEP     12    // clamp: max volume steps from one CAN frame
// Logs every knob movement the gate refuses, and every b2 transition.
// This is how the "rotation dies after ~20s" bug was found — set it back
// to 1 if rotation ever misbehaves again, in the car or on the bench.
#define DEBUG_ROTATION 0

// ═══════════════════════════════════════════════════════════════════
// LOGICAL KEYS — shared by both output backends
// ═══════════════════════════════════════════════════════════════════
enum : uint8_t {
  KEY_VOL_UP = 0,
  KEY_VOL_DOWN,
  KEY_MUTE,
  KEY_NEXT,
  KEY_PREV,
  KEY_COUNT
};
// Referenced only by the boot banner, which is compiled out when no
// output backend is selected — hence the unused attribute.
static const char* KEY_NAME[KEY_COUNT] __attribute__((unused)) = {
  "VOL_UP", "VOL_DOWN", "MUTE", "NEXT", "PREV"
};

// ═══════════════════════════════════════════════════════════════════
// MODE LAYER
// ═══════════════════════════════════════════════════════════════════
// The same knob and d-pad mean different things depending on mode. Mode
// is selected by the labelled buttons, whose printed names roughly match
// what they now do — that matters when operating by feel while driving.
//
//   MEDIA  head unit over IR   (the v1.3.0 behaviour, unchanged)
//   HVAC   the Pi climate control
//   LIGHT  the ESP-NOW lighting controller
//   GAUGE  the T-Display gauge panels      (backup cluster)
//   TSDASH the TunerStudio dash on the TSDash Pi
//
// GAUGE and TSDASH are two different screens and deliberately two
// different modes: GAUGE is the backup cluster built from the
// T-Display-S3-Long panels, TSDASH is the TunerStudio Pi. They share no
// transport — GAUGE goes to the Pi as an action nothing consumes yet,
// TSDASH becomes HID keystrokes on the dash_bridge board.
//
// Two rules keep it from becoming a foot-gun:
//   * The NeoPixel flashes a signature colour on every mode change, so
//     you always know what the knob is about to do.
//   * Any non-MEDIA mode auto-reverts to MEDIA after MODE_TIMEOUT_MS of
//     inactivity. Volume is what you reach for by default; it should be
//     what the knob does unless you *just* said otherwise.
//
// Only MEDIA drives real hardware today. Every other mode emits a
// structured action on the link (see LINK below) — the interaction is
// fully testable now, and the transport can be added without touching
// any of this.
// ═══════════════════════════════════════════════════════════════════

#define MODE_TIMEOUT_MS 10000   // idle revert to MEDIA

enum : uint8_t { MODE_RADIO = 0, MODE_HVAC, MODE_ILLUM, MODE_GAUGE,
                 MODE_TSDASH, MODE_COUNT };

static const char* MODE_NAME[MODE_COUNT] =
  { "RADIO", "HVAC", "ILLUM", "GAUGE", "TSDASH" };

// Signature colours. HVAC/LIGHT borrow the 944S VFD dash palette so the
// controller reads as part of the same system: amber = heat, ice blue.
//
// TSDASH is deliberately NOT another shade of blue. It is the mode most
// easily confused with GAUGE — both page a gauge screen — and telling
// them apart at a glance while driving is the entire job of this LED.
static const uint8_t MODE_RGB[MODE_COUNT][3] = {
  { 0x2C, 0xE8, 0xD8 },   // MEDIA  — phosphor teal
  { 0xFF, 0xB0, 0x00 },   // HVAC   — amber
  { 0x9C, 0x40, 0xFF },   // LIGHT  — violet
  { 0x5C, 0xB8, 0xFF },   // GAUGE  — ice blue
  { 0x3A, 0xFF, 0x8C },   // TSDASH — spring green
};

// Actions. The five MEDIA actions MUST stay first and in KEY_* order —
// dispatch converts them with (act - ACT_VOL_UP).
enum : uint8_t {
  ACT_NONE = 0,
  ACT_VOL_UP, ACT_VOL_DOWN, ACT_MUTE, ACT_NEXT, ACT_PREV,
  ACT_TEMP_UP, ACT_TEMP_DOWN, ACT_FAN_UP, ACT_FAN_DOWN,
  ACT_HVAC_MODE_PREV, ACT_HVAC_MODE_NEXT, ACT_HVAC_TOGGLE,
  ACT_LIGHT_BRIGHTER, ACT_LIGHT_DIMMER,
  ACT_LIGHT_SCENE_PREV, ACT_LIGHT_SCENE_NEXT, ACT_LIGHT_TOGGLE,
  ACT_GAUGE_SCROLL_UP, ACT_GAUGE_SCROLL_DOWN,
  ACT_GAUGE_PAGE_PREV, ACT_GAUGE_PAGE_NEXT, ACT_GAUGE_SELECT,
  ACT_TSDASH_NEXT, ACT_TSDASH_PREV, ACT_TSDASH_CFG, ACT_TSDASH_HOME,
  ACT_AUX_SWAP,           // global: round screen clock <-> g-meter
  ACT_SYSTEM_TOGGLE,      // global: HVAC screen <-> system status page
  ACT_COUNT
};

static const char* ACT_NAME[ACT_COUNT] = {
  "NONE",
  "VOL_UP", "VOL_DOWN", "MUTE", "NEXT", "PREV",
  "TEMP_UP", "TEMP_DOWN", "FAN_UP", "FAN_DOWN",
  "HVAC_MODE_PREV", "HVAC_MODE_NEXT", "HVAC_TOGGLE",
  "LIGHT_BRIGHTER", "LIGHT_DIMMER",
  "LIGHT_SCENE_PREV", "LIGHT_SCENE_NEXT", "LIGHT_TOGGLE",
  "GAUGE_SCROLL_UP", "GAUGE_SCROLL_DOWN",
  "GAUGE_PAGE_PREV", "GAUGE_PAGE_NEXT", "GAUGE_SELECT",
  "TSDASH_NEXT", "TSDASH_PREV", "TSDASH_CFG", "TSDASH_HOME",
  "AUX_SWAP", "SYSTEM_TOGGLE",
};

// What each physical input does in each mode.
struct ModeMap { uint8_t knobCW, knobCCW, knobPress, left, right, up, down; };

static const ModeMap MODE_MAP[MODE_COUNT] = {
  /* RADIO */ { ACT_VOL_UP,         ACT_VOL_DOWN,        ACT_MUTE,
                ACT_PREV,           ACT_NEXT,            ACT_NONE,          ACT_NONE },
  /* HVAC  */ { ACT_TEMP_UP,        ACT_TEMP_DOWN,       ACT_HVAC_TOGGLE,
                ACT_HVAC_MODE_PREV, ACT_HVAC_MODE_NEXT,  ACT_FAN_UP,        ACT_FAN_DOWN },
  /* ILLUM */ { ACT_LIGHT_BRIGHTER, ACT_LIGHT_DIMMER,    ACT_LIGHT_TOGGLE,
                ACT_LIGHT_SCENE_PREV, ACT_LIGHT_SCENE_NEXT, ACT_NONE,       ACT_NONE },
  /* GAUGE */ { ACT_GAUGE_SCROLL_UP, ACT_GAUGE_SCROLL_DOWN, ACT_GAUGE_SELECT,
                ACT_GAUGE_PAGE_PREV, ACT_GAUGE_PAGE_NEXT, ACT_NONE,         ACT_NONE },
  // TSDASH tilt matches the keystroke it produces: up is Ctrl+Up, down is
  // Ctrl+Down. Worth keeping if these are ever remapped — the gesture and
  // the shortcut pointing the same way is the whole reason it is learnable
  // without looking. Knob press repeats HOME as the get-me-back-out input.
  /* TSDASH */{ ACT_TSDASH_NEXT,    ACT_TSDASH_PREV,      ACT_TSDASH_HOME,
                ACT_TSDASH_PREV,    ACT_TSDASH_NEXT,      ACT_TSDASH_CFG,   ACT_TSDASH_HOME },
};

// ═══════════════════════════════════════════════════════════════════
// BUTTON MAP — the one place to remap the controller
// ═══════════════════════════════════════════════════════════════════
// Which physical iDrive button selects which mode. Edit freely. The only
// rule worth keeping is that BACK stays pointed at your default mode, so
// there is always one button that gets you home without looking.
//
// MENU is the large top-centre button and the easiest to find by feel,
// so it gets HVAC — the mode most likely to be wanted while driving.
//
//   button   mode     rationale
//   ------   -----    ------------------------------------------------
//   MEDIA    RADIO    printed label matches: volume / mute / track
//   MENU     HVAC     top centre, easiest reach, most used
//   MAP      ILLUM    interior illumination
//   NAV      GAUGE    backup gauge cluster (T-Display panels)
//   OPTION   TSDASH   TunerStudio dash on the TSDash Pi
//
// BACK used to be a second way home. The owner does not need it — MEDIA is
// the printed label for the default mode and reaches it just as fast — so
// BACK now opens the SYSTEM STATUS page instead (see GLOBAL_BUTTONS).
// The one rule worth keeping is that MEDIA still gets you home without
// looking; it is simply the only button that does.
static const struct { const char* button; uint8_t mode; } MODE_BUTTONS[] = {
  { "MEDIA",  MODE_RADIO  },
  { "MENU",   MODE_HVAC   },
  { "MAP",    MODE_ILLUM  },
  { "NAV",    MODE_GAUGE  },
  { "OPTION", MODE_TSDASH },
};

// Buttons that fire an action directly from ANY mode, bypassing the mode
// map. Add a row here for anything you want on a single press regardless
// of what the knob is currently bound to.
static const struct { const char* button; uint8_t action; uint8_t r, g, b; }
GLOBAL_BUTTONS[] = {
  // Was OPTION until TSDASH claimed that button. COM was the only free
  // input left, and this is the least-used of the two functions — a
  // one-shot toggle rather than something you sit and operate.
  { "COM",  ACT_AUX_SWAP,      0x5C, 0xB8, 0xFF },  // round screen: clock <-> G-meter
  // Global rather than a mode: the status page is read-only, so the knob
  // keeps doing whatever it was doing while you look at it. Press again
  // to dismiss — one button in, same button out.
  { "BACK", ACT_SYSTEM_TOGGLE, 0xFF, 0xFF, 0xFF },  // system status page
};

static uint8_t  activeMode    = MODE_RADIO;
static uint32_t modeTouchedAt = 0;
static uint32_t lastHeartbeat = 0;

// ═══════════════════════════════════════════════════════════════════
// LINK — structured output for everything that is not the head unit
// ═══════════════════════════════════════════════════════════════════
// Newline-delimited JSON, one object per event. Chosen because it is
// trivially parseable on the Pi (json.loads per readline), self-framing
// over a lossy serial line, and human-readable when debugging.
//
// Emitted on a HARDWARE UART, deliberately separate from the USB CDC
// console: mixing machine protocol with debug text on one stream makes
// both worse, and USB CDC on this board is also the programming port.
// Set LINK_UART 0 to emit only to the USB console.
// ═══════════════════════════════════════════════════════════════════

// Heartbeat cadence. The Pi cannot otherwise tell an idle knob from a dead
// UART: idrive_last_s only moves when someone touches the controller, so a
// status page built on it would show green straight through a cut wire.
// This is the only thing that makes "the link is up" an answerable question.
// Cheap — one buffered UART write every 2 s, nothing blocking.
#define LINK_HB_MS    2000

#define LINK_UART     1        // 1 = also write NDJSON to the hardware UART
#define LINK_BAUD     115200
#define LINK_TX_PIN   43       // board's labelled UART header (U0TXD)
#define LINK_RX_PIN   44       // (U0RXD) — change if your header differs
#define LinkSerial    Serial1

// ═══════════════════════════════════════════════════════════════════
// IR BACKEND
// ═══════════════════════════════════════════════════════════════════
// Codes come from ir_capture/ir_capture.ino. Until they are pasted in,
// every entry is 0 and the IR task silently skips it — the boot banner
// tells you how many are still unset.
//
// IR_CARRIER_BYPASS: leave 0 when driving a real IR LED (the library
// generates the 38 kHz carrier as normal). Set to 1 ONLY if you are
// injecting into the head unit's IR receiver OUTPUT pin, which expects
// an already-demodulated signal — there, the carrier must be off.
// ═══════════════════════════════════════════════════════════════════
#if OUT_IR

#define IR_QUEUE_LEN      32
// A NEC frame is ~67 ms, and the protocol repeats on a 110 ms period. 45 ms
// of quiet puts frame-to-frame at ~112 ms, matching what a real remote does —
// so the head unit sees the cadence it already expects.
#define IR_GAP_MS         45    // quiet time between consecutive frames
#define IR_TASK_STACK     6144
#define IR_TASK_CORE      0     // Arduino loop() runs on core 1
#define IR_CARRIER_BYPASS 0     // 1 = injecting at the receiver pin
#define IR_ECHO_WINDOW_MS 200   // loopback monitor: max wait for the echo

#if IR_LOOPBACK_MONITOR
// The receiver lives on CORE 1 with the Arduino loop, never inside irTask.
// Registering the IR receive interrupt from a core-0 task goes through the
// inter-processor call path and overflows ipc0's small stack — an instant
// "Stack canary watchpoint triggered (ipc0)" panic at boot. Send on core 0,
// receive on core 1.
// 15 ms end-of-frame timeout, NOT the 50 ms used in ir_capture. The timeout
// must be SHORTER than IR_GAP_MS or back-to-back sends get concatenated into
// one buffer and decode as garbage — which shows up as spurious "UNKNOWN
// echo" lines during a fast knob spin. ir_capture keeps 50 ms on purpose:
// it is a discovery tool that may meet protocols with long internal gaps.
IRrecv         irMonitor(IR_RX_PIN, 1024, 15, true);
decode_results irRes;
// Counters are diagnostics only, written on one core and read on the other;
// a stale read just misprints a tally, so no synchronisation is warranted.
static volatile uint32_t irSent = 0, irEchoed = 0;
#endif

struct IrCode {
  decode_type_t proto;
  uint64_t      code;
  uint16_t      bits;
};

// Captured from the CP-71W remote 2026-07-28 with ir_capture.
// All NEC, address 0x00, 32-bit. Every frame's inverted halves XOR to
// 0xFF (addr^~addr and cmd^~cmd both checked), and each code repeated
// identically 5-6 times, so these are solid.
//   0xFF02FD cmd 0x02      0xFF9867 cmd 0x98      0xFFE21D cmd 0xE2
//   0xFF906F cmd 0x90      0xFFE01F cmd 0xE0
static const IrCode IR_CODES[KEY_COUNT] = {
  /* KEY_VOL_UP   */ { decode_type_t::NEC, 0xFF02FD, 32 },
  /* KEY_VOL_DOWN */ { decode_type_t::NEC, 0xFF9867, 32 },
  /* KEY_MUTE     */ { decode_type_t::NEC, 0xFFE21D, 32 },
  /* KEY_NEXT     */ { decode_type_t::NEC, 0xFF906F, 32 },
  /* KEY_PREV     */ { decode_type_t::NEC, 0xFFE01F, 32 },
};

static QueueHandle_t irQueue = nullptr;

// Runs on core 0. Blocking IR work is confined here so the CAN loop on
// core 1 keeps servicing keep-alives and inbound frames uninterrupted.
static void irTask(void* arg) {
  IRsend irsend(IR_TX_PIN, false, !IR_CARRIER_BYPASS);
  irsend.begin();

  uint8_t key;
  for (;;) {
    if (xQueueReceive(irQueue, &key, portMAX_DELAY) != pdTRUE) continue;
    if (key >= KEY_COUNT) continue;
    const IrCode& c = IR_CODES[key];
    if (c.code == 0) continue;                 // not captured yet

    irsend.send(c.proto, c.code, c.bits);

#if IR_LOOPBACK_MONITOR
    irSent = irSent + 1;                       // '++' on volatile is deprecated
    Serial.printf("   IR sent  %-9s\n", KEY_NAME[key]);
#endif

    vTaskDelay(pdMS_TO_TICKS(IR_GAP_MS));
  }
}

#if IR_LOOPBACK_MONITOR
// Runs on core 1 from loop(). Each code is unique, so a decoded value maps
// straight back to the key that produced it — no cross-core coordination.
// "sent" climbing without "echo" following means the LED is not emitting.
static void irMonitorService() {
  if (!irMonitor.decode(&irRes)) return;
  if (!irRes.repeat) {
    int8_t k = -1;
    for (uint8_t i = 0; i < KEY_COUNT; i++)
      if (IR_CODES[i].code == irRes.value) { k = (int8_t)i; break; }
    irEchoed = irEchoed + 1;
    if (k >= 0)
      Serial.printf("   IR echo  %-9s VERIFIED   (sent %lu / echoed %lu)\n",
                    KEY_NAME[k], (unsigned long)irSent, (unsigned long)irEchoed);
    else
      Serial.printf("   IR echo  UNKNOWN 0x%llX — not one of ours\n",
                    (unsigned long long)irRes.value);
  }
  irMonitor.resume();
}
#endif

static void irPush(uint8_t key, uint8_t times) {
  if (!irQueue || key >= KEY_COUNT) return;
  while (times--) {
    // Non-blocking send: if the queue is full we drop rather than stall
    // the CAN loop waiting on the IR task.
    if (xQueueSend(irQueue, &key, 0) != pdTRUE) return;
  }
}

static uint8_t irUnsetCount() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < KEY_COUNT; i++) if (IR_CODES[i].code == 0) n++;
  return n;
}

#endif // OUT_IR

// ═══════════════════════════════════════════════════════════════════
// SWC RESISTOR LADDER BACKEND (fallback)
// ═══════════════════════════════════════════════════════════════════
// The head unit's wired-remote input is a resistance-to-ground ladder:
// the radio holds the line up through its own pull-up and measures the
// resistance you place between the line and ground.
//
//   assert  → pinMode(pin, OUTPUT); digitalWrite(pin, LOW)
//   release → pinMode(pin, INPUT)   (high-Z, >1 MΩ)
//   Exactly one leg is ever asserted at a time.
//
// ⚠ The CP-71W has no learn mode, so these values are unverified. The
//   real map must be found by sweeping ~0–5kΩ and noting what hits.
// ═══════════════════════════════════════════════════════════════════
#if OUT_SWC

#define SWC_HOLD_MS   140   // how long the resistance is asserted
#define SWC_GAP_MS    60    // release gap between consecutive presses
#define SWC_QUEUE_LEN 24
#define SWC_NONE      0xFF

struct SwcLeg { uint8_t pin; uint16_t ohms; };

// Pins avoid: 4/8 (CAN), 17 (IR), 48 (NeoPixel), 19/20 (USB),
//             26-37 (flash + OPI PSRAM), 0/45/46 (strapping), 43/44 (UART0)
static const SwcLeg SWC_MAP[KEY_COUNT] = {
  /* KEY_VOL_UP   */ {  5,  560 },
  /* KEY_VOL_DOWN */ {  6, 1000 },
  /* KEY_MUTE     */ {  7, 1500 },
  /* KEY_NEXT     */ { 15, 2200 },
  /* KEY_PREV     */ { 16, 3900 },
};

static uint8_t  swcQueue[SWC_QUEUE_LEN];
static uint8_t  swcHead   = 0;
static uint8_t  swcTail   = 0;
static uint8_t  swcActive = SWC_NONE;
static uint32_t swcNextAt = 0;

static void swcInit() {
  for (uint8_t i = 0; i < KEY_COUNT; i++) pinMode(SWC_MAP[i].pin, INPUT);
  swcActive = SWC_NONE;
  swcHead = swcTail = 0;
  swcNextAt = 0;
}

// Drops on overflow rather than overwriting — a corrupted queue must
// never strand a leg asserted, which would read as a held button.
static void swcPush(uint8_t key, uint8_t times) {
  if (key >= KEY_COUNT) return;
  while (times--) {
    uint8_t next = (uint8_t)((swcHead + 1) % SWC_QUEUE_LEN);
    if (next == swcTail) return;          // full
    swcQueue[swcHead] = key;
    swcHead = next;
  }
}

static void swcService() {
  uint32_t now = millis();

  if (swcActive != SWC_NONE) {
    if ((int32_t)(now - swcNextAt) >= 0) {
      pinMode(SWC_MAP[swcActive].pin, INPUT);   // high-Z = released
      swcActive = SWC_NONE;
      swcNextAt = now + SWC_GAP_MS;             // inter-press gap
    }
    return;
  }

  if ((int32_t)(now - swcNextAt) < 0) return;   // still in the gap
  if (swcHead == swcTail) return;               // nothing queued

  uint8_t key = swcQueue[swcTail];
  swcTail = (uint8_t)((swcTail + 1) % SWC_QUEUE_LEN);

  pinMode(SWC_MAP[key].pin, OUTPUT);
  digitalWrite(SWC_MAP[key].pin, LOW);          // resistor → GND
  swcActive = key;
  swcNextAt = now + SWC_HOLD_MS;
}

#endif // OUT_SWC

// ── Unified output dispatch ───────────────────────────────────────
static void outPush(uint8_t key, uint8_t times) {
#if OUT_IR
  irPush(key, times);
#endif
#if OUT_SWC
  swcPush(key, times);
#endif
#if !OUT_IR && !OUT_SWC
  (void)key; (void)times;
#endif
}

// ── Globals ──────────────────────────────────────────────────────
Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastKeepalive = 0;
unsigned long lastSlowKA    = 0;
bool          initialized   = false;

// Previous frame state for edge detection
uint8_t lastB1      = 0x00;
bool    lastB1Valid = false;   // never use 0xFF as a sentinel — it is a real position
uint8_t lastB3 = 0x00;
uint8_t lastB4 = 0x00;
uint8_t lastB5 = 0x00;
uint8_t lastB6 = 0xC0;
uint8_t lastB7 = 0xF8;

// ── LED helpers — non-blocking, one distinct blink per press ──────
// Three states: IDLE -> ON (lit PULSE_MS) -> GAP (dark LED_GAP_MS) -> IDLE.
// Blinks queue rather than overwrite, so rapid presses of the SAME colour
// still read as separate flashes. The queue is bounded: on a sustained fast
// spin the LED drops extras instead of trailing seconds behind the knob.
struct LedReq { uint8_t r, g, b; };
static LedReq   ledQueue[LED_QUEUE_LEN];
static uint8_t  ledHead = 0, ledTail = 0;
static uint8_t  ledState = 0;          // 0 = idle, 1 = lit, 2 = dark gap
static uint32_t ledUntil = 0;

void flashLED(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t next = (uint8_t)((ledHead + 1) % LED_QUEUE_LEN);
  if (next == ledTail) return;         // full — drop, never block
  ledQueue[ledHead] = { r, g, b };
  ledHead = next;
}

void ledService() {
  uint32_t now = millis();

  if (ledState == 1) {                 // lit — time to go dark?
    if ((int32_t)(now - ledUntil) < 0) return;
    led.setPixelColor(0, led.Color(0, 0, 0));
    led.show();
    ledState = 2;
    ledUntil = now + LED_GAP_MS;
    return;
  }

  if (ledState == 2) {                 // enforced dark gap
    if ((int32_t)(now - ledUntil) < 0) return;
    ledState = 0;
    // deliberate fall-through: start the next queued blink on this same
    // tick, so cadence is exactly PULSE_MS + LED_GAP_MS with no wasted cycle
  }

  if (ledHead == ledTail) return;      // idle, nothing pending
  const LedReq& q = ledQueue[ledTail];
  ledTail = (uint8_t)((ledTail + 1) % LED_QUEUE_LEN);
  led.setPixelColor(0, led.Color(q.r, q.g, q.b));
  led.show();
  ledState = 1;
  ledUntil = now + PULSE_MS;
}

// Blocking blink — setup() only, where stalling is harmless.
void blinkBlocking(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
  delay(200);
  led.setPixelColor(0, led.Color(0, 0, 0));
  led.show();
}

// ── Generic CAN frame transmit ────────────────────────────────────
void sendFrame(uint16_t id, uint8_t len, uint8_t* data) {
  twai_message_t msg;
  msg.identifier       = id;
  msg.data_length_code = len;
  msg.extd             = 0;
  msg.rtr              = 0;
  msg.ss               = 0;
  msg.self             = 0;
  msg.dlc_non_comp     = 0;
  memcpy(msg.data, data, len);
  twai_transmit(&msg, pdMS_TO_TICKS(5));
}

// ── Button/input event handler ────────────────────────────────────
// This is your integration point. `count` is >1 only for knob rotation,
// where one CAN frame can carry several detents.
// ── Link: emit one structured event ───────────────────────────────
static void linkEmit(const char* action, uint8_t count) {
  char buf[112];
  int n = snprintf(buf, sizeof buf,
                   "{\"mode\":\"%s\",\"action\":\"%s\",\"count\":%u}\n",
                   MODE_NAME[activeMode], action, (unsigned)count);
  if (n <= 0) return;
  if (n > (int)sizeof buf) n = sizeof buf - 1;
#if LINK_UART
  LinkSerial.write((const uint8_t*)buf, n);
#endif
  Serial.printf("   -> %s", buf);   // human echo on the USB console
}

// Liveness only. Deliberately a DIFFERENT shape from an event — the Pi keys
// on "hb" and drops it before the action path, so a heartbeat can never be
// mistaken for input and leave the on-screen knob mirror permanently awake.
// No console echo: this fires forever and would bury real events.
static void linkHeartbeat() {
  char buf[96];
  int n = snprintf(buf, sizeof buf,
                   "{\"hb\":1,\"mode\":\"%s\",\"up\":%lu}\n",
                   MODE_NAME[activeMode], (unsigned long)(millis() / 1000));
  if (n <= 0) return;
  if (n > (int)sizeof buf) n = sizeof buf - 1;
#if LINK_UART
  LinkSerial.write((const uint8_t*)buf, n);
#endif
}

// ── Mode switching ────────────────────────────────────────────────
static void modeSet(uint8_t m) {
  if (m >= MODE_COUNT) return;
  bool changed = (m != activeMode);
  activeMode    = m;
  modeTouchedAt = millis();
  if (changed) {
    Serial.printf("MODE -> %s\n", MODE_NAME[m]);
    linkEmit("MODE_ENTER", 1);
  }
  flashLED(MODE_RGB[m][0], MODE_RGB[m][1], MODE_RGB[m][2]);
}

// Auto-revert to MEDIA so the knob is never unexpectedly wired to
// something else. Called every loop.
static void modeService() {
  if (activeMode == MODE_RADIO) return;
  if ((int32_t)(millis() - (modeTouchedAt + MODE_TIMEOUT_MS)) >= 0) {
    Serial.println("MODE timeout — reverting to MEDIA");
    modeSet(MODE_RADIO);
  }
}

// ── Action dispatch ───────────────────────────────────────────────
// The five MEDIA actions are contiguous and in KEY_* order, so they
// convert directly. Everything else goes out on the link.
static void dispatchAction(uint8_t act, uint8_t count) {
  if (act == ACT_NONE || act >= ACT_COUNT) return;
  modeTouchedAt = millis();          // any real action keeps the mode alive

  if (act >= ACT_VOL_UP && act <= ACT_PREV) {
    outPush((uint8_t)(act - ACT_VOL_UP), count);
  }

  // EVERY action is reported on the link, MEDIA included. The head unit is
  // driven by IR and the Pi plays no part in that — but the Pi still needs
  // to see volume and track events to animate the on-screen knob mirror.
  // Reporting is not routing: the IR path above is what actually acts.
  linkEmit(ACT_NAME[act], count);
}

void onButtonPress(const char* name, uint8_t r, uint8_t g, uint8_t b,
                   uint8_t count = 1) {
  if (count > 1) Serial.printf("PRESS: %s x%u\n", name, count);
  else           Serial.printf("PRESS: %s\n", name);

  // ── Mode selection — driven entirely by MODE_BUTTONS above ────
  // These never produce an action; they only re-point the knob.
  for (const auto& mb : MODE_BUTTONS)
    if (!strcmp(name, mb.button)) { modeSet(mb.mode); return; }

  // ── Global actions — driven by GLOBAL_BUTTONS, any mode ───────
  for (const auto& gb : GLOBAL_BUTTONS)
    if (!strcmp(name, gb.button)) {
      flashLED(gb.r, gb.g, gb.b);
      dispatchAction(gb.action, 1);
      return;
    }

  flashLED(r, g, b);

  // ── Everything else is interpreted through the active mode ────
  const ModeMap& mm = MODE_MAP[activeMode];
  uint8_t act = ACT_NONE;
  if      (!strcmp(name, "KNOB_CW"))    act = mm.knobCW;
  else if (!strcmp(name, "KNOB_CCW"))   act = mm.knobCCW;
  else if (!strcmp(name, "KNOB_PRESS")) act = mm.knobPress;
  else if (!strcmp(name, "LEFT"))       act = mm.left;
  else if (!strcmp(name, "RIGHT"))      act = mm.right;
  else if (!strcmp(name, "UP"))         act = mm.up;
  else if (!strcmp(name, "DOWN"))       act = mm.down;

  // Rotation carries a detent count; discrete presses are always 1.
  bool isRotation = !strcmp(name, "KNOB_CW") || !strcmp(name, "KNOB_CCW");
  dispatchAction(act, isRotation ? count : 1);

  // Still unmapped and free for future use: OPTION, COM.
}

// ── Fast keepalive — send every KEEPALIVE_MS ─────────────────────
// No inter-frame delays: the TWAI TX queue (raised to 10 in setup)
// absorbs the burst. The old delay(1) per frame burned 5 ms of every
// 20 ms cycle and blocked reception for that whole window.
void sendKeepalive() {
  // CAS ignition frame — signals key-on to iDrive
  uint8_t cas[8]   = {0x45,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  sendFrame(0x0AA, 8, cas);

  // KOMBI network alive — signals instrument cluster present
  uint8_t kombi[8] = {0x45,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  sendFrame(0x130, 8, kombi);

  // BMW NM alive — byte1=0x02=ALIVE (0xFF=sleep, never send)
  uint8_t nm[8]    = {0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00};
  sendFrame(0x440, 8, nm);

  // Secondary heartbeat
  uint8_t hb[8]    = {0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00};
  sendFrame(0x560, 8, hb);

  // Illumination — adjust ILLUM_LEVEL to change brightness
  uint8_t illum[8] = {ILLUM_LEVEL,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  sendFrame(0x202, 2, illum);
}

// ── Slow keepalive — send every SLOW_KA_MS ───────────────────────
void sendSlowKeepalive() {
  uint8_t ka[8] = {0x63,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  sendFrame(0x563, 1, ka);
}

// ── 0x5E7 initialization handshake ───────────────────────────────
// iDrive broadcasts 0x5E7 and expects 0x273 reply.
// byte4=0x01 indicates full initialization.
// NOTE: exact 0x273 reply format not yet confirmed from live car.
void handle5E7(twai_message_t& msg) {
  uint8_t reply[7] = {0x1D, 0xE1, 0x00, 0xF0, 0xFF, 0x7F, msg.data[2]};
  sendFrame(0x273, 7, reply);

  if (msg.data[4] == 0x01 && !initialized) {
    initialized = true;
    Serial.println("iDrive FULLY INITIALIZED — all inputs active");
    flashLED(0, 255, 0);
  }
}

// ── Main CAN frame decoder ────────────────────────────────────────
void handleFrame(twai_message_t& msg) {

  // Handle init handshake on every 0x5E7
  if (msg.identifier == 0x5E7) {
    handle5E7(msg);
    return;
  }

  // Only process input frames
  if (msg.identifier != 0x25B) return;

  uint8_t b1 = msg.data[1];
  uint8_t b2 = msg.data[2];
  uint8_t b3 = msg.data[3];
  uint8_t b4 = msg.data[4];
  uint8_t b5 = msg.data[5];
  uint8_t b6 = msg.data[6];
  uint8_t b7 = msg.data[7];

  // b2 is deliberately not used for anything now — gating rotation on it
  // was the v1.0.0 bug. Kept named for frame-layout clarity and read by
  // the rotation diagnostics when DEBUG_ROTATION is on.
#if !DEBUG_ROTATION
  (void)b2;
#endif

  // ── Knob press — bit0 of b3, rising edge ─────────────────────
  if ((b3 & 0x01) && !(lastB3 & 0x01))
    onButtonPress("KNOB_PRESS", 0,   0,   255); // blue

  // ── Directional pad — exact match on upper nibble of b3 ──────
  // DOWN=0x70 and LEFT=0xA0 have multiple bits set by hardware design
  uint8_t dirNow  = b3  & 0xF0;
  uint8_t dirLast = lastB3 & 0xF0;
  if (dirNow != dirLast && dirNow != 0x00) {
    if      (dirNow == 0x10) onButtonPress("UP",     0,   255, 255); // cyan
    else if (dirNow == 0x40) onButtonPress("RIGHT",  255, 255, 0  ); // yellow
    else if (dirNow == 0x70) onButtonPress("DOWN",   255, 128, 0  ); // orange
    else if (dirNow == 0xA0) onButtonPress("LEFT",   128, 0,   255); // purple
    else Serial.printf("DIR_UNKNOWN b3=0x%02X\n", b3);
  }

  // ── b4 bitmask — MENU, BACK ───────────────────────────────────
  uint8_t b4diff = b4 ^ lastB4;
  if (b4diff & 0x04 && b4 & 0x04) onButtonPress("MENU",   255, 0,   255); // magenta
  if (b4diff & 0x20 && b4 & 0x20) onButtonPress("BACK",   255, 50,  50 ); // pink

  // ── b5 bitmask — OPTION, COM ──────────────────────────────────
  uint8_t b5diff = b5 ^ lastB5;
  if (b5diff & 0x01 && b5 & 0x01) onButtonPress("OPTION", 255, 165, 0  ); // amber
  if (b5diff & 0x08 && b5 & 0x08) onButtonPress("COM",    0,   255, 128); // mint

  // ── b6 bitmask — MEDIA, NAV (idle=0xC0) ──────────────────────
  uint8_t b6diff = b6 ^ lastB6;
  if (b6diff & 0x01 && b6 & 0x01) onButtonPress("MEDIA",  0,   200, 255); // light blue
  if (b6diff & 0x08 && b6 & 0x08) onButtonPress("NAV",    100, 255, 100); // light green

  // ── b7 bitmask — MAP (idle=0xF8) ─────────────────────────────
  uint8_t b7diff = b7 ^ lastB7;
  if (b7diff & 0x01 && b7 & 0x01) onButtonPress("MAP",    255, 200, 0  ); // gold

#if DEBUG_ROTATION
  // Rotation dies after sustained spinning while buttons keep working —
  // so the fault is in the gate below, not in frame reception. Log what
  // b2 actually does, and every b1 change the gate throws away.
  {
    static uint8_t dbgLastB2  = 0x00;
    static bool    dbgB2Init  = false;
    static uint32_t dbgIgnored = 0;
    if (!dbgB2Init) { dbgLastB2 = b2; dbgB2Init = true;
      Serial.printf("  [dbg] initial b2=0x%02X b1=0x%02X\n", b2, b1); }
    else if (b2 != dbgLastB2) {
      Serial.printf("  [dbg] b2 CHANGED 0x%02X -> 0x%02X   (b1=0x%02X b3=0x%02X)\n",
                    dbgLastB2, b2, b1, b3);
      dbgLastB2 = b2;
    }
    // Did the knob move but the gate refuse it? After 1.3.0 the only
    // legitimate reason is "knob is currently pressed".
    if (b1 != lastB1 && !(lastB1Valid && !(b3 & 0x01))) {
      dbgIgnored++;
      Serial.printf("  [dbg] ROTATION DROPPED #%lu  b1 0x%02X->0x%02X  "
                    "b2=0x%02X %s%s\n",
                    (unsigned long)dbgIgnored, lastB1, b1, b2,
                    (b3 & 0x01)   ? "[knob pressed - expected] " : "",
                    (!lastB1Valid) ? "[no position reference yet]" : "");
    }
  }
#endif

  // ── Knob rotation ─────────────────────────────────────────────
  // b1 is an absolute 8-bit position counter; rotation is its delta.
  //
  // DO NOT gate this on b2 == 0x80 (as versions before 1.3.0 did). b2 is
  // not the two-state flag it was assumed to be — it takes at least 0x7F,
  // 0x80 and 0x81, and it LATCHES at 0x81 during sustained rotation. That
  // silently killed every rotation event until a knob press reset it,
  // while the buttons carried on working: the "encoder dies after ~20s but
  // forward/back and press still respond" bug. Measured on the bench:
  // 84 of 89 dropped rotations were b2 == 0x81.
  //
  // lastB1Valid replaces the old "lastB1 == 0xFF means invalid" sentinel.
  // 0xFF is a legitimate encoder position, so the sentinel threw away a
  // real detent every time the counter wrapped through it, and could wedge.
  //
  // int8_t signed cast handles byte wraparound (e.g. 0xFF→0x01 = +2 not -254).
  // The magnitude is the detent count for this frame — emit that many
  // volume steps, not one. Promote to int16_t before negating so that
  // delta == -128 cannot overflow, then clamp against glitch frames.
  if (lastB1Valid && !(b3 & 0x01)) {
    int8_t delta = (int8_t)(b1 - lastB1);
    if (delta != 0) {
      int16_t mag = delta > 0 ? (int16_t)delta : -(int16_t)delta;
      if (mag > MAX_STEP) mag = MAX_STEP;
      if (delta > 0) onButtonPress("KNOB_CW",  0,   255, 0, (uint8_t)mag);
      else           onButtonPress("KNOB_CCW", 255, 0,   0, (uint8_t)mag);
    }
  }

  // ── Store previous state ──────────────────────────────────────
  // While the knob is pressed, drop the position reference entirely so the
  // press itself cannot register as rotation; it is re-established on the
  // first frame after release.
  if (b3 & 0x01) {
    lastB1Valid = false;
  } else {
    lastB1      = b1;
    lastB1Valid = true;
  }
  lastB3 = b3;
  lastB4 = b4;
  lastB5 = b5;
  lastB6 = b6;
  lastB7 = b7;
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // ── CRITICAL: do not remove ────────────────────────────────────
  // Hardware USB CDC writes BLOCK when no host is draining the FIFO.
  // The core defaults to tx_timeout_ms=100 with up to 20 consecutive
  // timeouts, so ONE Serial.printf can stall for ~2 seconds once the
  // buffer backs up. This sketch prints on every input event, so with
  // no serial monitor attached — i.e. installed in the car — loop()
  // stalls on every press, and the NeoPixel lags and skips events.
  // The symptom vanishes the instant a terminal is opened, which makes
  // it very easy to misdiagnose as flaky hardware.
  // 0 = never block; output is simply discarded when nobody listens.
  Serial.setTxTimeoutMs(0);

#if LINK_UART
  // Machine protocol on its own hardware UART, away from the USB console.
  LinkSerial.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
#endif

#if OUT_SWC
  swcInit();   // all ladder legs high-Z before anything else
#endif

#if OUT_IR
  #if IR_LOOPBACK_MONITOR
    irMonitor.enableIRIn();   // MUST be here on core 1, never inside irTask
  #endif
  irQueue = xQueueCreate(IR_QUEUE_LEN, sizeof(uint8_t));
  xTaskCreatePinnedToCore(irTask, "irTask", IR_TASK_STACK,
                          nullptr, 1, nullptr, IR_TASK_CORE);
#endif

  led.begin();
  led.setBrightness(40);
  led.setPixelColor(0, led.Color(0, 0, 0));
  led.show();

  // Init TWAI (ESP32 built-in CAN controller)
  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CTX_PIN,
                                (gpio_num_t)CRX_PIN,
                                TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 10;   // absorb the 5-frame keepalive burst
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  // Pre-wake backlight — send 0x202 twice before main loop
  uint8_t illum[8] = {ILLUM_LEVEL,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  sendFrame(0x202, 2, illum); delay(5);
  sendFrame(0x202, 2, illum); delay(5);

  // Wake loop — wait for first iDrive frame
  Serial.println("Waking iDrive...");
  bool awake = false;
  unsigned long wakeStart = millis();

  while (!awake) {
    sendKeepalive();
    sendSlowKeepalive();
    delay(20);

    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(5)) == ESP_OK) {
      handleFrame(msg);
      if (msg.identifier == 0x25B || msg.identifier == 0x5E7) {
        awake = true;
        Serial.printf("iDrive awake in %lums\n", millis() - wakeStart);
      }
    }

    if (millis() - wakeStart > 10000) {
      Serial.println("Wake timeout — physical button press required");
      break;
    }
  }

  // Discard anything the wake loop queued — no phantom output on boot.
#if OUT_SWC
  swcInit();
#endif
#if OUT_IR
  xQueueReset(irQueue);
#endif
  ledHead = ledTail = 0;   // no leftover blinks from the wake loop
  ledState = 0;

  blinkBlocking(255, 255, 255); // white = ready
  Serial.println("iDrive ready — all 14 inputs mapped");

#if OUT_IR
  Serial.printf("IR output ACTIVE on GPIO%u (task on core %u)%s\n",
                IR_TX_PIN, IR_TASK_CORE,
                IR_CARRIER_BYPASS ? ", carrier BYPASSED" : "");
#if IR_LOOPBACK_MONITOR
  Serial.printf("  loopback monitor ON — receiver on GPIO%u verifies each send\n",
                IR_RX_PIN);
#endif
  uint8_t unset = irUnsetCount();
  if (unset) {
    Serial.printf("  ⚠ %u of %u IR codes are still placeholders — "
                  "run ir_capture and paste them in.\n", unset, KEY_COUNT);
    for (uint8_t i = 0; i < KEY_COUNT; i++)
      if (IR_CODES[i].code == 0) Serial.printf("      unset: %s\n", KEY_NAME[i]);
  } else {
    Serial.println("  all 5 IR codes populated");
  }
#endif
#if OUT_SWC
  Serial.println("SWC ladder ACTIVE (values UNVERIFIED — no learn mode on this radio):");
  for (uint8_t i = 0; i < KEY_COUNT; i++)
    Serial.printf("  %-9s GPIO%-2u  %u ohm\n",
                  KEY_NAME[i], SWC_MAP[i].pin, SWC_MAP[i].ohms);
#endif
#if !OUT_IR && !OUT_SWC
  Serial.println("No output backend compiled in — decode only.");
#endif

#if LINK_UART
  Serial.printf("LINK: NDJSON on UART TX=GPIO%u RX=GPIO%u @ %u baud\n",
                LINK_TX_PIN, LINK_RX_PIN, (unsigned)LINK_BAUD);
#else
  Serial.println("LINK: USB console only (LINK_UART=0)");
#endif
  Serial.printf("MODE: %s  (auto-revert after %ums idle)\n",
                MODE_NAME[activeMode], (unsigned)MODE_TIMEOUT_MS);
  // Print the live map rather than a hardcoded string, so the banner can
  // never drift from MODE_BUTTONS the way the old one did.
  Serial.print("  ");
  for (const auto& mb : MODE_BUTTONS)
    Serial.printf("%s=%s  ", mb.button, MODE_NAME[mb.mode]);
  for (const auto& gb : GLOBAL_BUTTONS)
    Serial.printf("%s=%s  ", gb.button, ACT_NAME[gb.action]);
  Serial.println();

  modeTouchedAt = millis();
  Serial.println("Waiting for 0x5E7 init handshake...");
}

// ── Main loop ─────────────────────────────────────────────────────
void loop() {
  if (millis() - lastKeepalive >= KEEPALIVE_MS) {
    sendKeepalive();
    lastKeepalive = millis();
  }

  if (millis() - lastSlowKA >= SLOW_KA_MS) {
    sendSlowKeepalive();
    lastSlowKA = millis();
  }

  twai_message_t msg;
  if (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
    handleFrame(msg);
  }

  if (millis() - lastHeartbeat >= LINK_HB_MS) {
    linkHeartbeat();
    lastHeartbeat = millis();
  }

  modeService();  // revert to MEDIA after an idle period
#if OUT_SWC
  swcService();   // advance the ladder press state machine
#endif
#if OUT_IR && IR_LOOPBACK_MONITOR
  irMonitorService();   // confirm each transmitted code actually went out
#endif
  ledService();   // turn the NeoPixel back off when its time is up
}
