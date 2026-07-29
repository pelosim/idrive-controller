// ═══════════════════════════════════════════════════════════════════
// BMW F-Series iDrive — Universal CAN Input Controller
// Hardware: Lonely Binary ESP32-S3 + SN65HVD230 CAN transceiver
// Version:  1.1.0
// Author:   Mark Pelosi
// License:  MIT
// Repo:     https://github.com/pelosim/idrive-controller
// ═══════════════════════════════════════════════════════════════════
//
// DESCRIPTION:
//   Decodes all inputs from a BMW F-Series iDrive controller (Preh,
//   part #6582 6829079-03) over CAN bus and drives an aftermarket head
//   unit's wired-remote (SWC) input via a switched resistor ladder.
//   Default target: Power Acoustik CP-71W.
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
// WIRING — SWC ladder output (see SWC section below for full notes):
//   GPIO5  ─[560Ω] ─┐
//   GPIO6  ─[1.0kΩ]─┤
//   GPIO7  ─[1.5kΩ]─┼── head unit wired-remote line (3.5mm TIP,
//   GPIO15 ─[2.2kΩ]─┤    or the Blue/Yellow "W/R"/"REM" wire)
//   GPIO16 ─[3.9kΩ]─┘
//   Head unit remote GND (3.5mm SLEEVE) → Common GND
//
//   ⚠ BEFORE WIRING: measure the head unit's remote line to ground with
//     nothing connected. If it sits above 3.3V (many units pull up to
//     5V), do NOT drive the GPIOs into it directly — put a 2N7000 per
//     leg (gate←GPIO, drain→resistor→line, source→GND) so the ESP32
//     never touches the line. See the SWC section for detail.
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
//   - Adafruit NeoPixel (install via Library Manager)
//
// COMPLETE BUTTON MAP (CAN ID 0x25B, 500 kbps):
//   b0  — rolling counter (ignore)
//   b1  — knob absolute position (signed delta = rotation)
//   b2  — 0x7F=knob pressed, 0x80=at rest
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
// CHANGES IN 1.1.0:
//   - SWC output implemented as a switched RESISTOR LADDER, replacing
//     the planned PWM+RC voltage output (which cannot work — the head
//     unit measures resistance against its own pull-up, so a 10k/10µF
//     source is both too high-impedance and far too slow).
//   - LED flash and SWC key presses are now fully non-blocking. The old
//     delay(150) inside onButtonPress() stalled the 20 ms keep-alives
//     and dropped inbound frames on every press.
//   - Knob rotation now emits one volume step PER DETENT. Previously a
//     multi-count delta fired a single event, losing most of a fast spin.
// ═══════════════════════════════════════════════════════════════════

#include "driver/twai.h"
#include <Adafruit_NeoPixel.h>

// ── Pin definitions ──────────────────────────────────────────────
#define CTX_PIN  4    // SN65HVD230 CTX
#define CRX_PIN  8    // SN65HVD230 CRX
#define LED_PIN  48   // Onboard NeoPixel (Lonely Binary ESP32-S3)

// ── Tunable parameters ───────────────────────────────────────────
#define PULSE_MS     150   // LED flash duration ms
#define KEEPALIVE_MS 20    // Fast keepalive interval ms
#define SLOW_KA_MS   1000  // Slow keepalive interval ms (0x563)
#define ILLUM_LEVEL  0xFD  // Backlight brightness: 0x00=off, 0xFD=full

// ═══════════════════════════════════════════════════════════════════
// SWC RESISTOR LADDER OUTPUT
// ═══════════════════════════════════════════════════════════════════
// The head unit's wired-remote input is a resistance-to-ground ladder:
// the radio holds the line up through its own internal pull-up and
// measures the resistance you place between the line and ground. Each
// distinct resistance = one button. This is exactly what a PAC SWI-RC
// does internally; we just drive it from the iDrive instead of from a
// steering wheel.
//
// HOW A "PRESS" WORKS:
//   assert  → pinMode(pin, OUTPUT); digitalWrite(pin, LOW)
//             that leg's resistor is now tied to ground.
//   release → pinMode(pin, INPUT)
//             high-Z (>1 MΩ), leg effectively disconnected.
//   Exactly one leg is ever asserted at a time.
//
// RESISTOR VALUES:
//   Chosen from PAC's proven set (47/100/150/560/1000/1500/3900Ω),
//   skipping the low end where the ESP32's ~30Ω output-LOW impedance
//   would be a large error term. At 560Ω and up that 30Ω is under 6%.
//   Every adjacent pair is separated by ≥1.4x, wider than any radio's
//   detection window — so mis-reads are very unlikely.
//
// CALIBRATION:
//   Values only need to be *stable and well separated* if the unit has
//   an SWC "study"/"learn" screen — you teach it whatever you output.
//   If it expects a fixed factory ladder instead, sweep: temporarily
//   tie single resistors from the line to ground by hand and watch what
//   the radio does, then set the table below to the values that hit.
//   Measure each leg's true resistance pin-to-GND with a DMM while that
//   pin is driven LOW, and record the real number here.
// ═══════════════════════════════════════════════════════════════════

#define SWC_ENABLE    1     // set 0 to build with CAN decode only
#define SWC_HOLD_MS   140   // how long the resistance is asserted
#define SWC_GAP_MS    60    // release gap between consecutive presses
#define SWC_QUEUE_LEN 24    // pending presses (a fast spin queues many)
#define SWC_MAX_STEP  12    // clamp: max volume steps from one frame

enum : uint8_t {
  SWC_VOL_UP = 0,
  SWC_VOL_DOWN,
  SWC_MUTE,
  SWC_NEXT,
  SWC_PREV,
  SWC_COUNT
};
#define SWC_NONE 0xFF

struct SwcLeg {
  uint8_t     pin;    // ESP32-S3 GPIO driving this leg
  uint16_t    ohms;   // resistor fitted in series with that pin
  const char* label;
};

// Pins avoid: 4/8 (CAN), 48 (NeoPixel), 19/20 (USB),
//             26-37 (flash + OPI PSRAM), 0/45/46 (strapping), 43/44 (UART0)
static const SwcLeg SWC_MAP[SWC_COUNT] = {
  {  5,  560, "VOL_UP"   },
  {  6, 1000, "VOL_DOWN" },
  {  7, 1500, "MUTE"     },
  { 15, 2200, "NEXT"     },
  { 16, 3900, "PREV"     },
};

static uint8_t  swcQueue[SWC_QUEUE_LEN];
static uint8_t  swcHead   = 0;          // write index
static uint8_t  swcTail   = 0;          // read index
static uint8_t  swcActive = SWC_NONE;   // leg currently asserted
static uint32_t swcNextAt = 0;          // release deadline, or gap end

// ── Globals ──────────────────────────────────────────────────────
Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastKeepalive = 0;
unsigned long lastSlowKA    = 0;
bool          initialized   = false;
static uint32_t ledOffAt    = 0;   // 0 = LED already off

// Previous frame state for edge detection
uint8_t lastB1 = 0xFF;
uint8_t lastB3 = 0x00;
uint8_t lastB4 = 0x00;
uint8_t lastB5 = 0x00;
uint8_t lastB6 = 0xC0;
uint8_t lastB7 = 0xF8;

// ── SWC: release every leg to high-Z, drop anything queued ────────
void swcInit() {
#if SWC_ENABLE
  for (uint8_t i = 0; i < SWC_COUNT; i++) pinMode(SWC_MAP[i].pin, INPUT);
#endif
  swcActive = SWC_NONE;
  swcHead = swcTail = 0;
  swcNextAt = 0;
}

// ── SWC: queue `times` presses of one key ─────────────────────────
// Drops on overflow rather than overwriting — a corrupted queue must
// never strand a leg asserted, which would read as a held button.
void swcPush(uint8_t key, uint8_t times) {
#if SWC_ENABLE
  if (key >= SWC_COUNT) return;
  while (times--) {
    uint8_t next = (uint8_t)((swcHead + 1) % SWC_QUEUE_LEN);
    if (next == swcTail) return;          // full
    swcQueue[swcHead] = key;
    swcHead = next;
  }
#else
  (void)key; (void)times;
#endif
}

// ── SWC: non-blocking press state machine — call every loop ───────
void swcService() {
#if SWC_ENABLE
  uint32_t now = millis();

  // Currently asserting a leg — hold until its deadline, then release.
  if (swcActive != SWC_NONE) {
    if ((int32_t)(now - swcNextAt) >= 0) {
      pinMode(SWC_MAP[swcActive].pin, INPUT);   // high-Z = released
      swcActive = SWC_NONE;
      swcNextAt = now + SWC_GAP_MS;             // enforce inter-press gap
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
#endif
}

// ── LED helper — non-blocking ─────────────────────────────────────
void flashLED(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
  ledOffAt = millis() + PULSE_MS;
  if (!ledOffAt) ledOffAt = 1;   // never land on the "off" sentinel
}

void ledService() {
  if (ledOffAt && (int32_t)(millis() - ledOffAt) >= 0) {
    led.setPixelColor(0, led.Color(0, 0, 0));
    led.show();
    ledOffAt = 0;
  }
}

// Blocking blink — setup() only, where stalling is harmless.
void blinkBlocking(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
  delay(PULSE_MS);
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
// This is your integration point. `count` is the number of repeats —
// it is >1 only for knob rotation, where one CAN frame can carry
// several detents. Everything else passes 1.
void onButtonPress(const char* name, uint8_t r, uint8_t g, uint8_t b,
                   uint8_t count = 1) {
  if (count > 1) Serial.printf("PRESS: %s x%u\n", name, count);
  else           Serial.printf("PRESS: %s\n", name);
  flashLED(r, g, b);

  // ── Head unit control ────────────────────────────────────────
  if      (!strcmp(name, "KNOB_CW"))    swcPush(SWC_VOL_UP,   count);
  else if (!strcmp(name, "KNOB_CCW"))   swcPush(SWC_VOL_DOWN, count);
  else if (!strcmp(name, "KNOB_PRESS")) swcPush(SWC_MUTE,     1);
  else if (!strcmp(name, "RIGHT"))      swcPush(SWC_NEXT,     1);
  else if (!strcmp(name, "LEFT"))       swcPush(SWC_PREV,     1);

  // Unmapped so far: UP, DOWN, MENU, BACK, OPTION, COM, MEDIA, NAV, MAP
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

  // ── Knob rotation ─────────────────────────────────────────────
  // int8_t signed cast handles byte wraparound (e.g. 0xFF→0x01 = +2 not -254).
  // The magnitude is the detent count for this frame — emit that many
  // volume steps, not one. Promote to int16_t before negating so that
  // delta == -128 cannot overflow, then clamp against glitch frames.
  if (b2 == 0x80 && lastB1 != 0xFF) {
    int8_t delta = (int8_t)(b1 - lastB1);
    if (delta != 0) {
      int16_t mag = delta > 0 ? (int16_t)delta : -(int16_t)delta;
      if (mag > SWC_MAX_STEP) mag = SWC_MAX_STEP;
      if (delta > 0) onButtonPress("KNOB_CW",  0,   255, 0, (uint8_t)mag);
      else           onButtonPress("KNOB_CCW", 255, 0,   0, (uint8_t)mag);
    }
  }

  // ── Store previous state ──────────────────────────────────────
  if (b3 & 0x01) lastB1 = 0xFF; // reset position ref while pressed
  else            lastB1 = b1;
  lastB3 = b3;
  lastB4 = b4;
  lastB5 = b5;
  lastB6 = b6;
  lastB7 = b7;
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  swcInit();   // all ladder legs high-Z before anything else

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

  // Discard anything the wake loop queued — no phantom volume on boot.
  swcInit();

  blinkBlocking(255, 255, 255); // white = ready
  Serial.println("iDrive ready — all 14 inputs mapped");
#if SWC_ENABLE
  Serial.println("SWC ladder ACTIVE:");
  for (uint8_t i = 0; i < SWC_COUNT; i++)
    Serial.printf("  %-9s GPIO%-2u  %u ohm\n",
                  SWC_MAP[i].label, SWC_MAP[i].pin, SWC_MAP[i].ohms);
#else
  Serial.println("SWC output DISABLED (SWC_ENABLE=0) — decode only");
#endif
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

  swcService();   // advance the ladder press state machine
  ledService();   // turn the NeoPixel back off when its time is up
}
