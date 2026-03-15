// ═══════════════════════════════════════════════════════════════════
// BMW F-Series iDrive — Universal CAN Input Controller
// Hardware: Lonely Binary ESP32-S3 + SN65HVD230 CAN transceiver
// Version:  1.0.0
// Author:   Mark Pelosi
// License:  MIT
// Repo:     https://github.com/YOUR_USERNAME/idrive-controller
// ═══════════════════════════════════════════════════════════════════
//
// DESCRIPTION:
//   Decodes all inputs from a BMW F-Series iDrive controller (Preh,
//   part #6582 6829079-03) over CAN bus and exposes them as digital
//   events for any downstream use — GPIO, SWC voltage output, BLE
//   HID, MQTT, or any other output you want to wire in.
//
// WIRING:
//   iDrive Green        → SN65HVD230 CANH
//   iDrive Green/Orange → SN65HVD230 CANL
//   120Ω resistor       → CANH to CANL  ← mandatory, crash without it
//   SN65HVD230 VCC      → ESP32 3.3V
//   SN65HVD230 GND      → Common GND
//   SN65HVD230 CRX      → ESP32 GPIO8
//   SN65HVD230 CTX      → ESP32 GPIO4
//   12V bench/car GND   → Common GND    ← floating ground = no data
//   SWC output (future) → GPIO2 → [10kΩ] → SWC wire → [10µF] → GND
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
// NEXT STEPS:
//   - Capture 0x273 reply on live BMW to complete auto-wake
//   - Add SWC output in onButtonPress() for head unit control
//   - Wire RC filter: GPIO2 → [10kΩ] → SWC wire → [10µF] → GND
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

// ── Globals ──────────────────────────────────────────────────────
Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastKeepalive = 0;
unsigned long lastSlowKA    = 0;
bool          initialized   = false;

// Previous frame state for edge detection
uint8_t lastB1 = 0xFF;
uint8_t lastB3 = 0x00;
uint8_t lastB4 = 0x00;
uint8_t lastB5 = 0x00;
uint8_t lastB6 = 0xC0;
uint8_t lastB7 = 0xF8;

// ── LED helper ────────────────────────────────────────────────────
void flashLED(uint8_t r, uint8_t g, uint8_t b) {
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
// This is your integration point — add your own output logic here.
// Examples:
//   digitalWrite(somePin, HIGH);   // GPIO output
//   outputSWC(duty, holdMs);       // SWC voltage for head unit
//   bleKeyboard.press(KEY_MEDIA_NEXT_TRACK);  // BLE HID
void onButtonPress(const char* name, uint8_t r, uint8_t g, uint8_t b) {
  Serial.printf("PRESS: %s\n", name);
  flashLED(r, g, b);

  // ── ADD YOUR OUTPUT LOGIC BELOW ──────────────────────────────
  // if (strcmp(name, "KNOB_CW") == 0)  { /* volume up */ }
  // if (strcmp(name, "KNOB_CCW") == 0) { /* volume down */ }
  // if (strcmp(name, "MENU") == 0)     { /* menu */ }
  // ... etc
}

// ── Fast keepalive — send every KEEPALIVE_MS ─────────────────────
void sendKeepalive() {
  // CAS ignition frame — signals key-on to iDrive
  uint8_t cas[8]   = {0x45,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  sendFrame(0x0AA, 8, cas); delay(1);

  // KOMBI network alive — signals instrument cluster present
  uint8_t kombi[8] = {0x45,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  sendFrame(0x130, 8, kombi); delay(1);

  // BMW NM alive — byte1=0x02=ALIVE (0xFF=sleep, never send)
  uint8_t nm[8]    = {0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00};
  sendFrame(0x440, 8, nm); delay(1);

  // Secondary heartbeat
  uint8_t hb[8]    = {0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00};
  sendFrame(0x560, 8, hb); delay(1);

  // Illumination — adjust ILLUM_LEVEL to change brightness
  uint8_t illum[8] = {ILLUM_LEVEL,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  sendFrame(0x202, 2, illum); delay(1);
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
    flashLED(0, 255, 0); delay(100);
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
  // int8_t signed cast handles byte wraparound (e.g. 0xFF→0x01 = +2 not -254)
  if (b2 == 0x80 && lastB1 != 0xFF) {
    int8_t delta = (int8_t)(b1 - lastB1);
    if (delta > 0) {
      onButtonPress("KNOB_CW",  0,   255, 0  ); // green
      Serial.printf("KNOB_CW  %d click(s)\n", delta);
    } else if (delta < 0) {
      onButtonPress("KNOB_CCW", 255, 0,   0  ); // red
      Serial.printf("KNOB_CCW %d click(s)\n", -delta);
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

  led.begin();
  led.setBrightness(40);
  led.setPixelColor(0, led.Color(0, 0, 0));
  led.show();

  // Init TWAI (ESP32 built-in CAN controller)
  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CTX_PIN,
                                (gpio_num_t)CRX_PIN,
                                TWAI_MODE_NORMAL);
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

  flashLED(255, 255, 255); // white = ready
  Serial.println("iDrive ready — all 14 inputs mapped");
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
}
