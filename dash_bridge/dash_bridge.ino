// ═══════════════════════════════════════════════════════════════════
// dash_bridge — USB HID keyboard bridge:  HVAC Pi ──> TSDash Pi
// ═══════════════════════════════════════════════════════════════════
// Board: Lonely Binary ESP32-S3 Dev Module (N16R8), the spare.
//
// WHY THIS BOARD EXISTS
// The TSDash Pi has no SSH, no network in the car, and no shared bus
// with the HVAC Pi. But it does have USB host ports, and TS Dash has
// documented keyboard shortcuts:
//
//     Ctrl + Right   move to dash on right
//     Ctrl + Left    move to dash on left
//     Ctrl + Up      configuration
//     Ctrl + Down    back to main dash
//
// (Confirmed working on the TSDash Pi with a real keyboard, 2026-07-30.)
//
// So this board becomes a second keyboard. The TSDash Pi needs NOTHING
// installed on it — no listener, no daemon, no config. It already knows
// how to handle a keyboard, which is the entire point of this approach.
//
// TOPOLOGY — two USB-C ports on this board, two independent buses:
//
//   HVAC Pi USB hub ──> [UART port]  dash_bridge  [native USB] ──> TSDash Pi
//                        (bridge chip)              (GPIO19/20)
//                        commands in                HID out
//                        flashing + console
//
// Neither Pi is ever a USB device. This board is a device on BOTH buses
// at once, which is what makes the whole thing work — a hub cannot join
// two hosts, and a USB-A to USB-A cable between them is a way to let the
// smoke out.
//
// ⚠ POWER: both USB-C ports feed this board's 5V rail, and on most dual-
// port S3 boards those VBUS lines are commoned. Plugging both in ties the
// two Pis' 5V rails together. Use a DATA-ONLY cable (VBUS cut) to the
// TSDash Pi and let the HVAC Pi hub power this board. HID needs only
// D+/D- and ground, so it enumerates fine self-powered. Meter continuity
// between the two ports' VBUS pins first — if your board already isolates
// them with diodes, a normal cable is fine.
//
// BUILD — USB Mode MUST be TinyUSB, not Hardware CDC. HID does not exist
// under hwcdc, which is what idrive_controller is pinned to. The #error
// below makes a wrong flag fail loudly instead of producing a board that
// enumerates and never types.
//
//   export ARDUINO_DIRECTORIES_DATA=~/.arduino-cli-esp32v3/data
//   export ARDUINO_DIRECTORIES_USER=~/.arduino-cli-esp32v3/user
//   export ARDUINO_DIRECTORIES_DOWNLOADS=~/.arduino-cli-esp32v3/downloads
//   arduino-cli compile --warnings all --fqbn
//     esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=default,FlashSize=16M,PSRAM=opi
//     dash_bridge
//   (one line; the FQBN is split here only to fit the margin)
//
// Arduino IDE: ESP32S3 Dev Module · USB Mode "USB-OTG (TinyUSB)" ·
// USB CDC On Boot **Disabled** · Flash 16MB · PSRAM OPI.
//
// Deliberately NO USB CDC on the native port. That port faces the TSDash
// Pi, which will never drain a CDC endpoint — exactly the condition that
// stalls writes ~2s each and made the iDrive board look like flaky
// hardware. No CDC endpoint, no stall, nothing to remember. Console and
// flashing both live on the UART port instead, where the HVAC Pi is.
//
// PROTOCOL — text in, JSON back, same shape as the lighting board.
//
//   D NEXT        Ctrl+Right   dash to the right
//   D PREV        Ctrl+Left    dash to the left
//   D CFG         Ctrl+Up      configuration screen
//   D HOME        Ctrl+Down    back to main dash
//   D PING        no keystroke — liveness check
//   D GET         no keystroke — status report
//
//   {"src":"dash","ok":1,"cmd":"NEXT","q":1,"sent":12,"usb":1}
//
// One keystroke per command, never a burst. The knob clamps rotation to
// 1..12 detents per frame and a flick that fires eight Ctrl+Rights leaves
// the driver somewhere unpredictable with no way to see where they landed
// — this board cannot read back which dash is showing, the same blind-
// controller problem the lighting board solves by owning its own state.
// ═══════════════════════════════════════════════════════════════════

#if ARDUINO_USB_MODE
#error "Wrong USB mode. Build with USBMode=default (USB-OTG / TinyUSB). \
HID is unavailable under USBMode=hwcdc (Hardware CDC and JTAG)."
#endif

#include "USB.h"
#include "USBHIDKeyboard.h"
#include "esp_mac.h"

#define FW_VERSION "1.1.0"

// ── Tunables ───────────────────────────────────────────────────────
#define LINK_BAUD   115200
#define HOLD_MS     25    // modifier+key held down; Java needs a real press
#define GAP_MS      40    // minimum quiet time between keystrokes
#define QUEUE_LEN   8     // bounded — a flick drops the excess, never queues seconds of input
#define USE_LED     1     // NeoPixel on GPIO48 (same board as idrive_controller)
#define LED_PIN     48
#define HEARTBEAT_MS 3000

// ── Which Serial is the Pi? ────────────────────────────────────────
// With CDC on boot enabled, `Serial` is the native-USB CDC and UART0 is
// `Serial0`. With it disabled — how this is meant to be built — `Serial`
// IS UART0. Alias it so the sketch is correct either way and does not
// silently start talking to the TSDash Pi instead of the HVAC Pi.
#if ARDUINO_USB_CDC_ON_BOOT
  #define LINK Serial0
#else
  #define LINK Serial
#endif

#if USE_LED
#include <Adafruit_NeoPixel.h>
static Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);
#endif

USBHIDKeyboard Keyboard;

// ── Keystroke queue ────────────────────────────────────────────────
struct Stroke { uint8_t mod; uint8_t key; };

static Stroke   queue[QUEUE_LEN];
static uint8_t  qHead = 0, qTail = 0, qCount = 0;

enum SendState { S_IDLE, S_HELD, S_GAP };
static SendState sendState = S_IDLE;
static unsigned long stateAt = 0;
static Stroke   inFlight = { 0, 0 };

static volatile bool usbUp    = false;   // host enumerated, not suspended
static uint32_t      sentCount = 0;      // keystrokes actually pushed to HID
static uint32_t      dropCount = 0;      // queue-full drops

// ── Who am I ───────────────────────────────────────────────────────
// Every other board on the Pi's hub is pinned by udev on its MAC, which
// the ESP32-S3 publishes as the USB serial string on its NATIVE port.
// This board is reached over its UART port instead, where the host talks
// to a CH340 — a separate chip with its own fixed descriptor and no idea
// what our MAC is. udev therefore cannot pin us by MAC, and this CH340
// reports no serial at all, so /dev/tsdash has to be pinned to a physical
// hub port and follows the socket rather than the board.
//
// So we say it out loud instead. The MAC goes in the boot banner and in
// every status line, which lets the Pi confirm it is talking to the board
// it thinks it is — a moved cable becomes a log line instead of silent
// keystrokes going to the wrong place.
static char macStr[18] = "??:??:??:??:??:??";

static void readMac() {
  uint8_t m[6] = {0};
  if (esp_read_mac(m, ESP_MAC_WIFI_STA) == ESP_OK)
    snprintf(macStr, sizeof macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
             m[0], m[1], m[2], m[3], m[4], m[5]);
}

// ── LED ────────────────────────────────────────────────────────────
// Colours match the car: teal for forward, amber for back, ice blue for
// the config screens, red for a command this board did not understand.
static unsigned long ledOffAt = 0;
static unsigned long lastBeat = 0;

static void ledSet(uint8_t r, uint8_t g, uint8_t b, uint16_t ms) {
#if USE_LED
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
  ledOffAt = ms ? millis() + ms : 0;
#endif
}

static void ledService() {
#if USE_LED
  unsigned long now = millis();
  if (ledOffAt && (int32_t)(now - ledOffAt) >= 0) {
    led.setPixelColor(0, 0);
    led.show();
    ledOffAt = 0;
  }
  // Dim heartbeat so a headless board on a hub still proves it is alive.
  if (!ledOffAt && (now - lastBeat) >= HEARTBEAT_MS) {
    lastBeat = now;
    ledSet(0, usbUp ? 6 : 0, usbUp ? 6 : 0, 60);
    if (!usbUp) ledSet(6, 2, 0, 60);   // amber tick = no USB host yet
  }
#endif
}

// ── Queue helpers ──────────────────────────────────────────────────
static bool enqueue(uint8_t mod, uint8_t key) {
  if (qCount >= QUEUE_LEN) { dropCount++; return false; }
  queue[qTail] = { mod, key };
  qTail = (qTail + 1) % QUEUE_LEN;
  qCount++;
  return true;
}

// Non-blocking press/release. No delay() anywhere in the send path —
// same rule as idrive_controller, kept here so the two boards behave
// the same way when someone comes back to this in a year.
static void sendService() {
  unsigned long now = millis();

  switch (sendState) {
    case S_IDLE:
      if (qCount == 0) return;
      inFlight = queue[qHead];
      qHead = (qHead + 1) % QUEUE_LEN;
      qCount--;
      Keyboard.press(inFlight.mod);
      Keyboard.press(inFlight.key);
      sentCount++;
      stateAt = now;
      sendState = S_HELD;
      break;

    case S_HELD:
      if ((now - stateAt) < HOLD_MS) return;
      Keyboard.releaseAll();
      stateAt = now;
      sendState = S_GAP;
      break;

    case S_GAP:
      if ((now - stateAt) < GAP_MS) return;
      sendState = S_IDLE;
      break;
  }
}

// ── Status ─────────────────────────────────────────────────────────
static void report(const char* cmd, bool ok) {
  LINK.printf("{\"src\":\"dash\",\"ok\":%d,\"cmd\":\"%s\",\"q\":%u,"
              "\"sent\":%lu,\"drop\":%lu,\"usb\":%d,\"mac\":\"%s\"}\n",
              ok ? 1 : 0, cmd, (unsigned)qCount,
              (unsigned long)sentCount, (unsigned long)dropCount,
              usbUp ? 1 : 0, macStr);
}

// ── Command handling ───────────────────────────────────────────────
// Accepts "D NEXT" and bare "NEXT" alike. The HVAC Pi and this board are
// separate deployments that can be updated hours apart, so the link must
// not break just because one side is newer than the other — same reason
// apply_idrive_event() still accepts the pre-rename mode names.
static void handleLine(char* line) {
  // Trim leading space, strip CR, uppercase in place.
  char* p = line;
  while (*p == ' ' || *p == '\t') p++;
  for (char* q = p; *q; q++) {
    if (*q == '\r' || *q == '\n') { *q = 0; break; }
    *q = toupper((unsigned char)*q);
  }
  if (!*p) return;

  // Optional "D " prefix.
  if (p[0] == 'D' && (p[1] == ' ' || p[1] == '\t')) {
    p += 2;
    while (*p == ' ' || *p == '\t') p++;
  }
  if (!*p) return;

  if (!strcmp(p, "NEXT")) {
    enqueue(KEY_LEFT_CTRL, KEY_RIGHT_ARROW);
    ledSet(0, 40, 36, 90);            // teal
    report("NEXT", true);
  } else if (!strcmp(p, "PREV")) {
    enqueue(KEY_LEFT_CTRL, KEY_LEFT_ARROW);
    ledSet(48, 33, 0, 90);            // amber
    report("PREV", true);
  } else if (!strcmp(p, "CFG")) {
    enqueue(KEY_LEFT_CTRL, KEY_UP_ARROW);
    ledSet(12, 36, 60, 90);           // ice blue
    report("CFG", true);
  } else if (!strcmp(p, "HOME")) {
    enqueue(KEY_LEFT_CTRL, KEY_DOWN_ARROW);
    ledSet(12, 36, 60, 90);           // ice blue
    report("HOME", true);
  } else if (!strcmp(p, "PING") || !strcmp(p, "GET")) {
    report(p, true);                  // no keystroke — liveness only
  } else {
    ledSet(50, 0, 0, 120);            // red — unknown command
    report("?", false);
  }
}

// ── USB host state ─────────────────────────────────────────────────
// Informational only. Sends are attempted regardless of what this says:
// if the event plumbing is ever wrong, a working bridge that reports the
// wrong flag beats a silent bridge that refuses to type.
static void onUsbEvent(void* arg, esp_event_base_t base,
                       int32_t id, void* data) {
  if (base != ARDUINO_USB_EVENTS) return;
  switch (id) {
    case ARDUINO_USB_STARTED_EVENT:
    case ARDUINO_USB_RESUME_EVENT:  usbUp = true;  break;
    case ARDUINO_USB_STOPPED_EVENT:
    case ARDUINO_USB_SUSPEND_EVENT: usbUp = false; break;
    default: break;
  }
}

// ═══════════════════════════════════════════════════════════════════
void setup() {
  LINK.begin(LINK_BAUD);

#if USE_LED
  led.begin();
  led.setBrightness(255);
  led.clear();
  led.show();
#endif

  USB.onEvent(onUsbEvent);
  USB.productName("944S Dash Bridge");
  USB.manufacturerName("944S");
  Keyboard.begin();
  USB.begin();

  readMac();
  LINK.printf("\ndash_bridge %s — HVAC Pi -> TSDash Pi HID bridge\n", FW_VERSION);
  LINK.printf("mac %s (not in the CH340's USB descriptor — see readMac)\n", macStr);
  LINK.println("commands: D NEXT | D PREV | D CFG | D HOME | D PING | D GET");
  ledSet(0, 40, 36, 400);
}

void loop() {
  static char buf[48];
  static uint8_t n = 0;

  while (LINK.available()) {
    char c = (char)LINK.read();
    if (c == '\n' || c == '\r') {
      if (n) { buf[n] = 0; handleLine(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    } else {
      n = 0;                          // overlong junk — drop the line, not the link
    }
  }

  sendService();
  ledService();
}
