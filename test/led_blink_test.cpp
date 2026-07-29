// Host-side test of the NeoPixel blink state machine from
// idrive_controller.ino. Mocks millis() and the NeoPixel so we can prove
// that consecutive presses produce SEPARATE visible blinks — the exact
// regression seen on the bench, where two same-colour flashes merged into
// one continuous glow because nothing forced a dark period between them.
//
// Build: c++ -std=c++17 -o led_blink_test led_blink_test.cpp && ./led_blink_test
#include <cstdio>
#include <cstdint>
#include <vector>

static uint32_t g_now = 0;
static uint32_t millis() { return g_now; }

// ── NeoPixel mock: record every colour change with its timestamp ──
struct Sample { uint32_t t; uint8_t r, g, b; };
static std::vector<Sample> shown;
static uint8_t curR = 0, curG = 0, curB = 0;
struct Pixel {
  uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  }
  void setPixelColor(int, uint32_t c) {
    curR = (c >> 16) & 0xFF; curG = (c >> 8) & 0xFF; curB = c & 0xFF;
  }
  void show() { shown.push_back({ g_now, curR, curG, curB }); }
} led;

// ── Verbatim from the sketch ─────────────────────────────────────
#define PULSE_MS      70
#define LED_GAP_MS    45
#define LED_QUEUE_LEN  8

struct LedReq { uint8_t r, g, b; };
static LedReq   ledQueue[LED_QUEUE_LEN];
static uint8_t  ledHead = 0, ledTail = 0;
static uint8_t  ledState = 0;
static uint32_t ledUntil = 0;

void flashLED(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t next = (uint8_t)((ledHead + 1) % LED_QUEUE_LEN);
  if (next == ledTail) return;
  ledQueue[ledHead] = { r, g, b };
  ledHead = next;
}

void ledService() {
  uint32_t now = millis();
  if (ledState == 1) {
    if ((int32_t)(now - ledUntil) < 0) return;
    led.setPixelColor(0, led.Color(0, 0, 0));
    led.show();
    ledState = 2;
    ledUntil = now + LED_GAP_MS;
    return;
  }
  if (ledState == 2) {
    if ((int32_t)(now - ledUntil) < 0) return;
    ledState = 0;
    // deliberate fall-through
  }
  if (ledHead == ledTail) return;
  const LedReq& q = ledQueue[ledTail];
  ledTail = (uint8_t)((ledTail + 1) % LED_QUEUE_LEN);
  led.setPixelColor(0, led.Color(q.r, q.g, q.b));
  led.show();
  ledState = 1;
  ledUntil = now + PULSE_MS;
}

// ── Harness ──────────────────────────────────────────────────────
static int failures = 0;
static void check(bool ok, const char* what) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) failures++;
}
static void reset() {
  shown.clear(); ledHead = ledTail = 0; ledState = 0; ledUntil = 0;
  curR = curG = curB = 0; g_now = 0;
}
static void run(uint32_t ms) { for (uint32_t i = 0; i < ms; i++) { g_now++; ledService(); } }

// Count lit periods: transitions from dark to any non-black colour.
static int countBlinks() {
  int n = 0; bool lit = false;
  for (auto& s : shown) {
    bool nowLit = s.r || s.g || s.b;
    if (nowLit && !lit) n++;
    lit = nowLit;
  }
  return n;
}

int main() {
  printf("Test 1: three presses of the SAME colour = three distinct blinks\n");
  reset();
  flashLED(0,255,0); flashLED(0,255,0); flashLED(0,255,0);   // 3x green
  run(1000);
  printf("        blinks observed: %d\n", countBlinks());
  check(countBlinks() == 3, "same-colour presses do not merge into one glow");

  printf("\nTest 2: every blink is followed by a real dark period\n");
  reset();
  flashLED(255,0,0); flashLED(255,0,0);
  run(1000);
  bool gapOk = true; uint32_t litAt = 0; bool lit = false;
  for (auto& s : shown) {
    bool nowLit = s.r || s.g || s.b;
    if (nowLit && !lit) litAt = s.t;
    if (!nowLit && lit && (s.t - litAt) != PULSE_MS) gapOk = false;
    lit = nowLit;
  }
  check(gapOk, "each lit period lasts exactly PULSE_MS then goes dark");
  bool sawDark = false;
  for (auto& s : shown) if (!(s.r || s.g || s.b)) sawDark = true;
  check(sawDark, "LED actually reaches black between blinks");

  printf("\nTest 3: blink cadence is PULSE+GAP = 115ms\n");
  reset();
  flashLED(0,0,255); flashLED(0,0,255); flashLED(0,0,255);
  run(1000);
  std::vector<uint32_t> starts; lit = false;
  for (auto& s : shown) {
    bool nowLit = s.r || s.g || s.b;
    if (nowLit && !lit) starts.push_back(s.t);
    lit = nowLit;
  }
  bool cadence = true;
  for (size_t i = 1; i < starts.size(); i++) {
    printf("        blink %zu->%zu spacing = %ums\n", i, i+1, starts[i]-starts[i-1]);
    if (starts[i]-starts[i-1] != PULSE_MS + LED_GAP_MS) cadence = false;
  }
  check(cadence, "spacing is exactly PULSE_MS + LED_GAP_MS");

  printf("\nTest 4: distinct colours are preserved in order\n");
  reset();
  flashLED(255,0,0); flashLED(0,255,0); flashLED(0,0,255);
  run(1000);
  std::vector<int> seq;
  lit = false;
  for (auto& s : shown) {
    bool nowLit = s.r || s.g || s.b;
    if (nowLit && !lit) seq.push_back(s.r ? 0 : (s.g ? 1 : 2));
    lit = nowLit;
  }
  check(seq.size() == 3 && seq[0]==0 && seq[1]==1 && seq[2]==2,
        "red, green, blue emitted in that order");

  printf("\nTest 5: queue is bounded — a long spin cannot lag forever\n");
  reset();
  for (int i = 0; i < 200; i++) flashLED(0,255,0);
  int queued = (ledHead - ledTail + LED_QUEUE_LEN) % LED_QUEUE_LEN;
  printf("        queued after 200 presses: %d (cap %d)\n", queued, LED_QUEUE_LEN-1);
  check(queued == LED_QUEUE_LEN - 1, "queue capped, extras dropped not wrapped");
  run(3000);
  check(countBlinks() == LED_QUEUE_LEN - 1, "drains exactly what was queued");
  check(ledState == 0, "returns to idle");

  printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL TESTS PASSED",
         failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
