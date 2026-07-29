// Host-side test of the SWC ladder state machine copied verbatim from
// idrive_controller.ino (SWC backend). Build: c++ -std=c++17 -o swc_ladder_test swc_ladder_test.cpp && ./swc_ladder_test. Mocks millis()/pinMode/digitalWrite so we
// can prove the queue + timing behaviour without hardware.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>

// ── Arduino mocks ────────────────────────────────────────────────
static uint32_t g_now = 0;
static uint32_t millis() { return g_now; }
#define INPUT 0
#define OUTPUT 1
#define LOW 0

static int  g_pinMode[64];
static bool g_drivenLow[64];
static void pinMode(uint8_t p, int m) {
  g_pinMode[p] = m;
  if (m == INPUT) g_drivenLow[p] = false;
}
static void digitalWrite(uint8_t p, int v) { if (v == LOW) g_drivenLow[p] = true; }

// ── Verbatim from the sketch ─────────────────────────────────────
#define SWC_ENABLE    1
#define SWC_HOLD_MS   140
#define SWC_GAP_MS    60
#define SWC_QUEUE_LEN 24
#define SWC_MAX_STEP  12

enum : uint8_t { KEY_VOL_UP = 0, KEY_VOL_DOWN, KEY_MUTE, KEY_NEXT, KEY_PREV, KEY_COUNT };
#define SWC_NONE 0xFF

struct SwcLeg { uint8_t pin; uint16_t ohms; };
static const char* KEY_NAME[] = {"VOL_UP","VOL_DOWN","MUTE","NEXT","PREV"};
static const SwcLeg SWC_MAP[KEY_COUNT] = {
  { 5, 560 },
  { 6, 1000 },
  { 7, 1500 },
  { 15, 2200 },
  { 16, 3900 },
};

static uint8_t  swcQueue[SWC_QUEUE_LEN];
static uint8_t  swcHead   = 0;
static uint8_t  swcTail   = 0;
static uint8_t  swcActive = SWC_NONE;
static uint32_t swcNextAt = 0;

void swcInit() {
  for (uint8_t i = 0; i < KEY_COUNT; i++) pinMode(SWC_MAP[i].pin, INPUT);
  swcActive = SWC_NONE;
  swcHead = swcTail = 0;
  swcNextAt = 0;
}

void swcPush(uint8_t key, uint8_t times) {
  if (key >= KEY_COUNT) return;
  while (times--) {
    uint8_t next = (uint8_t)((swcHead + 1) % SWC_QUEUE_LEN);
    if (next == swcTail) return;
    swcQueue[swcHead] = key;
    swcHead = next;
  }
}

void swcService() {
  uint32_t now = millis();
  if (swcActive != SWC_NONE) {
    if ((int32_t)(now - swcNextAt) >= 0) {
      pinMode(SWC_MAP[swcActive].pin, INPUT);
      swcActive = SWC_NONE;
      swcNextAt = now + SWC_GAP_MS;
    }
    return;
  }
  if ((int32_t)(now - swcNextAt) < 0) return;
  if (swcHead == swcTail) return;
  uint8_t key = swcQueue[swcTail];
  swcTail = (uint8_t)((swcTail + 1) % SWC_QUEUE_LEN);
  pinMode(SWC_MAP[key].pin, OUTPUT);
  digitalWrite(SWC_MAP[key].pin, LOW);
  swcActive = key;
  swcNextAt = now + SWC_HOLD_MS;
}

// ── Test harness ─────────────────────────────────────────────────
static int failures = 0;
static void check(bool ok, const char* what) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) failures++;
}

// Run the machine for `ms`, recording each completed press and asserting
// that never more than one leg is driven low simultaneously.
static std::vector<std::string> run(uint32_t ms, bool* multiAssertSeen) {
  std::vector<std::string> presses;
  uint8_t prevActive = swcActive;
  for (uint32_t i = 0; i < ms; i++) {
    g_now++;
    swcService();
    int lowCount = 0;
    for (int p = 0; p < 64; p++) if (g_drivenLow[p]) lowCount++;
    if (lowCount > 1) *multiAssertSeen = true;
    if (swcActive != prevActive && swcActive != SWC_NONE)
      presses.push_back(KEY_NAME[swcActive]);
    prevActive = swcActive;
  }
  return presses;
}

int main() {
  bool multi = false;

  printf("Test 1: five-detent spin emits five VOL_UP presses\n");
  swcInit();
  swcPush(KEY_VOL_UP, 5);
  auto p1 = run(2000, &multi);
  check(p1.size() == 5, "exactly 5 presses emitted");
  bool allVolUp = true;
  for (auto& s : p1) if (s != "VOL_UP") allVolUp = false;
  check(allVolUp, "all presses are VOL_UP");
  check(!multi, "never more than one leg asserted at once");
  check(swcActive == SWC_NONE, "no leg left asserted at end");
  bool allReleased = true;
  for (uint8_t i = 0; i < KEY_COUNT; i++)
    if (g_drivenLow[SWC_MAP[i].pin]) allReleased = false;
  check(allReleased, "every leg returned to high-Z");

  printf("\nTest 2: press cadence is HOLD+GAP = 200ms\n");
  swcInit();
  swcPush(KEY_VOL_DOWN, 3);
  std::vector<uint32_t> startTimes;
  uint8_t prev = SWC_NONE;
  for (uint32_t i = 0; i < 1000; i++) {
    g_now++; swcService();
    if (swcActive != prev && swcActive != SWC_NONE) startTimes.push_back(g_now);
    prev = swcActive;
  }
  check(startTimes.size() == 3, "3 presses emitted");
  bool cadenceOk = true;
  for (size_t i = 1; i < startTimes.size(); i++) {
    uint32_t d = startTimes[i] - startTimes[i-1];
    printf("        gap between press %zu and %zu = %ums\n", i, i+1, d);
    if (d != SWC_HOLD_MS + SWC_GAP_MS) cadenceOk = false;
  }
  check(cadenceOk, "cadence is exactly HOLD_MS+GAP_MS (200ms)");

  printf("\nTest 3: mixed keys preserve FIFO order\n");
  swcInit();
  swcPush(KEY_MUTE, 1); swcPush(KEY_NEXT, 1); swcPush(KEY_PREV, 1);
  auto p3 = run(1000, &multi);
  bool order = p3.size() == 3 && p3[0] == "MUTE" && p3[1] == "NEXT" && p3[2] == "PREV";
  check(order, "MUTE, NEXT, PREV emitted in order");

  printf("\nTest 4: queue overflow drops cleanly, never corrupts\n");
  swcInit();
  swcPush(KEY_VOL_UP, 200);          // far beyond QUEUE_LEN
  int queued = (swcHead - swcTail + SWC_QUEUE_LEN) % SWC_QUEUE_LEN;
  printf("        queued after pushing 200: %d (cap %d)\n", queued, SWC_QUEUE_LEN - 1);
  check(queued == SWC_QUEUE_LEN - 1, "queue capped at LEN-1, no wrap corruption");
  auto p4 = run(8000, &multi);
  check(p4.size() == (size_t)(SWC_QUEUE_LEN - 1), "drains exactly what was queued");
  check(!multi, "still never double-asserts under overflow");
  check(swcActive == SWC_NONE, "drains fully to idle");

  printf("\nTest 5: millis() rollover near 2^32 is handled\n");
  swcInit();
  g_now = 0xFFFFFF00;                // ~256ms before wrap
  swcPush(KEY_VOL_UP, 4);
  auto p5 = run(2000, &multi);
  printf("        presses across the wrap: %zu\n", p5.size());
  check(p5.size() == 4, "all 4 presses survive the 32-bit wrap");
  check(swcActive == SWC_NONE, "idle after wrap");

  printf("\nTest 6: swcInit() clears a queue mid-flight (boot discard)\n");
  swcInit();
  g_now = 1000;
  swcPush(KEY_VOL_UP, 10);
  g_now += 50; swcService();          // assert one leg
  check(swcActive != SWC_NONE, "a leg is asserted before init");
  swcInit();
  check(swcActive == SWC_NONE, "swcInit clears active leg");
  bool cleared = true;
  for (uint8_t i = 0; i < KEY_COUNT; i++)
    if (g_drivenLow[SWC_MAP[i].pin]) cleared = false;
  check(cleared, "swcInit returns every leg to high-Z");
  auto p6 = run(3000, &multi);
  check(p6.empty(), "no queued presses survive swcInit");

  printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL TESTS PASSED",
         failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
