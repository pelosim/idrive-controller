// Host-side test of the mode layer from idrive_controller.ino. The mode
// rules are where a foot-gun would hide — a knob silently wired to the
// cabin temperature is much worse than one that does nothing — so the
// safety properties are asserted explicitly here.
//
// Build: c++ -std=c++17 -o mode_layer_test mode_layer_test.cpp && ./mode_layer_test
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

static uint32_t g_now = 0;
static uint32_t millis() { return g_now; }

// ── Capture what the sketch would have done ──────────────────────
struct IrCall   { uint8_t key; uint8_t count; };
struct LinkCall { std::string mode, action; uint8_t count; };
static std::vector<IrCall>   irCalls;
static std::vector<LinkCall> linkCalls;
static std::vector<std::string> ledFlashes;   // "R,G,B"

// ── Verbatim from the sketch ─────────────────────────────────────
#define MODE_TIMEOUT_MS 10000
enum : uint8_t { KEY_VOL_UP = 0, KEY_VOL_DOWN, KEY_MUTE, KEY_NEXT, KEY_PREV, KEY_COUNT };
enum : uint8_t { MODE_MEDIA = 0, MODE_HVAC, MODE_LIGHT, MODE_GAUGE, MODE_COUNT };
static const char* MODE_NAME[MODE_COUNT] = { "MEDIA", "HVAC", "LIGHT", "GAUGE" };
static const uint8_t MODE_RGB[MODE_COUNT][3] = {
  { 0x2C, 0xE8, 0xD8 }, { 0xFF, 0xB0, 0x00 },
  { 0x9C, 0x40, 0xFF }, { 0x5C, 0xB8, 0xFF },
};
enum : uint8_t {
  ACT_NONE = 0,
  ACT_VOL_UP, ACT_VOL_DOWN, ACT_MUTE, ACT_NEXT, ACT_PREV,
  ACT_TEMP_UP, ACT_TEMP_DOWN, ACT_FAN_UP, ACT_FAN_DOWN,
  ACT_HVAC_MODE_PREV, ACT_HVAC_MODE_NEXT, ACT_HVAC_TOGGLE,
  ACT_LIGHT_BRIGHTER, ACT_LIGHT_DIMMER,
  ACT_LIGHT_SCENE_PREV, ACT_LIGHT_SCENE_NEXT, ACT_LIGHT_TOGGLE,
  ACT_GAUGE_SCROLL_UP, ACT_GAUGE_SCROLL_DOWN,
  ACT_GAUGE_PAGE_PREV, ACT_GAUGE_PAGE_NEXT, ACT_GAUGE_SELECT,
  ACT_COUNT
};
static const char* ACT_NAME[ACT_COUNT] = {
  "NONE", "VOL_UP", "VOL_DOWN", "MUTE", "NEXT", "PREV",
  "TEMP_UP", "TEMP_DOWN", "FAN_UP", "FAN_DOWN",
  "HVAC_MODE_PREV", "HVAC_MODE_NEXT", "HVAC_TOGGLE",
  "LIGHT_BRIGHTER", "LIGHT_DIMMER",
  "LIGHT_SCENE_PREV", "LIGHT_SCENE_NEXT", "LIGHT_TOGGLE",
  "GAUGE_SCROLL_UP", "GAUGE_SCROLL_DOWN",
  "GAUGE_PAGE_PREV", "GAUGE_PAGE_NEXT", "GAUGE_SELECT",
};
struct ModeMap { uint8_t knobCW, knobCCW, knobPress, left, right, up, down; };
static const ModeMap MODE_MAP[MODE_COUNT] = {
  { ACT_VOL_UP, ACT_VOL_DOWN, ACT_MUTE, ACT_PREV, ACT_NEXT, ACT_NONE, ACT_NONE },
  { ACT_TEMP_UP, ACT_TEMP_DOWN, ACT_HVAC_TOGGLE, ACT_HVAC_MODE_PREV, ACT_HVAC_MODE_NEXT, ACT_FAN_UP, ACT_FAN_DOWN },
  { ACT_LIGHT_BRIGHTER, ACT_LIGHT_DIMMER, ACT_LIGHT_TOGGLE, ACT_LIGHT_SCENE_PREV, ACT_LIGHT_SCENE_NEXT, ACT_NONE, ACT_NONE },
  { ACT_GAUGE_SCROLL_UP, ACT_GAUGE_SCROLL_DOWN, ACT_GAUGE_SELECT, ACT_GAUGE_PAGE_PREV, ACT_GAUGE_PAGE_NEXT, ACT_NONE, ACT_NONE },
};
static uint8_t  activeMode    = MODE_MEDIA;
static uint32_t modeTouchedAt = 0;

static void outPush(uint8_t key, uint8_t times) { irCalls.push_back({key, times}); }
static void flashLED(uint8_t r, uint8_t g, uint8_t b) {
  char b2[24]; snprintf(b2, sizeof b2, "%u,%u,%u", r, g, b);
  ledFlashes.push_back(b2);
}
static void linkEmit(const char* action, uint8_t count) {
  linkCalls.push_back({ MODE_NAME[activeMode], action, count });
}
static void modeSet(uint8_t m) {
  if (m >= MODE_COUNT) return;
  bool changed = (m != activeMode);
  activeMode = m;
  modeTouchedAt = millis();
  if (changed) linkEmit("MODE_ENTER", 1);
  flashLED(MODE_RGB[m][0], MODE_RGB[m][1], MODE_RGB[m][2]);
}
static void modeService() {
  if (activeMode == MODE_MEDIA) return;
  if ((int32_t)(millis() - (modeTouchedAt + MODE_TIMEOUT_MS)) >= 0) modeSet(MODE_MEDIA);
}
static void dispatchAction(uint8_t act, uint8_t count) {
  if (act == ACT_NONE || act >= ACT_COUNT) return;
  modeTouchedAt = millis();
  if (act >= ACT_VOL_UP && act <= ACT_PREV) outPush((uint8_t)(act - ACT_VOL_UP), count);
  else                                       linkEmit(ACT_NAME[act], count);
}
static void onButtonPress(const char* name, uint8_t r, uint8_t g, uint8_t b, uint8_t count = 1) {
  if      (!strcmp(name, "MEDIA")) { modeSet(MODE_MEDIA); return; }
  else if (!strcmp(name, "NAV"))   { modeSet(MODE_HVAC);  return; }
  else if (!strcmp(name, "MAP"))   { modeSet(MODE_LIGHT); return; }
  else if (!strcmp(name, "MENU"))  { modeSet(MODE_GAUGE); return; }
  else if (!strcmp(name, "BACK"))  { modeSet(MODE_MEDIA); return; }
  flashLED(r, g, b);
  const ModeMap& mm = MODE_MAP[activeMode];
  uint8_t act = ACT_NONE;
  if      (!strcmp(name, "KNOB_CW"))    act = mm.knobCW;
  else if (!strcmp(name, "KNOB_CCW"))   act = mm.knobCCW;
  else if (!strcmp(name, "KNOB_PRESS")) act = mm.knobPress;
  else if (!strcmp(name, "LEFT"))       act = mm.left;
  else if (!strcmp(name, "RIGHT"))      act = mm.right;
  else if (!strcmp(name, "UP"))         act = mm.up;
  else if (!strcmp(name, "DOWN"))       act = mm.down;
  bool isRotation = !strcmp(name, "KNOB_CW") || !strcmp(name, "KNOB_CCW");
  dispatchAction(act, isRotation ? count : 1);
}

// ── Harness ──────────────────────────────────────────────────────
static int failures = 0;
static void check(bool ok, const char* what) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) failures++;
}
static void reset() {
  irCalls.clear(); linkCalls.clear(); ledFlashes.clear();
  activeMode = MODE_MEDIA; modeTouchedAt = 0; g_now = 0;
}
static void advance(uint32_t ms) { for (uint32_t i = 0; i < ms; i++) { g_now++; modeService(); } }

int main() {
  printf("Test 1: MEDIA mode is unchanged from v1.3.0\n");
  reset();
  onButtonPress("KNOB_CW", 0,255,0, 3);
  onButtonPress("KNOB_PRESS", 0,0,255);
  onButtonPress("RIGHT", 255,255,0);
  onButtonPress("LEFT", 128,0,255);
  bool ok = irCalls.size() == 4
         && irCalls[0].key == KEY_VOL_UP && irCalls[0].count == 3
         && irCalls[1].key == KEY_MUTE   && irCalls[1].count == 1
         && irCalls[2].key == KEY_NEXT
         && irCalls[3].key == KEY_PREV;
  check(ok, "volume/mute/track still go straight to IR, detent count preserved");
  check(linkCalls.empty(), "MEDIA mode emits nothing on the link");

  printf("\nTest 2: NAV switches to HVAC and re-targets the knob\n");
  reset();
  onButtonPress("NAV", 0,0,0);
  check(activeMode == MODE_HVAC, "NAV selects HVAC");
  onButtonPress("KNOB_CW", 0,255,0, 2);
  onButtonPress("UP", 0,255,255);
  ok = linkCalls.size() == 3
    && linkCalls[0].action == "MODE_ENTER"
    && linkCalls[1].action == "TEMP_UP" && linkCalls[1].count == 2
    && linkCalls[2].action == "FAN_UP";
  check(ok, "knob becomes setpoint, UP becomes fan, counts preserved");
  check(irCalls.empty(), "NO IR is emitted while in HVAC mode");

  printf("\nTest 3: mode-select buttons never leak an action\n");
  reset();
  for (const char* n : {"MEDIA","NAV","MAP","MENU","BACK"}) onButtonPress(n, 0,0,0);
  bool onlyEnters = true;
  for (auto& c : linkCalls) if (c.action != "MODE_ENTER") onlyEnters = false;
  check(onlyEnters, "MEDIA/NAV/MAP/MENU/BACK produce only MODE_ENTER");
  check(irCalls.empty(), "and never an IR send");

  printf("\nTest 4: auto-revert to MEDIA after idle\n");
  reset();
  onButtonPress("NAV", 0,0,0);
  advance(MODE_TIMEOUT_MS - 100);
  check(activeMode == MODE_HVAC, "still HVAC just before the timeout");
  advance(200);
  check(activeMode == MODE_MEDIA, "reverted to MEDIA after the timeout");

  printf("\nTest 5: activity keeps a mode alive\n");
  reset();
  onButtonPress("NAV", 0,0,0);
  for (int i = 0; i < 5; i++) { advance(MODE_TIMEOUT_MS - 500); onButtonPress("KNOB_CW", 0,0,0, 1); }
  check(activeMode == MODE_HVAC, "repeated use prevents the revert");
  advance(MODE_TIMEOUT_MS + 100);
  check(activeMode == MODE_MEDIA, "and it still reverts once activity stops");

  printf("\nTest 6: MEDIA never times out\n");
  reset();
  advance(MODE_TIMEOUT_MS * 5);
  check(activeMode == MODE_MEDIA, "MEDIA is the resting state, no revert churn");
  check(linkCalls.empty(), "idle MEDIA emits nothing");

  printf("\nTest 7: BACK is the panic button from any mode\n");
  reset();
  for (uint8_t m : {MODE_HVAC, MODE_LIGHT, MODE_GAUGE}) {
    activeMode = m;
    onButtonPress("BACK", 0,0,0);
    if (activeMode != MODE_MEDIA) { check(false, "BACK returned to MEDIA"); break; }
  }
  check(activeMode == MODE_MEDIA, "BACK returns to MEDIA from every mode");

  printf("\nTest 8: every mode change flashes its signature colour\n");
  reset();
  onButtonPress("NAV", 0,0,0);
  bool sawAmber = false;
  for (auto& f : ledFlashes) if (f == "255,176,0") sawAmber = true;
  check(sawAmber, "HVAC flashes amber (255,176,0)");
  reset();
  onButtonPress("MAP", 0,0,0);
  bool sawViolet = false;
  for (auto& f : ledFlashes) if (f == "156,64,255") sawViolet = true;
  check(sawViolet, "LIGHT flashes violet (156,64,255)");

  printf("\nTest 9: unmapped inputs in a mode do nothing at all\n");
  reset();
  onButtonPress("MAP", 0,0,0);              // LIGHT has no UP/DOWN
  size_t before = linkCalls.size();
  onButtonPress("UP", 0,0,0);
  onButtonPress("DOWN", 0,0,0);
  check(linkCalls.size() == before, "UP/DOWN silent in LIGHT mode");
  check(irCalls.empty(), "and emit no IR");

  printf("\nTest 10: no action is reachable from the wrong mode\n");
  reset();
  onButtonPress("MENU", 0,0,0);             // GAUGE
  onButtonPress("KNOB_CW", 0,0,0, 1);
  bool leaked = false;
  for (auto& c : linkCalls)
    if (c.action.rfind("TEMP_", 0) == 0 || c.action.rfind("LIGHT_", 0) == 0) leaked = true;
  check(!leaked, "GAUGE mode cannot emit HVAC or LIGHT actions");
  check(irCalls.empty(), "GAUGE mode cannot emit IR");

  printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL TESTS PASSED",
         failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
