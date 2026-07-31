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
enum : uint8_t { MODE_RADIO = 0, MODE_HVAC, MODE_ILLUM, MODE_GAUGE,
                 MODE_TSDASH, MODE_COUNT };
static const char* MODE_NAME[MODE_COUNT] =
  { "RADIO", "HVAC", "ILLUM", "GAUGE", "TSDASH" };
static const uint8_t MODE_RGB[MODE_COUNT][3] = {
  { 0x2C, 0xE8, 0xD8 }, { 0xFF, 0xB0, 0x00 },
  { 0x9C, 0x40, 0xFF }, { 0x5C, 0xB8, 0xFF },
  { 0x3A, 0xFF, 0x8C },
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
  ACT_TSDASH_NEXT, ACT_TSDASH_PREV, ACT_TSDASH_CFG, ACT_TSDASH_HOME,
  ACT_AUX_SWAP, ACT_SYSTEM_TOGGLE,
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
  "TSDASH_NEXT", "TSDASH_PREV", "TSDASH_CFG", "TSDASH_HOME",
  "AUX_SWAP", "SYSTEM_TOGGLE",
};
struct ModeMap { uint8_t knobCW, knobCCW, knobPress, left, right, up, down; };
static const ModeMap MODE_MAP[MODE_COUNT] = {
  { ACT_VOL_UP, ACT_VOL_DOWN, ACT_MUTE, ACT_PREV, ACT_NEXT, ACT_NONE, ACT_NONE },
  { ACT_TEMP_UP, ACT_TEMP_DOWN, ACT_HVAC_TOGGLE, ACT_HVAC_MODE_PREV, ACT_HVAC_MODE_NEXT, ACT_FAN_UP, ACT_FAN_DOWN },
  { ACT_LIGHT_BRIGHTER, ACT_LIGHT_DIMMER, ACT_LIGHT_TOGGLE, ACT_LIGHT_SCENE_PREV, ACT_LIGHT_SCENE_NEXT, ACT_NONE, ACT_NONE },
  { ACT_GAUGE_SCROLL_UP, ACT_GAUGE_SCROLL_DOWN, ACT_GAUGE_SELECT, ACT_GAUGE_PAGE_PREV, ACT_GAUGE_PAGE_NEXT, ACT_NONE, ACT_NONE },
  { ACT_TSDASH_NEXT, ACT_TSDASH_PREV, ACT_TSDASH_HOME, ACT_TSDASH_PREV, ACT_TSDASH_NEXT, ACT_TSDASH_CFG, ACT_TSDASH_HOME },
};
static uint8_t  activeMode    = MODE_RADIO;
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
  if (activeMode == MODE_RADIO) return;
  if ((int32_t)(millis() - (modeTouchedAt + MODE_TIMEOUT_MS)) >= 0) modeSet(MODE_RADIO);
}
static void dispatchAction(uint8_t act, uint8_t count) {
  if (act == ACT_NONE || act >= ACT_COUNT) return;
  modeTouchedAt = millis();
  if (act >= ACT_VOL_UP && act <= ACT_PREV) outPush((uint8_t)(act - ACT_VOL_UP), count);
  else                                       linkEmit(ACT_NAME[act], count);
}
static void onButtonPress(const char* name, uint8_t r, uint8_t g, uint8_t b, uint8_t count = 1) {
  static const struct { const char* button; uint8_t mode; } MODE_BUTTONS[] = {
    { "MEDIA", MODE_RADIO }, { "MENU", MODE_HVAC }, { "MAP", MODE_ILLUM },
    { "NAV", MODE_GAUGE }, { "OPTION", MODE_TSDASH },
  };
  for (const auto& mb : MODE_BUTTONS)
    if (!strcmp(name, mb.button)) { modeSet(mb.mode); return; }
  static const struct { const char* button; uint8_t action; } GLOBAL_BUTTONS[] = {
    { "COM", ACT_AUX_SWAP }, { "BACK", ACT_SYSTEM_TOGGLE },
  };
  for (const auto& gb : GLOBAL_BUTTONS)
    if (!strcmp(name, gb.button)) { flashLED(r,g,b); dispatchAction(gb.action, 1); return; }
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
  activeMode = MODE_RADIO; modeTouchedAt = 0; g_now = 0;
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

  printf("\nTest 2: MENU switches to HVAC and re-targets the knob\n");
  reset();
  onButtonPress("MENU", 0,0,0);
  check(activeMode == MODE_HVAC, "MENU selects HVAC");
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
  for (const char* n : {"MEDIA","NAV","MAP","MENU","OPTION"}) onButtonPress(n, 0,0,0);
  bool onlyEnters = true;
  for (auto& c : linkCalls) if (c.action != "MODE_ENTER") onlyEnters = false;
  check(onlyEnters, "MEDIA/NAV/MAP/MENU/OPTION produce only MODE_ENTER");
  check(irCalls.empty(), "and never an IR send");

  printf("\nTest 4: auto-revert to MEDIA after idle\n");
  reset();
  onButtonPress("MENU", 0,0,0);
  advance(MODE_TIMEOUT_MS - 100);
  check(activeMode == MODE_HVAC, "still HVAC just before the timeout");
  advance(200);
  check(activeMode == MODE_RADIO, "reverted to MEDIA after the timeout");

  printf("\nTest 5: activity keeps a mode alive\n");
  reset();
  onButtonPress("MENU", 0,0,0);
  for (int i = 0; i < 5; i++) { advance(MODE_TIMEOUT_MS - 500); onButtonPress("KNOB_CW", 0,0,0, 1); }
  check(activeMode == MODE_HVAC, "repeated use prevents the revert");
  advance(MODE_TIMEOUT_MS + 100);
  check(activeMode == MODE_RADIO, "and it still reverts once activity stops");

  printf("\nTest 6: MEDIA never times out\n");
  reset();
  advance(MODE_TIMEOUT_MS * 5);
  check(activeMode == MODE_RADIO, "MEDIA is the resting state, no revert churn");
  check(linkCalls.empty(), "idle MEDIA emits nothing");

  printf("\nTest 7: MEDIA is the way home from any mode\n");
  // Was "BACK is the panic button". BACK became the SYSTEM STATUS toggle in
  // 1.8.0 at the owner's request — MEDIA carries the printed label for the
  // default mode and reaches it just as fast, so the second way home was
  // redundant. The property still worth guaranteeing is that ONE button gets
  // you to volume without looking, so that is what this now asserts.
  reset();
  for (uint8_t m : {MODE_HVAC, MODE_ILLUM, MODE_GAUGE, MODE_TSDASH}) {
    activeMode = m;
    onButtonPress("MEDIA", 0,0,0);
    if (activeMode != MODE_RADIO) { check(false, "MEDIA returned to RADIO"); break; }
  }
  check(activeMode == MODE_RADIO, "MEDIA returns to RADIO from every mode");

  printf("\nTest 8: every mode change flashes its signature colour\n");
  reset();
  onButtonPress("MENU", 0,0,0);
  bool sawAmber = false;
  for (auto& f : ledFlashes) if (f == "255,176,0") sawAmber = true;
  check(sawAmber, "HVAC (MENU) flashes amber (255,176,0)");
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
  onButtonPress("NAV", 0,0,0);              // GAUGE
  onButtonPress("KNOB_CW", 0,0,0, 1);
  bool leaked = false;
  for (auto& c : linkCalls)
    if (c.action.rfind("TEMP_", 0) == 0 || c.action.rfind("LIGHT_", 0) == 0) leaked = true;
  check(!leaked, "GAUGE mode cannot emit HVAC or LIGHT actions");
  check(irCalls.empty(), "GAUGE mode cannot emit IR");

  // ── TSDASH (OPTION) ──────────────────────────────────────────────
  // GAUGE and TSDASH page two different screens — the backup cluster and
  // the TunerStudio Pi. Confusing them means the knob silently drives the
  // wrong display, which is exactly the class of foot-gun this file exists
  // to catch, so the separation is asserted in both directions.
  printf("\nTest 11: OPTION enters TSDASH and its inputs map correctly\n");
  reset();
  onButtonPress("OPTION", 0,0,0);
  check(activeMode == MODE_TSDASH, "OPTION selects TSDASH");
  check(!ledFlashes.empty() && ledFlashes.back() == "58,255,140",
        "TSDASH flashes spring green (58,255,140)");

  reset();
  onButtonPress("OPTION", 0,0,0);
  onButtonPress("KNOB_CW",    0,0,0, 1);
  onButtonPress("KNOB_CCW",   0,0,0, 1);
  onButtonPress("UP",    0,0,0, 1);
  onButtonPress("DOWN",  0,0,0, 1);
  onButtonPress("KNOB_PRESS", 0,0,0, 1);
  std::vector<std::string> got;
  for (auto& c : linkCalls) if (c.action != "MODE_ENTER") got.push_back(c.action);
  check(got == std::vector<std::string>{ "TSDASH_NEXT", "TSDASH_PREV",
                                         "TSDASH_CFG", "TSDASH_HOME",
                                         "TSDASH_HOME" },
        "CW/CCW page, tilt up/down = CFG/HOME, press = HOME");
  check(irCalls.empty(), "TSDASH mode cannot emit IR");

  printf("\nTest 12: TSDASH and GAUGE cannot reach each other\n");
  reset();
  onButtonPress("OPTION", 0,0,0);           // TSDASH
  onButtonPress("KNOB_CW",   0,0,0, 1);
  onButtonPress("KNOB_CCW",  0,0,0, 1);
  onButtonPress("LEFT", 0,0,0, 1);
  onButtonPress("RIGHT",0,0,0, 1);
  bool gaugeLeak = false;
  for (auto& c : linkCalls)
    if (c.action.rfind("GAUGE_", 0) == 0) gaugeLeak = true;
  check(!gaugeLeak, "TSDASH mode cannot emit GAUGE actions");

  reset();
  onButtonPress("NAV", 0,0,0);              // GAUGE
  onButtonPress("KNOB_CW",   0,0,0, 1);
  onButtonPress("KNOB_CCW",  0,0,0, 1);
  onButtonPress("LEFT", 0,0,0, 1);
  onButtonPress("RIGHT",0,0,0, 1);
  bool tsdashLeak = false;
  for (auto& c : linkCalls)
    if (c.action.rfind("TSDASH_", 0) == 0) tsdashLeak = true;
  check(!tsdashLeak, "GAUGE mode cannot emit TSDASH actions");

  printf("\nTest 13: BACK is a global SYSTEM_TOGGLE, not a mode\n");
  for (uint8_t m : { MODE_RADIO, MODE_HVAC, MODE_ILLUM, MODE_GAUGE, MODE_TSDASH }) {
    reset();
    modeSet(m);
    uint8_t before = activeMode;
    linkCalls.clear();
    onButtonPress("BACK", 0,0,0);
    bool one = linkCalls.size() == 1 && linkCalls[0].action == "SYSTEM_TOGGLE";
    check(one && activeMode == before,
          "BACK toggles system status and leaves the mode alone");
  }
  reset();
  onButtonPress("BACK", 0,0,0);
  check(irCalls.empty(), "BACK never emits IR, even from RADIO");

  printf("\nTest 14: COM still reaches AUX_SWAP from every mode\n");
  for (uint8_t m : { MODE_RADIO, MODE_HVAC, MODE_ILLUM, MODE_GAUGE, MODE_TSDASH }) {
    reset();
    modeSet(m);
    linkCalls.clear();
    onButtonPress("COM", 0,0,0);
    check(linkCalls.size() == 1 && linkCalls[0].action == "AUX_SWAP",
          "COM emits AUX_SWAP regardless of mode");
  }

  printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL TESTS PASSED",
         failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
