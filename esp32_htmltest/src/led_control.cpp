#include "led_control.h"

// current command (received via Serial1 or Serial)
volatile int currentCommand = 0;
int lastCommand = -1;

// timing
unsigned long nowMillis = 0;

// helper: convert LED number (1..103) to index (0..102)
inline int ledIndex(int ledNum) {
  if (ledNum < 1) ledNum = 1;
  if (ledNum > NUM_LEDS) ledNum = NUM_LEDS;
  return ledNum - 1;
}

// ranges (LED numbers)
const int FRONT1_MIN = 1,   FRONT1_MAX = 7;    // front left-ish (1 = center front)
const int LEFT_MIN   = 8,   LEFT_MAX   = 34;
const int BACK_MIN   = 35,  BACK_MAX   = 58;   // back: 35..58
const int RIGHT_MIN  = 59,  RIGHT_MAX  = 87;
const int FRONT2_MIN = 88,  FRONT2_MAX = 103;  // front right-ish

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- helpers for colors where channel sums must be <= BRIGHTNESS ---
// produce scaled RGB triple (0..255 each) and pack to 32-bit via strip.Color()
uint32_t scaledColorFromRGB(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t sum = (uint32_t)r + (uint32_t)g + (uint32_t)b;
  if (sum == 0) return strip.Color(0,0,0);
  if (sum <= BRIGHTNESS) {
    return strip.Color(r, g, b);
  } else {
    // scale down proportionally to make sum == BRIGHTNESS
    float scale = (float)BRIGHTNESS / (float)sum;
    uint8_t rn = (uint8_t)(r * scale);
    uint8_t gn = (uint8_t)(g * scale);
    uint8_t bn = (uint8_t)(b * scale);
    return strip.Color(rn, gn, bn);
  }
}

// convenience named colors (constructed to sum = BRIGHTNESS roughly)
inline uint32_t colorRed()    { return scaledColorFromRGB(BRIGHTNESS, 0, 0); }
inline uint32_t colorGreen()  { return scaledColorFromRGB(0, BRIGHTNESS, 0); }
inline uint32_t colorBlue()   { return scaledColorFromRGB(0, 0, BRIGHTNESS); }
inline uint32_t colorYellow() { return scaledColorFromRGB(BRIGHTNESS/2, BRIGHTNESS - BRIGHTNESS/2, 0); } // approx split
inline uint32_t colorCyan()   { return scaledColorFromRGB(0, BRIGHTNESS/2, BRIGHTNESS - BRIGHTNESS/2); }
inline uint32_t colorPurple() { return scaledColorFromRGB(BRIGHTNESS/2, 0, BRIGHTNESS - BRIGHTNESS/2); }
inline uint32_t colorOrange() { return scaledColorFromRGB((uint8_t)(BRIGHTNESS*0.7f), (uint8_t)(BRIGHTNESS*0.3f), 0); }

// clear helpers
void clearAll() { strip.clear(); }
void clearRangeLEDnum(int a, int b) { if (a> b) return; for (int n = a; n<=b; ++n) strip.setPixelColor(ledIndex(n), 0); }

// Useful: convert ColorHSV result to scaled RGB meeting BRIGHTNESS
uint32_t colorHSV_scaled(uint32_t hsv) {
  // hsv is produced by strip.ColorHSV(hue*182, sat, val) usage below; it returns 24-bit RGB
  uint8_t r = (hsv >> 16) & 0xFF;
  uint8_t g = (hsv >> 8) & 0xFF;
  uint8_t b = hsv & 0xFF;
  return scaledColorFromRGB(r, g, b);
}

void stripAck(){
    clearAll();
    strip.show();
}

void stripINIT(){
    strip.begin();
    strip.setBrightness(BRIGHTNESS);
    strip.show();
}

// ---------------- IMPLEMENTATIONS ----------------

// 1. FirstConnectionBT: startup animation (same as earlier)
void commandFirstConnectionBT() {
  static bool ran = false;
  if (!ran) {
    for (int i = 1; i <= NUM_LEDS; ++i) {
      strip.setPixelColor(ledIndex(i), colorBlue());
      strip.show();
      delay(12);
      strip.setPixelColor(ledIndex(i), 0);
    }
    delay(50);
    for (int i = 1; i <= NUM_LEDS; ++i) strip.setPixelColor(ledIndex(i), colorBlue());
    strip.show();
    delay(250);
    clearAll();
    strip.show();
    ran = true;
  } else {
    for (int i = 1; i <= NUM_LEDS; ++i) strip.setPixelColor(ledIndex(i), colorBlue());
    strip.show();
  }
}

// 2. ManualMode_idle: blue with cycling purple hue (kept logic)
void commandManual_idle() {
  static int pos = 0;
  static unsigned long t = 0;
  const int SPREAD = 20;        // wide purple area
  const unsigned long PERIOD = 100; // ms per shift
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;
  pos = (pos + 1) % NUM_LEDS;
  for (int i = 1; i <= NUM_LEDS; ++i) {
    int idx = ledIndex(i);
    int dist = abs(idx - pos);
    if (dist > NUM_LEDS/2) dist = NUM_LEDS - dist;
    uint8_t r = 0, g = 0, b = BRIGHTNESS;
    if (dist < SPREAD) {
      r = map(SPREAD - dist, 0, SPREAD, 0, BRIGHTNESS/2);
    }
    strip.setPixelColor(idx, scaledColorFromRGB(r, g, b));
  }
  strip.show();
}

// 3. ManualMode_forward: blue pair scan from (46,47) outward until (7,87)
void commandManual_forward() {
  static int step = 0;
  static unsigned long t = 0;
  const unsigned long PERIOD = 60; // adjust speed
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;

  int leftLed  = 46 - step;
  int rightLed = 47 + step;

  if (leftLed < 7 || rightLed > 87) {
    step = 0;
    leftLed = 46;
    rightLed = 47;
  }

  clearAll();
  strip.setPixelColor(ledIndex(leftLed), colorBlue());
  strip.setPixelColor(ledIndex(rightLed), colorBlue());

  // halos (dimmer)
  if (leftLed - 1 >= 1) strip.setPixelColor(ledIndex(leftLed - 1), scaledColorFromRGB(0,0,BRIGHTNESS/3));
  if (rightLed + 1 <= NUM_LEDS) strip.setPixelColor(ledIndex(rightLed + 1), scaledColorFromRGB(0,0,BRIGHTNESS/3));

  strip.show();
  step++;
}

// 4. ManualMode_backwards: red pulsating in back (35..58) only
void commandManual_backwards() {
  static bool on = false;
  static unsigned long t = 0;
  const unsigned long PERIOD = 300;
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;
  on = !on;
  clearAll();
  if (on) {
    for (int n = BACK_MIN; n <= BACK_MAX; ++n) strip.setPixelColor(ledIndex(n), colorRed());
  }
  strip.show();
}

// 5. ManualMode_left: yellow animation from 46 & 1 → center, keep 3-LED clusters visible
void commandManual_left() {
  static int step = 0;
  static unsigned long t = 0;
  const unsigned long PERIOD = 60;
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;

  int a = 46 - step; // leftwards from 46
  int b = 1 + step;  // rightwards from 1 toward middle

  if (a <= b) { // reset when meet/cross
    step = 0;
    a = 46; b = 1;
  }

  clearAll();
  // For each side, light a cluster of 3 LEDs centered towards middle so it's noticeable
  // Left cluster: a, a-1 (towards front), a+1 (towards back) but clamp within valid range
  for (int ca = -1; ca <= 1; ++ca) {
    int ln = a + ca;
    if (ln >= 1 && ln <= NUM_LEDS) strip.setPixelColor(ledIndex(ln), colorYellow());
  }
  // Right cluster around b
  for (int cb = -1; cb <= 1; ++cb) {
    int rn = b + cb;
    if (rn >= 1 && rn <= NUM_LEDS) strip.setPixelColor(ledIndex(rn), colorYellow());
  }

  // small dim trailing neighbors for visibility
  if (a+2 <= NUM_LEDS) strip.setPixelColor(ledIndex(a+2), scaledColorFromRGB(20,20,0));
  if (b-2 >= 1) strip.setPixelColor(ledIndex(b-2), scaledColorFromRGB(20,20,0));

  strip.show();
  step++;
}

// 6. ManualMode_right: yellow animation from 47 & 103 → center, 3-LED clusters
void commandManual_right() {
  static int step = 0;
  static unsigned long t = 0;
  const unsigned long PERIOD = 60;
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;
  int a = 47 + step;    // increases (toward front)
  int b = 103 - step;   // decreases (toward middle)

  if (a >= b) { step = 0; a = 47; b = 103; }

  clearAll();
  for (int ca = -1; ca <= 1; ++ca) {
    int ln = a + ca;
    if (ln >= 1 && ln <= NUM_LEDS) strip.setPixelColor(ledIndex(ln), colorYellow());
  }
  for (int cb = -1; cb <= 1; ++cb) {
    int rn = b + cb;
    if (rn >= 1 && rn <= NUM_LEDS) strip.setPixelColor(ledIndex(rn), colorYellow());
  }
  if (a-2 >= 1) strip.setPixelColor(ledIndex(a-2), scaledColorFromRGB(20,20,0));
  if (b+2 <= NUM_LEDS) strip.setPixelColor(ledIndex(b+2), scaledColorFromRGB(20,20,0));
  strip.show();
  step++;
}

// 7. LineMode_greenIdle: like manual idle but green base and only 7..87 visible
void commandLine_greenIdle() {
  static int pos = 0;
  static unsigned long t = 0;
  const int SPREAD = 20;
  const unsigned long PERIOD = 100;
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;
  pos = (pos + 1) % (RIGHT_MAX - LEFT_MIN + 1);

  clearAll();
  // active leds 7..87
  for (int n = 7; n <= 87; ++n) {
    int idx = ledIndex(n);
    int dist = abs(idx - pos);
    if (dist > NUM_LEDS / 2) dist = NUM_LEDS - dist;
    uint8_t r = 0, g = BRIGHTNESS, b = 0;
    if (dist < SPREAD) {
      uint8_t addB = map(SPREAD - dist, 0, SPREAD, 0, BRIGHTNESS/2);
      // reduce green so sum <= BRIGHTNESS
      if (addB > g) addB = g;
      g = BRIGHTNESS - addB;
      b = addB;
    }
    strip.setPixelColor(idx, scaledColorFromRGB(r, g, b));
  }
  strip.show();
}

// 8. LineMode_redIdle: red with cycling yellow hue full (7..87)
void commandLine_redIdle() {
  static int pos = 0; static unsigned long t = 0;
  const int SPREAD = 12; const unsigned long PERIOD = 120;
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;
  pos = (pos + 1) % (RIGHT_MAX - LEFT_MIN + 1);
  clearAll();
  for (int n = 7; n <= 87; ++n) {
    int idx = ledIndex(n);
    int dist = abs(idx - pos);
    if (dist > NUM_LEDS / 2) dist = NUM_LEDS - dist;
    uint8_t r = BRIGHTNESS, g = 0, b = 0;
    if (dist < SPREAD) {
      uint8_t addG = map(SPREAD - dist, 0, SPREAD, 0, BRIGHTNESS/2);
      if (addG > (127 - r)) addG = (127 - r);
      g = addG;
    }
    strip.setPixelColor(idx, scaledColorFromRGB(r, g, b));
  }
  strip.show();
}

// 9. LineMode_blueIdle: blue with cycling purple hue (7..87)
void commandLine_blueIdle() {
  static int pos = 0; static unsigned long t = 0;
  const int SPREAD = 18; const unsigned long PERIOD = 100;
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;
  pos = (pos + 1) % (RIGHT_MAX - LEFT_MIN + 1);
  clearAll();
  for (int n = 7; n <= 87; ++n) {
    int idx = ledIndex(n);
    int dist = abs(idx - pos);
    if (dist > NUM_LEDS / 2) dist = NUM_LEDS - dist;
    uint8_t r = 0, g = 0, b = BRIGHTNESS;
    if (dist < SPREAD) {
      r = map(SPREAD - dist, 0, SPREAD, 0, BRIGHTNESS/2);
    }
    strip.setPixelColor(idx, scaledColorFromRGB(r, g, b));
  }
  strip.show();
}

// 10. LineMode_following: same as forward but only 7..87
void commandLine_following() {
  static int step = 0; static unsigned long t = 0;
  const unsigned long PERIOD = 60;
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;
  int leftLed = 46 - step;
  int rightLed = 47 + step;
  if (leftLed < 7 || rightLed > 87) { step = 0; leftLed = 46; rightLed = 47; }
  clearAll();
  // only set if within 7..87
  if (leftLed >= 7 && leftLed <= 87) strip.setPixelColor(ledIndex(leftLed), colorBlue());
  if (rightLed >= 7 && rightLed <= 87) strip.setPixelColor(ledIndex(rightLed), colorBlue());
  strip.show();
  step++;
}

// 11. ObjectNearby: gradual red on/off (0.5s ramp up, 0.5s ramp down) on 7..87
void commandObjectNearby() {
  static unsigned long phaseStart = 0;
  static bool increasing = true;
  const unsigned long RAMP_MS = 500;
  if (phaseStart == 0) phaseStart = nowMillis;
  unsigned long dt = (nowMillis - phaseStart) % (RAMP_MS * 2);
  uint8_t val;
  if (dt < RAMP_MS) {
    // ramp up
    val = (uint8_t) map(dt, 0, RAMP_MS, 0, BRIGHTNESS);
  } else {
    // ramp down
    val = (uint8_t) map(dt - RAMP_MS, 0, RAMP_MS, BRIGHTNESS, 0);
  }
  clearAll();
  for (int n = 7; n <= 87; ++n) strip.setPixelColor(ledIndex(n), scaledColorFromRGB(val, 0, 0));
  strip.show();
}

// 12. SettingsWindow: ramp on/off in 1.5s, cycle colors green -> blue -> red each cycle
void commandSettingsWindow() {
  static unsigned long cycleStart = 0;
  static int cyclePhase = 0; // 0=green,1=blue,2=red
  const unsigned long RAMP_MS = 1500;
  if (cycleStart == 0) cycleStart = nowMillis;
  unsigned long elapsed = nowMillis - cycleStart;
  unsigned long dt = elapsed % (RAMP_MS * 2); // one up + one down
  // compute brightness scalar 0..BRIGHTNESS
  uint8_t val;
  if (dt < RAMP_MS) val = map(dt, 0, RAMP_MS, 0, BRIGHTNESS);
  else val = map(dt - RAMP_MS, 0, RAMP_MS, BRIGHTNESS, 0);
  // switch color every full up+down cycle
  if (elapsed / (RAMP_MS*2) > 0 && ((elapsed / (RAMP_MS*2)) != ((elapsed - (nowMillis - elapsed)) / (RAMP_MS*2)))) {
    // not needed, keep simple: rotate based on elapsed
  }
  int cycleIndex = (elapsed / (RAMP_MS*2)) % 3;
  clearAll();
  if (cycleIndex == 0) { // green
    for (int i = 1; i <= NUM_LEDS; ++i) strip.setPixelColor(ledIndex(i), scaledColorFromRGB(0, val, 0));
  } else if (cycleIndex == 1) { // blue
    for (int i = 1; i <= NUM_LEDS; ++i) strip.setPixelColor(ledIndex(i), scaledColorFromRGB(0, 0, val));
  } else { // red
    for (int i = 1; i <= NUM_LEDS; ++i) strip.setPixelColor(ledIndex(i), scaledColorFromRGB(val, 0, 0));
  }
  strip.show();
}

// 13. Emergency stop: fast flashing red all LEDs
void commandEmergencyStop() {
  static bool on = false;
  static unsigned long t = 0;
  const unsigned long PERIOD = 150;
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;
  on = !on;
  for (int i = 1; i <= NUM_LEDS; ++i) strip.setPixelColor(ledIndex(i), on ? colorRed() : 0);
  strip.show();
}

// 14 / 0. AllOff
void commandAllOff() {
  clearAll();
  strip.show();
}

// 15. RGB slider (rainbow) - scaled
void commandRGBSlider() {
  static int baseHue = 0;
  static unsigned long t = 0;
  const unsigned long PERIOD = 20;
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;
  baseHue = (baseHue + 2) % 360;
  for (int i = 0; i < NUM_LEDS; ++i) {
    int hue = (baseHue + (i * 360 / NUM_LEDS)) % 360;
    uint32_t c = strip.ColorHSV(hue * 182, 255, 255); // use val=255 then scale
    // scale to BRIGHTNESS
    strip.setPixelColor(i, colorHSV_scaled(c));
  }
  strip.show();
}

// 19 AMR_Idle: Orange base with a black spot (twice the manual idle spread)
void commandAMR_Idle() {
  static int pos = 0;
  static unsigned long t = 0;
  const int SPREAD = 40; // twice manual (manual used 20)
  const unsigned long PERIOD = 140;
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;
  pos = (pos + 1) % NUM_LEDS;
  for (int i = 1; i <= NUM_LEDS; ++i) {
    int idx = ledIndex(i);
    int dist = abs(idx - pos);
    if (dist > NUM_LEDS/2) dist = NUM_LEDS - dist;
    // black spot centered at pos: where dist < SPREAD => black, else orange base
    if (dist < SPREAD) strip.setPixelColor(idx, 0);
    else strip.setPixelColor(idx, colorOrange());
  }
  strip.show();
}

// 20 AMR_Receiving: same as AMR_Idle but spot spins very fast
void commandAMR_Receiving() {
  static int pos = 0;
  static unsigned long t = 0;
  const int SPREAD = 40;
  const unsigned long PERIOD = 30; // very fast
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;
  pos = (pos + 3) % NUM_LEDS; // faster increments
  for (int i = 1; i <= NUM_LEDS; ++i) {
    int idx = ledIndex(i);
    int dist = abs(idx - pos);
    if (dist > NUM_LEDS/2) dist = NUM_LEDS - dist;
    if (dist < SPREAD) strip.setPixelColor(idx, 0);
    else strip.setPixelColor(idx, colorOrange());
  }
  strip.show();
}

// 21 AMR_RouteLoaded: like AMR_Idle but green base
void commandAMR_RouteLoaded() {
  static int pos = 0;
  static unsigned long t = 0;
  const int SPREAD = 40;
  const unsigned long PERIOD = 140;
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;
  pos = (pos + 1) % NUM_LEDS;
  for (int i = 1; i <= NUM_LEDS; ++i) {
    int idx = ledIndex(i);
    int dist = abs(idx - pos);
    if (dist > NUM_LEDS/2) dist = NUM_LEDS - dist;
    if (dist < SPREAD) strip.setPixelColor(idx, 0);
    else strip.setPixelColor(idx, colorGreen());
  }
  strip.show();
}

// 22 AMR_Running: same as Manual forward but green colored
void commandAMR_Running() {
  static int step = 0;
  static unsigned long t = 0;
  const unsigned long PERIOD = 60;
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;
  int leftLed = 46 - step;
  int rightLed = 47 + step;
  if (leftLed < 7 || rightLed > 87) { step = 0; leftLed = 46; rightLed = 47; }
  clearAll();
  if (leftLed >= 1 && leftLed <= NUM_LEDS) strip.setPixelColor(ledIndex(leftLed), colorGreen());
  if (rightLed >= 1 && rightLed <= NUM_LEDS) strip.setPixelColor(ledIndex(rightLed), colorGreen());
  strip.show();
  step++;
}

// 30 ChristmasMode: dynamic flickering garland + sparkles
void commandChristmasMode() {
  static unsigned long t = 0;
  const unsigned long PERIOD = 80;
  if (nowMillis - t < PERIOD) return;
  t = nowMillis;

  clearAll();

  // ------------------------------
  // 1. Alternating garland (static, but visually solid)
  // ------------------------------
  for (int i = 0; i < NUM_LEDS; i++) {
    if ((i / 3) % 2 == 0)
      strip.setPixelColor(i, colorGreen());   // your scaled green
    else
      strip.setPixelColor(i, colorRed());     // your scaled red
  }

  // ------------------------------
  // 2. Random flickering ornaments
  // ------------------------------
  const int TWINKLE_COUNT = 10;   // more flicker = more alive

  for (int k = 0; k < TWINKLE_COUNT; k++) {
    int r = random(NUM_LEDS);

    // choose white-ish or blue
    bool white = random(2);

    // random brightness respecting max 127 limit
    uint8_t b = random(20, 100);   // flicker intensity change

    uint8_t rC = white ? b : 0;
    uint8_t gC = white ? b : 0;
    uint8_t bC = white ? b : b;    // blue stays blue

    strip.setPixelColor(r, scaledColorFromRGB(rC, gC, bC));
  }

  // ------------------------------
  // 3. Occasional spark bursts (rare bonus effect)
  // ------------------------------
  if (random(10) == 0) {   // ~10% chance each frame
    int burstOrigin = random(NUM_LEDS);
    for (int j = -2; j <= 2; j++) {
      int idx = burstOrigin + j;
      if (idx >= 0 && idx < NUM_LEDS) {
        uint8_t sparkle = 80 + random(40);
        strip.setPixelColor(idx, scaledColorFromRGB(sparkle, sparkle, sparkle));
      }
    }
  }

  strip.show();
}

