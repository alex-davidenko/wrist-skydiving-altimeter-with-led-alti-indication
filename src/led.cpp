#include "led.h"

#include "config.h"

#if LED_DRIVER == LED_DRIVER_NEOPIXEL
  #include <Adafruit_NeoPixel.h>
  static Adafruit_NeoPixel g_pixel(LED_COUNT, PIN_LED, NEO_GRB + NEO_KHZ800);
#endif

namespace led {

const Rgb kBlack   = {  0,   0,   0};
const Rgb kRed     = {255,   0,   0};
const Rgb kYellow  = {255, 150,   0};  // amber rather than 255,255,0 — reads as
                                       // clearly distinct from green on a
                                       // cheap WS2812
const Rgb kGreen   = {  0, 255,   0};
const Rgb kBlue    = { 60, 140, 255};  // light enough for black digits
const Rgb kMagenta = {255,   0, 200};
const Rgb kWhite   = {255, 255, 255};

namespace {

uint8_t  g_brightness   = LED_BRIGHTNESS;
Rgb      g_flashColor   = kBlack;
uint32_t g_flashUntilMs = 0;
bool     g_flashActive  = false;

uint8_t scale(uint8_t v, uint8_t b)
{
  return static_cast<uint8_t>((static_cast<uint16_t>(v) * b) / 255);
}

// True for the "on" half of a blink cycle.
bool blinkOn(uint32_t nowMs, uint16_t periodMs)
{
  return periodMs == 0 || (nowMs % periodMs) < (periodMs / 2u);
}

}  // namespace

void begin()
{
#if LED_DRIVER == LED_DRIVER_NEOPIXEL
  g_pixel.begin();
  g_pixel.show();
#elif LED_DRIVER == LED_DRIVER_NONE
  // nothing to initialise
#else
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
#endif
  off();
}

// What was last written to the hardware. This lives here, not inside render(),
// because set() and off() write to the strip directly and render() skips a
// repaint when nothing changed. With the cache private to render(), any call to
// off() left it claiming a colour the strip was not showing — and the next
// request for that same colour was then skipped as redundant.
//
// The symptom was specific and misleading: on jump 184 the strip was commanded
// green for 42 s of freefall and stayed dark, while yellow, red and the landing
// ladder all worked, because only a colour DIFFERENT from the stale cache got
// through. It looked like green was broken. Green was fine.
static Rgb     g_lastColor  = {1, 1, 1};   // not a colour we ever set
static uint8_t g_lastBright = 0;
static bool    g_primed     = false;

void set(const Rgb &c, uint8_t bright)
{
  g_lastColor = c;
  g_lastBright = bright;
  g_primed = true;

  const uint8_t b = bright ? bright : g_brightness;
  const uint8_t r = scale(c.r, b);
  const uint8_t g = scale(c.g, b);
  const uint8_t bl = scale(c.b, b);

#if LED_DRIVER == LED_DRIVER_NONE
  (void)r; (void)g; (void)bl;
#elif LED_DRIVER == LED_DRIVER_NEOPIXEL
  // Whole strip as one lamp: every pixel shows the zone colour, so a bracelet
  // reads as a single band rather than a row of dots.
  const uint32_t c32 = g_pixel.Color(r, g, bl);
  for (uint16_t i = 0; i < LED_COUNT; i++) g_pixel.setPixelColor(i, c32);
  g_pixel.show();
#elif LED_DRIVER == LED_DRIVER_RGB_CC
  analogWrite(PIN_LED_R, r);
  analogWrite(PIN_LED_G, g);
  analogWrite(PIN_LED_B, bl);
#else  // LED_DRIVER_RGB_CA — common anode sinks current, so invert
  analogWrite(PIN_LED_R, 255 - r);
  analogWrite(PIN_LED_G, 255 - g);
  analogWrite(PIN_LED_B, 255 - bl);
#endif
}

void off() { set(kBlack); }

static bool g_powered = false;

void power(bool on)
{
#if PIN_LED_PWR >= 0
  pinMode(PIN_LED_PWR, OUTPUT);
  digitalWrite(PIN_LED_PWR, on ? LED_PWR_ON : !LED_PWR_ON);
  if (on) delay(2);            // let the strip's rail come up before clocking data
#endif
  g_powered = on;
}

bool powered() { return g_powered; }

void setBrightness(uint8_t b) { g_brightness = b; }
uint8_t brightness() { return g_brightness; }

LedPattern offPattern()   { return {kBlack,   0,               0}; }
LedPattern faultPattern() { return {kMagenta, FAULT_PERIOD_MS, 0}; }

LedPattern freefallPattern(uint8_t zone)
{
  switch (zone)
  {
    case ZONE_BLINK_RED: return {kRed,    BLINK_PERIOD_MS, 0};
    case ZONE_RED:       return {kRed,    0,               0};
    case ZONE_YELLOW:    return {kYellow, 0,               0};
    case ZONE_GREEN:     return {kGreen,  0,               0};
    // Above the green band you are still in the aircraft or have just exited.
    // Dim blue keeps it distinct from "good to pull" green without being
    // distracting. Change to kGreen or kBlack to taste.
    case ZONE_ABOVE:     return {kBlue,   0,               0};
    case ZONE_OFF:
    default:             return offPattern();
  }
}

LedPattern landingPattern(uint8_t z)
{
  switch (z)
  {
    case LAND_SLOW:   return {kGreen, LANDING_SLOW_PERIOD_MS, 0};
    case LAND_FAST:   return {kGreen, LANDING_FAST_PERIOD_MS, 0};
    // "bright steady green" — full output regardless of the configured
    // brightness, because this is the one that matters on final approach.
    case LAND_STEADY: return {kGreen, 0, 255};
    case LAND_DARK_LOW:
    case LAND_DARK_HIGH:
    default:          return offPattern();
  }
}

LedPattern flightPattern(uint8_t mode, uint8_t zone, uint8_t landingZone,
                         float altitudeM, bool inAircraft)
{
  if (mode == MODE_FREEFALL) return freefallPattern(zone);

  // In the aircraft — climbing, or levelled off on jump run. Blue with the
  // altitude, all the way to exit. `inAircraft` is what carries it through the
  // level-off, when vertical speed alone would look like a canopy ride.
  if (mode == MODE_CLIMB || inAircraft)
  {
    const bool unbuckle = altitudeM >= CLIMB_UNBUCKLE_LO_M &&
                          altitudeM <= CLIMB_UNBUCKLE_HI_M;
    // screenOnly: the panel goes blue, the strip stays dark for the climb.
    return {kBlue, unbuckle ? (uint16_t)CLIMB_UNBUCKLE_BLINK_MS : (uint16_t)0, 0, true};
  }

  // Under canopy the freefall colours mean nothing — a good canopy at 900 m
  // does not warrant a red light.
  return landingPattern(landingZone);
}

void flashOnce(const Rgb &c, uint16_t durationMs, uint32_t nowMs)
{
  g_flashColor   = c;
  g_flashUntilMs = nowMs + durationMs;
  g_flashActive  = true;
}

void render(const LedPattern &p, uint32_t nowMs)
{
  // Only repaint when the output actually changes — WS2812 updates briefly
  // disable interrupts, and there is no reason to do that 40x a second.
  Rgb     want   = p.color;
  uint8_t bright = p.brightness;

  if (g_flashActive)
  {
    if (static_cast<int32_t>(nowMs - g_flashUntilMs) >= 0) g_flashActive = false;
    else { want = g_flashColor; bright = 0; }
  }

  if (!g_flashActive && !blinkOn(nowMs, p.periodMs)) want = kBlack;

  if (!g_primed || want.r != g_lastColor.r || want.g != g_lastColor.g ||
      want.b != g_lastColor.b || bright != g_lastBright)
  {
    set(want, bright);   // set() is what updates the cache now
  }
}

void selfTest()
{
  const Rgb seq[] = {kRed, kYellow, kGreen, kBlue, kMagenta};
  for (const Rgb &c : seq)
  {
    set(c);
    delay(350);
  }
  off();
}

// Play every pattern in order so the landing blink rates can be eyeballed on
// the bench without needing altitude. Blocking; only called from the console.
void patternPreview()
{
  struct Step { const char *name; LedPattern p; uint16_t ms; };
  const Step steps[] = {
      {"climb flash x3", {kGreen,  0, 0},                      0},
      {"ABOVE (blue)",   freefallPattern(ZONE_ABOVE),       1500},
      {"GREEN",          freefallPattern(ZONE_GREEN),       1500},
      {"YELLOW",         freefallPattern(ZONE_YELLOW),      1500},
      {"RED",            freefallPattern(ZONE_RED),         1500},
      {"BLINK RED",      freefallPattern(ZONE_BLINK_RED),   2500},
      {"LAND 3 blink/s", landingPattern(LAND_SLOW),         3000},
      {"LAND 6 blink/s", landingPattern(LAND_FAST),         3000},
      {"LAND steady",    landingPattern(LAND_STEADY),       2000},
      {"FAULT",          faultPattern(),                    2000},
  };

  for (const Step &s : steps)
  {
    Serial.printf("  %s\n", s.name);
    if (s.ms == 0)
    {
      for (int i = 0; i < 3; i++) { set(kGreen); delay(150); off(); delay(400); }
      continue;
    }
    const uint32_t end = millis() + s.ms;
    while (static_cast<int32_t>(millis() - end) < 0)
    {
      render(s.p, millis());
      delay(5);
    }
  }
  off();
}

}  // namespace led
