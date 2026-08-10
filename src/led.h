#pragma once
//
// Thin RGB output layer. Swapping between the onboard WS2812 and a discrete
// RGB LED is a one-line change in config.h — everything above this file just
// describes a pattern.
//

#include <Arduino.h>

#include "flight_mode.h"
#include "zones.h"

struct Rgb
{
  uint8_t r, g, b;
};

// What the LED should be doing right now.
//   periodMs == 0 -> solid
//   brightness == 0 -> use the configured global brightness
struct LedPattern
{
  Rgb      color;
  uint16_t periodMs;
  uint8_t  brightness;
};

namespace led {

void begin();

void set(const Rgb &c, uint8_t brightness = 0);
void off();

// Strip power, through the high-side MOSFET. Idle current is ~1 mA per pixel
// whatever they are showing, so this is what keeps a bracelet off the power
// budget. No-op when PIN_LED_PWR is -1.
void power(bool on);
bool powered();

void setBrightness(uint8_t b);
uint8_t brightness();

// Named colours, so callers do not sprinkle magic RGB triples around.
extern const Rgb kBlack;
extern const Rgb kRed;
extern const Rgb kYellow;
extern const Rgb kGreen;
extern const Rgb kBlue;
extern const Rgb kMagenta;
extern const Rgb kWhite;

// Patterns for each state.
// THE single source of truth for what the device shows in flight. Both the live
// loop and the demo call this, so a demo cannot drift from reality and a
// threshold change lands in both at once.
LedPattern flightPattern(uint8_t mode, uint8_t zone, uint8_t landingZone,
                         float altitudeM, bool inAircraft);

LedPattern freefallPattern(uint8_t zone);
LedPattern landingPattern(uint8_t landingZone);
LedPattern faultPattern();
LedPattern offPattern();

// Drive the LED. Call every loop. A pending one-shot flash overrides `p`.
void render(const LedPattern &p, uint32_t nowMs);

// One-shot flash, used for the climb progress marker. Non-blocking.
void flashOnce(const Rgb &c, uint16_t durationMs, uint32_t nowMs);

// Blocking sweeps for bench verification.
void selfTest();
void patternPreview();

}  // namespace led
