#pragma once
//
// Minimal AXS5106L capacitive touch reader.
//
// Shares the I2C bus with the MS5611 — touch answers at 0x63, the barometer at
// 0x77, so there is no conflict and no extra wiring. Protocol is small enough
// not to need Waveshare's ESP-IDF driver: reset the controller, then read 14
// bytes from register 0x01. data[1] is the touch count, followed by 12-bit
// X/Y pairs.
//
// Touch-down is detected from the INT line's falling edge, the same way
// Waveshare's driver does it, so we are not polling a bus the sample loop also
// needs.
//

#include <Arduino.h>

namespace touch {

struct Point
{
  int16_t x, y;      // already mapped into display coordinates
  bool    valid;
};

// Returns false if the controller does not answer; everything else no-ops.
bool begin();
bool available();

// Most recent touch-down since the last call, or {invalid} if none.
// Consumes the event, so a single tap is reported exactly once.
Point takeTouch();

// Raw controller coordinates of the last touch, for calibration. The mapping
// from these to display coordinates cannot be derived from the datasheet alone
// — it depends on how the panel is mounted — so `T` on the console prints both
// and TOUCH_SWAP_XY / TOUCH_FLIP_X / TOUCH_FLIP_Y in config.h correct it.
void rawLast(int16_t *x, int16_t *y);

}  // namespace touch
