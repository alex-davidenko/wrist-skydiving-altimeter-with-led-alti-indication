#pragma once
//
// Minimal AXS5106L capacitive touch reader with tap/swipe gestures.
//
// Shares the I2C bus with the MS5611 — touch answers at 0x63, the barometer at
// 0x77, so there is no conflict and no extra wiring. Protocol is small enough
// not to need Waveshare's ESP-IDF driver: reset the controller, then read 14
// bytes from register 0x01. data[1] is the touch count, followed by 12-bit
// X/Y pairs.
//
// GESTURES. The controller has no notion of press/release — it simply keeps
// reporting while a finger is down, firing the INT line each time. So a
// "release" is inferred from the reports stopping. A gesture accumulates the
// first and last positions of a contact, and is classified once the reports
// go quiet: mostly-horizontal movement is a swipe, anything else is a tap at
// the point where the finger landed.
//

#include <Arduino.h>

namespace touch {

enum EventType : uint8_t
{
  EV_NONE = 0,
  EV_TAP,
  EV_SWIPE_LEFT,     // finger moved right -> left
  EV_SWIPE_RIGHT
};

struct Event
{
  EventType type;
  int16_t   x, y;    // where the contact started, in display coordinates
};

// Returns false if the controller does not answer; everything else no-ops.
bool begin();
bool available();

// Completed gesture since the last call, or EV_NONE. Consumes the event, so a
// gesture is reported exactly once. Call frequently — release detection is
// driven from this.
Event takeEvent();

// Raw controller coordinates of the last contact, for calibration.
void rawLast(int16_t *x, int16_t *y);

}  // namespace touch
