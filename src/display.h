#pragma once
//
// 172x320 JD9853 panel on the Waveshare ESP32-S3-Touch-LCD-1.47.
//
// The whole point of this screen for a skydiving altimeter is that a
// full-screen colour fill is enormously more visible in daylight than any
// single LED. So the zone colour is the *background*, and the altitude number
// sits on top of it. You read the colour from across a room and the number when
// you look directly at it.
//
// Compiled out entirely on boards without a panel.
//

#include <Arduino.h>

namespace display {

// Returns false if the panel did not initialise; everything else then no-ops.
bool begin();

// Boot/status text while there is nothing to plot yet.
void message(const char *line1, const char *line2);

// Draw the current state. Cheap when nothing changed — it only touches the
// pixels that actually differ.
void update(float altitudeM, float vspeedMps, uint8_t zone,
            const char *label, bool fault, uint32_t nowMs);

// Microseconds taken by the last full-screen fill. Full fills are the
// expensive operation on this panel and the reason the display is throttled;
// exposed so `s` can report it rather than us guessing.
uint32_t lastFillUs();

bool available();

}  // namespace display
