#pragma once
//
// 172x320 JD9853 panel on the Waveshare ESP32-S3-Touch-LCD-1.47.
//
// The whole point of this screen for a skydiving altimeter is that a
// full-screen colour fill is enormously more visible in daylight than any
// single LED. So the zone colour is the *background*, and the altitude fills
// the screen on top of it. You read the colour from across a room and the
// number when you look directly at it.
//
// THREADING. The panel renders on its own FreeRTOS task pinned to the other
// core. A full-screen fill measured 23.7 ms against a 25 ms sample period, so
// drawing from the sample loop cost one dropped sample per zone change — and
// would have cost ~28% of the loop once the screen started blinking at 6 Hz.
// Rendering on the second core decouples them completely: the sample loop
// calls publish(), which is a spinlock-protected struct copy, and never
// touches SPI.
//
// Compiled out entirely on boards without a panel.
//

#include <Arduino.h>

#include "led.h"

namespace display {

// Bring up the panel. Call from setup(), before startTask().
// Returns false if the panel did not initialise; everything else then no-ops.
bool begin();

// Direct, blocking text render. Only safe before startTask() — used for boot
// and for the sensor-fault dead end.
void message(const char *line1, const char *line2);

// Spawn the renderer on the other core.
void startTask();

// Hand the renderer a new state. Cheap and safe to call from the sample loop
// every iteration; the task paces its own redraws.
void publish(const LedPattern &pattern, float altitudeM, float vspeedMps);

// Microseconds taken by the last full-screen fill — the expensive operation,
// worth reporting rather than guessing at.
uint32_t lastFillUs();

bool available();

}  // namespace display
