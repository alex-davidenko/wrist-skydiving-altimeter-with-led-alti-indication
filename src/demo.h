#pragma once
//
// Plays a complete jump on the screen so the whole display sequence can be
// seen on the ground.
//
// The profile is fed through the *real* ZoneTracker, landing ladder and
// FlightModeTracker — its own instances, so live state is untouched — and the
// pattern is built exactly the way the flying firmware builds it. So this
// demonstrates the actual logic rather than an animation of what it is
// supposed to look like, and a bug in the zone thresholds would show up here.
//
// Altitude runs on a compressed clock (DEMO_SPEED) because the real profile is
// nearly four minutes. Blink rates deliberately stay on the real clock, so
// 3 Hz and 6 Hz look exactly as they will in the air.
//

#include <Arduino.h>

namespace demo {

void start();
void stop();
bool active();

// Advance the profile and drive the display. Returns false once finished.
bool update(uint32_t nowMs);

// Seconds of profile remaining, for the on-screen countdown.
int secondsLeft();

}  // namespace demo
