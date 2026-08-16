#pragma once
//
// Replays a recorded jump on the screen, exit to landing.
//
// Like demo.cpp, the samples are fed through the *real* ZoneTracker, landing
// ladder and FlightModeTracker — its own instances, so live state is untouched
// — and the pattern comes from led::flightPattern(). So this shows what the
// device actually did on that jump, not a drawing of it, and a change to a
// threshold shows up here against real data.
//
// Unlike demo.cpp the altitude is not synthesised: it is the logged altitude,
// decimated on load. A 3 MB file is read once when REPLAY is pressed and kept
// as a few thousand samples in PSRAM, so playback never touches the card. That
// costs a second or two on opening and buys a replay that cannot stutter when
// the card decides to do housekeeping mid-frame.
//

#include <Arduino.h>

namespace replay {

// Read and decimate a jump file. Returns false if it cannot be opened or holds
// nothing usable. Blocking, a second or two for a full jump.
bool load(const char *path);

// Playback starts at the exit, minus a short lead-in so the first thing you see
// is the aircraft rather than a wall of green.
void start();
void stop();
bool active();

// 1, 2 or 3. Applies to the profile clock only — blink rates stay on the real
// clock, so 3 Hz and 6 Hz look the way they will in the air at any speed.
void setSpeed(uint8_t mult);
uint8_t speed();

// Advance and drive the display. False once the jump has landed.
bool update(uint32_t nowMs);

// Where playback is, for the on-screen scrub line.
uint8_t percent();

}  // namespace replay
