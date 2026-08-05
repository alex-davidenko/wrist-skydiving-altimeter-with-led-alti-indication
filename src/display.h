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

// On-screen UI. Geometry lives in display.cpp next to the rendering, and
// hitTest() uses the same rectangles, so the buttons and the touch targets
// cannot drift apart.
//
// Every target is kept inside x 20..300, y 45..150. Corner taps measured on
// hardware only reach about x 15..302, y 34..152 — a button against an edge
// would be unreachable.
enum UiScreen : uint8_t
{
  UI_ALT = 0,          // normal altitude display
  UI_MENU,             // page 1: ZERO / POWER OFF
  UI_MENU2,            // page 2: UNMOUNT CARD / DEMO JUMP
  UI_MENU3,            // page 3: USB DRIVE
  UI_CONFIRM_ZERO,
  UI_CONFIRM_UNMOUNT,
  UI_CONFIRM_USB,
  UI_CONFIRM_POWER,
  UI_BANNER            // plain two-line message
};

enum UiAction : uint8_t
{
  ACT_NONE = 0,
  ACT_ZERO,
  ACT_UNMOUNT,
  ACT_DEMO,
  ACT_USB,
  ACT_UNITS,
  ACT_POWER,
  ACT_CANCEL,
  ACT_CONFIRM
};

void    setScreen(uint8_t screen);
uint8_t screen();
void    setBanner(const char *line1, const char *line2);

// What a touch at (x,y) means on the screen currently shown.
uint8_t hitTest(int16_t x, int16_t y);

// Blank the panel and put the controller to sleep, before deep or light sleep.
void sleep();

// Bring the panel back after a light sleep. The controller keeps its registers
// (it is separately powered), so this is sleep-out plus display-on rather than
// a full re-initialisation.
void wake();

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

// Battery volts, shown top-right opposite the vertical speed. Updated slowly
// and independently, so the demo does not have to know about it.
void setBattery(float volts);

// Text for the top strip, replacing the vertical-speed readout. Empty restores
// the speed. Used for the UNBUCKLE reminder during the climb.
void setTopText(const char *text);

// Display units. Affects rendering only — nothing internal changes.
void setUnitsFeet(bool feet);
bool unitsFeet();

// Microseconds taken by the last full-screen fill — the expensive operation,
// worth reporting rather than guessing at.
uint32_t lastFillUs();

bool available();

}  // namespace display
