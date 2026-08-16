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
  UI_MENU4,            // page 4: SET CLOCK
  UI_SET_CLOCK,        // the field editor itself
  UI_SET_JUMPNO,       // same editor, four digits, your jump total
  UI_LED_TEST,         // bench only: strip bring-up
  UI_REPLAY,           // a recorded jump, played back
  UI_MENU5,            // page 5: LOGBOOK
  UI_LOGBOOK,          // list of jumps read off the card
  UI_JUMP_DETAIL,      // one jump's numbers
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
  ACT_CONFIRM,
  ACT_CLOCK,           // open the clock editor
  ACT_JUMPNO,          // open the jump-number editor
  ACT_LEDTEST,         // open the LED bring-up screen
  ACT_REPLAY,          // play the jump shown in the detail view
  ACT_RSPD1, ACT_RSPD2, ACT_RSPD3,   // replay speed
  ACT_REPLAY_EXIT,     // back to the stats
  ACT_LOGBOOK,         // open the logbook
  ACT_LOG_ROW0,        // tap a row in the list
  ACT_LOG_ROW1,
  ACT_LOG_ROW2,
  ACT_LOG_PREV,
  ACT_LOG_NEXT,
  ACT_CLK_DOWN,
  ACT_CLK_UP,
  ACT_CLK_NEXT
};

void    setScreen(uint8_t screen);
uint8_t screen();
void    setBanner(const char *line1, const char *line2);

// What a touch at (x,y) means on the screen currently shown.
uint8_t hitTest(int16_t x, int16_t y);

// Clock editor contents: year, month, day, hour, minute, and which of the five
// is being edited. Forces a repaint, so it is safe to call on every change.
void setClockEdit(const int16_t *f5, uint8_t active);
// Four digits, most significant first. Shares the clock editor's buttons and
// its ACT_CLK_* actions — same gesture, so there is one thing to learn.
void setJumpEdit(const int16_t *d4, uint8_t active);
// Bench LED bring-up: pattern name, brightness, and which of the two is being
// edited. Shares the clock editor's buttons.
void setLedTest(const char *name, uint8_t bright, uint8_t active);

// One page of the logbook. `rows` are three pre-formatted lines, empty for a
// slot with nothing in it; `page`/`pages` drive the footer.
void setLogbookPage(const char rows[3][32], uint8_t page, uint8_t pages,
                    uint16_t total);
// The detail view: a title and five pre-formatted lines. Formatting stays in
// main.cpp next to the data rather than being reinvented in the renderer.
void setJumpDetail(const char *title, const char lines[5][32]);

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

// EVE-style boot greeting: two blue eyes fade up on black, blink, then curve
// into a smile. Blocking, ~2.3 s, and like message() only safe before
// startTask(). Compiled out by BOOT_FACE_ENABLED.
void bootFace();

// Fade the held smile out and release the cell. bootFace() deliberately stops
// with the smile on screen so the boot zero runs under it rather than over a
// black screen; this ends the sequence.
void bootFaceOut();

// The going-to-sleep face: eyes fade up, blink twice slowly, then close for
// good and fade out. Blocking, ~3.5 s. Unlike bootFace() this is safe with the
// renderer running — it suspends the task first — and it leaves the panel dark,
// so call display::sleep() straight after.
void sleepFace();


// Two-line boot banner, faded in and out around whatever happens between them.
// Blocking, same pre-startTask() rule as message(). bannerLine1() repaints line
// one at full brightness for the progress dots — pad it to a constant width, or
// a shrinking string leaves its own tail on screen.
void bannerIn(const char *line1, const char *line2);
void bannerLine1(const char *line1);
void bannerLine2In(const char *line2);
void bannerOut();

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
