#pragma once
//
// All the knobs live here. Nothing below this file should need editing for
// normal bring-up and tuning.
//

#include "zones.h"

// ===========================================================================
//  MODE
// ===========================================================================
// 1 = BENCH: thresholds scaled to desk height (metres instead of kilometres)
//            so the zone logic can be exercised by lifting the board by hand.
// 0 = FLIGHT: the real skydiving thresholds.
// Override from the command line with: pio run -D BENCH_MODE=0
#ifndef BENCH_MODE
#define BENCH_MODE 1
#endif

// ===========================================================================
//  LED DRIVER SELECTION
// ===========================================================================
#define LED_DRIVER_NEOPIXEL   0   // WS2812 / onboard RGB, one data pin
#define LED_DRIVER_RGB_CC     1   // discrete RGB LED, common cathode  (to GND)
#define LED_DRIVER_RGB_CA     2   // discrete RGB LED, common anode    (to 3V3)
#define LED_DRIVER_NONE       3   // no LED at all — serial/console only

// ===========================================================================
//  PINS  (Waveshare ESP32-S3-Touch-LCD-1.47)
// ===========================================================================
#define I2C_FREQ_HZ   400000

// GPIO41/42 are this board's I2C bus. It is shared with the capacitive touch
// controller and is broken out on the right-hand header, which is exactly
// where the GY-63 wants to go — no extra pins needed.
//
// NOTE the touch panel already has pull-ups on this bus and the GY-63 brings
// its own 4.7k pair. In parallel that is ~2.4k, which sinks ~1.4 mA at 3.3 V —
// inside the I2C spec's 3 mA, so it is fine, but it is why you should not add
// any more. If the bus ever misbehaves, lifting the GY-63's pull-ups is the
// first thing to try.
#define PIN_I2C_SDA   42
#define PIN_I2C_SCL   41

// No user-controllable RGB LED on this board — GPIO38 is LCD_CLK here, unlike
// the non-touch ESP32-S3-LCD-1.47. Default to an external WS2812 on a free
// header pin; driving an unconnected pin is harmless if you have not wired one.
// Set LED_DRIVER_NONE if you would rather it stayed quiet.
#define LED_DRIVER    LED_DRIVER_NEOPIXEL
#define PIN_LED       7

#define PIN_LED_R     1
#define PIN_LED_G     2
#define PIN_LED_B     3

// BOOT button on the S3.
#define PIN_BUTTON    0
#define BUTTON_ACTIVE_LOW 1

// ---- 1.47" JD9853 panel -------------------------------------------------
#define DISPLAY_ENABLED   1
#define PIN_LCD_SCK      38
#define PIN_LCD_MOSI     39
#define PIN_LCD_CS       21
#define PIN_LCD_DC       45
#define PIN_LCD_RST      40
#define PIN_LCD_BL       46
// 0/2 = portrait 172x320, 1/3 = landscape 320x172. Landscape gives the
// altitude number far more room, so it is the default.
#define DISPLAY_ROTATION  1
// The altitude glyph size is chosen at runtime as the largest that fits the
// digit count, capped here. Base font is 5x7, so size N is 6N px per digit
// advance and 8N px tall.
#define ALT_TEXT_SIZE_MAX  17
#define ALT_BOTTOM_BAND    26   // px reserved at the bottom for vertical speed
#define DISPLAY_PERIOD_MS  50   // 20 Hz render; free now that it has its own core

// The renderer runs pinned to the core the Arduino loop does not use, because
// a full-screen fill measured 23.7 ms against a 25 ms sample period.
#define DISPLAY_TASK_CORE  0
#define DISPLAY_TASK_STACK 6144
#define DISPLAY_TASK_PRIO  1

// Optional: drive the GY-63's mode straps from GPIOs instead of wiring them to
// rails. Costs two pins and buys certainty — the firmware then guarantees I2C
// mode and a known address regardless of what the breakout's own pull-ups do.
// Set either to -1 to leave that pin unconnected and strap it in hardware.
#define PIN_SENSOR_PS     9    // held HIGH  -> I2C (LOW would select SPI)
#define PIN_SENSOR_CSB   11    // LOW -> address 0x77, HIGH -> 0x76
#define SENSOR_CSB_LEVEL  0    // 0 = LOW = 0x77


// 0-255. The onboard WS2812 is retina-searing indoors; turn this up when you
// move to an external high-power LED for daylight use.
#define LED_BRIGHTNESS   40

// Blinking red is the "below 800 m" warning — the most urgent thing this
// device says. 167 ms = 6 blinks/s, matching the fastest landing rate.
#define BLINK_PERIOD_MS  167   // blinking-red cadence (full on+off cycle)
// The fault pattern must stay distinguishable from blinking red, so it is now
// separated by colour (magenta vs red) rather than by rate.
#define FAULT_PERIOD_MS  200   // sensor-fault cadence

// ===========================================================================
//  SENSOR / TIMING
// ===========================================================================
// The MS5611 library's read() is blocking and performs a pressure conversion
// followed by a temperature conversion. At OSR_ULTRA_HIGH each takes ~10 ms,
// so one read() is ~20 ms and 40 Hz (25 ms) is about the practical ceiling.
// Drop to OSR_HIGH (~10 ms total) if you want more loop headroom at the cost
// of roughly 1.5x the pressure noise.
#define SENSOR_OSR          OSR_ULTRA_HIGH
#define SAMPLE_PERIOD_MS    25      // 40 Hz

// Serial CSV is decoupled from sampling so logging never gates the loop.
#define CSV_PERIOD_MS       100     // 10 Hz

// I2C address. GY-63: CSB open/GND -> 0x77, CSB to VCC -> 0x76.
// Startup probes 0x77 then 0x76, so this is just the first guess.
#define MS5611_ADDR_PRIMARY   0x77
#define MS5611_ADDR_SECONDARY 0x76

// Pressure compensation scaling.
//
//   0 = MS5611 datasheet maths (SENSt1 = C1*2^15, OFFt1 = C2*2^16)
//   1 = MS5607 / app-note maths (SENSt1 = C1*2^16, OFFt1 = C2*2^17)
//
// The two differ by exactly a factor of two on the dominant pressure terms,
// so the wrong choice gives a pressure that is exactly half (or double) the
// real one. Many boards sold as "GY-63 MS5611" carry an MS5607 or a clone die
// and need mode 1. If the boot plausibility check complains, flip this.
#define SENSOR_MATH_MODE    1

// Consecutive failed reads before the LED goes to the fault pattern.
#define SENSOR_FAIL_LIMIT   10

// ===========================================================================
//  GROUND ZERO / CALIBRATION
// ===========================================================================
#define ZERO_SAMPLE_COUNT     80     // ~2 s at 40 Hz

// Two separate acceptance checks, because scatter and movement are different
// problems and peak-to-peak spread cannot tell them apart.
//
// Do NOT go back to a peak-to-peak test. Peak-to-peak grows with sample count
// and is dominated by tails: with the MS5611's real ~0.22 m single-sample
// sigma, 80 samples span 0.9-1.3 m as a matter of course. A peak-to-peak limit
// tight enough to catch a moving board rejects every good calibration too.
//
// sigma : single-sample scatter. Catches a failing sensor or a gale, and is
//         set generously because normal scatter is large and harmless — it
//         averages out.
#define ZERO_MAX_SIGMA_M      (BENCH_MODE ? 0.70f : 2.00f)
// drift : difference between the first- and second-half means. This is the
//         one that actually catches a board being moved mid-average, because
//         noise cancels in a mean but a trend does not. It can be tight: the
//         standard error of each half-mean is only ~0.035 m at bench scale.
#define ZERO_MAX_DRIFT_M      (BENCH_MODE ? 0.20f : 1.00f)
// Long-press duration on the BOOT button to trigger a re-zero.
#define ZERO_BUTTON_HOLD_MS   1500

// ===========================================================================
//  FILTER TUNING
// ===========================================================================
#if BENCH_MODE
  // Desk testing: nothing moves fast, and 0.1 m thresholds need the noise
  // pushed well below the 5 cm hysteresis band. Trade response for smoothness.
  #define FILTER_SIGMA_ACCEL  0.05f   // m/s^2
  #define FILTER_SIGMA_MEAS   0.25f   // m
#else
  // Freefall: must track a ~50 m/s ramp and a multi-g canopy opening.
  #define FILTER_SIGMA_ACCEL  3.0f    // m/s^2
  #define FILTER_SIGMA_MEAS   0.60f   // m
#endif

// Innovation-bias gate, in sigmas, for the adaptive process noise.
// Measured on simulated profiles (see test/test_core):
//   gate off -> canopy-opening peak error 3.5 m, 4 s to settle on the bench
//   gate 2.0 -> canopy-opening peak error 0.8 m, 0.12 s to settle
// Lower = re-locks faster after acceleration, smooths slightly less.
// Set <= 0 for a plain constant-velocity Kalman.
#define FILTER_GATE_SIGMA   2.0f
#define FILTER_MAX_INFLATE  500.0f

// ===========================================================================
//  FLIGHT PHASE  (FLIGHT mode only — bench keeps the plain colour ladder)
// ===========================================================================
// Below this descent rate you are under a working canopy, and the freefall
// colour zones stop meaning anything useful. Above it you are in freefall or
// riding a high-speed malfunction — which is exactly when those colours are
// still correct, so a mal keeps the red/blinking-red warnings.
//
// CAVEAT: a hard spiral under a small canopy can genuinely exceed 20 m/s and
// will flip the display back to freefall colours. If you spiral hard, raise
// this. Enter/exit differ to stop a descent sitting near the threshold from
// flapping the entire meaning of the LED.
#define MODE_FREEFALL_ENTER_MPS   20.0f
#define MODE_FREEFALL_EXIT_MPS    15.0f

// Aircraft climb is ~5 m/s; 2 m/s enters, 0.5 m/s holds.
#define MODE_CLIMB_ENTER_MPS       2.0f
#define MODE_CLIMB_EXIT_MPS        0.5f

// A mode change swaps the whole meaning of the LED, so it is damped harder
// than a band change within a mode.
#define MODE_DWELL_MS              400

// Climb progress: one green flash per this much altitude gained.
#define CLIMB_MARK_INTERVAL_M    100.0f
#define CLIMB_FLASH_MS             150

// Landing ladder blink periods (full on+off cycle).
#define LANDING_SLOW_PERIOD_MS     333   // 3 blinks/s
#define LANDING_FAST_PERIOD_MS     167   // 6 blinks/s

// ===========================================================================
//  ZONE THRESHOLDS
// ===========================================================================
// Order matches the Zone enum: bounds[i] separates zone i from zone i+1.
//   OFF | BLINK_RED | RED | YELLOW | GREEN | ABOVE
#if BENCH_MODE
// The flight thresholds divided by 1000, plus an LED-off band near the desk.
//
// Caveat worth knowing: at this scale the bands are 0.3-0.7 m wide while the
// MS5611's raw noise is ~0.25 m, so bench testing runs close to the sensor's
// physical floor. The numbers below were picked by simulation (see
// test/test_core) as the point where the LED never chatters while parked
// — worst case 6.8 s between changes over 300 s at each of 81 heights — and
// every zone is still reachable and stable from both directions.
//
// The LED-off boundary is 0.15 rather than 0.10 so that, with hysteresis, it
// actually commits to off below 0.08 m and relights above 0.22 m. A boundary
// at exactly 0.10 would need the board *below* its zero height to go dark.
//
// Test at the middle of a band, not on a boundary: hysteresis makes a height
// sitting exactly on a threshold depend on which side you approached from.
// Good test heights are 0.05 / 0.50 / 0.95 / 1.35 / 2.00 m.
// Bench keeps SYMMETRIC damping, deliberately. These exact numbers were
// validated on hardware and the asymmetric urgency bias below is a flight
// safety behaviour, not a bench one — at desk scale it would just make the
// reading sit one band low near a boundary and confuse the test.
//
// On dwell: 300 ms is just short of a cliff. Measured min gap between zone
// changes while parked: 87 s at 300 ms, but only 1.6 s at 250 ms. Going below
// 300 buys ~50 ms of response and reintroduces visible chatter.
static const ZoneConfig kZoneConfig = {
    // off/blink  blink/red  red/yel  yel/grn  grn/above
    {   0.15f,     0.80f,     1.20f,   1.50f,   4.50f },
    0.07f, 0.07f,   // hysteresis urgent / relax, m
    300,   300      // dwell urgent / relax, ms
};
#else
// Flight uses ASYMMETRIC damping. Dropping a zone means danger increasing and
// commits almost immediately; climbing back is damped hard. Measured on a
// 50 m/s descent through the 800 m threshold:
//   symmetric  (5 m / 120 ms both ways) -> LED lights 11.2 m late (225 ms)
//   asymmetric (0 m / 50 ms urgent)     -> LED lights  ~2-3 m late
// The urgent dwell is 50 ms rather than 25 ms because 25 ms false-alarms:
// injecting one 30 m bad read every 50 s while hovering above a threshold
// produced 24 spurious blink-reds in 20 minutes at 25 ms, and zero at 50 ms.
// A false blink-red is a safe failure, but false alarms destroy trust in the
// indicator, which is its own safety problem.
static const ZoneConfig kZoneConfig = {
    // The LED-off band is disabled in flight: a very negative first boundary
    // means ZONE_OFF is unreachable, so the LED keeps blinking red all the way
    // down. Under canopy the landing ladder takes over anyway.
    {  kZoneBoundDisabledLow, 800.0f, 1200.0f, 1500.0f, 4500.0f },
    0.0f, 8.0f,   // hysteresis urgent / relax, m
    50,   400     // dwell urgent / relax, ms
};

// Landing ladder — only shown under canopy (see FlightModeTracker).
//   > 300 m : dark, nothing to do yet
//   200-300 : 3 blinks/s green
//   100-200 : 6 blinks/s green
//    10-100 : bright steady green
//   <  10 m : dark, assistance finished
static const ZoneConfig kLandingConfig = {
    //  dark/steady  steady/fast  fast/slow  slow/dark   (top disabled)
    {   10.0f,       100.0f,      200.0f,    300.0f,     kZoneBoundDisabledHigh },
    2.0f, 8.0f,   // hysteresis urgent / relax, m
    100,  400     // dwell urgent / relax, ms
};
#endif
