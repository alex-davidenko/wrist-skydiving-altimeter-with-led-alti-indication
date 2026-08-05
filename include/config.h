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
// The altitude uses baked typefaces from tools/make_font.py, picked at runtime
// by digit count — see renderFrame(). Nothing to configure here.
// Vertical speed sits in a strip along the TOP, so the altitude digits own
// everything below it and cannot be clipped by the readout.
#define ALT_TOP_BAND       26
#define DISPLAY_PERIOD_MS  50   // 20 Hz content redraw
// Backlight blink resolution. The task ticks this fast so blink edges are
// crisp; only the GPIO is touched at this rate, not the panel.
#define DISPLAY_TICK_MS     5

// The renderer runs pinned to the core the Arduino loop does not use, because
// a full-screen fill measured 23.7 ms against a 25 ms sample period.
#define DISPLAY_TASK_CORE  0
#define DISPLAY_TASK_STACK 6144
#define DISPLAY_TASK_PRIO  1

// ---- AXS5106L capacitive touch -------------------------------------------
// Shares the I2C bus with the barometer (touch 0x63, MS5611 0x77).
// Pin numbers confirmed from Waveshare's own Arduino examples.
#define TOUCH_ENABLED     1
#define PIN_TOUCH_RST    47
#define PIN_TOUCH_INT    48

// Display geometry at DISPLAY_ROTATION 1 (landscape).
#define DISPLAY_W       320
#define DISPLAY_H       172

// The controller reports in the panel's native 172x320 portrait frame, so the
// axes need remapping — and which way round depends on how the panel is
// mounted, which no datasheet states. Run `T` on the console, tap the corners,
// and set these from what it prints.
// Measured on hardware with labelled corners: top-left raw (38,9),
// bottom-right raw (149,313). These settings map those to (9,38) and
// (313,149) — both ascending, so the corners come out the right way round.
// The inset is just not tapping the literal edge pixels.
#define TOUCH_SWAP_XY     1
#define TOUCH_FLIP_X      0
#define TOUCH_FLIP_Y      0
// The controller has no press/release notion — it just keeps reporting while a
// finger is down (~20 reports per tap). A release is inferred from the reports
// stopping for this long, at which point the contact is classified.
#define TOUCH_RELEASE_MS    120
// Horizontal travel needed to count as a swipe rather than a tap.
#define TOUCH_SWIPE_MIN_PX   60

// ---- Automatic ground-reference correction (weather drift) ---------------
// Mirrors what production kit does: CYPRES re-measures ground pressure twice a
// minute, the Vigil recalibrates every two minutes "progressively". Leaving the
// reference fixed is the actual error — uncorrected drift is tens of metres
// over a jumping day.
//
// The settle time scales with altitude, per Alti-2's published behaviour
// (~10 min at 500 ft, "several hours" at 12,000 ft), which is what makes an
// aircraft holding at altitude safe: 4 min at 25 m, but 3.3 hours at 4000 m.
// FLIGHT ONLY. The constants below are absolute metres, which are negligible
// against 300 m flight bands but enormous against bench ones: a 1 m settle band
// spans three bench zones, and a 0.5 m/min slew walks a board held at 1.35 m
// down through red, blinking red and off in three minutes without it moving.
// That looks exactly like a bug while bench testing. Test auto-zero with a
// flight build sitting on a desk, where altitude is ~0 and it does the right
// thing.
#define AUTOZERO_ENABLED         (!BENCH_MODE)
#define AUTOZERO_SETTLED_MPS     0.5f
// 1 m band, so a walk upstairs restarts the timer rather than being absorbed.
#define AUTOZERO_BAND_M          1.0f
#define AUTOZERO_BASE_MIN        3.0f
#define AUTOZERO_PER_M_MIN       0.05f
// 0.5 m/min is 17x faster than real drift needs (~0.03 m/min), and slow enough
// that a five-minute excursion moves the reference under a metre.
#define AUTOZERO_SLEW_M_PER_MIN  0.5f
// Independent guard: any sustained motion latches "in flight" and disables
// correction until the device is back low and still.
#define AUTOZERO_LATCH_VS_MPS    1.0f
#define AUTOZERO_LATCH_ALT_M     150.0f
#define AUTOZERO_LATCH_CLEAR_MS  120000

// ---- Idle light sleep -----------------------------------------------------
// The big power lever: ~35 mA of MCU plus ~25 mA of backlight, duty-cycled down
// to a wake every 30 s. CYPRES does the same thing at the same interval.
//
// THE CONDITION IS "ON THE GROUND", NOT "NOT CLIMBING". An aircraft holding at
// 4000 m for clearance has ~zero vertical speed and looks identical to sitting
// on the ground — sleeping there would mean up to 30 s of blindness, which at
// 50 m/s is 1500 m of missed freefall. So sleep requires a low altitude as well
// as stillness, plus the same in-flight latch the drift correction uses.
//
// Climbing at 5 m/s clears 25 m in five seconds, so a 30 s wake still catches
// the climb by ~150 m, long before anything depends on it.
// Enabled in BOTH builds. Unlike auto-zero, nothing here is scale-sensitive —
// the 25 m ceiling is simply always satisfied on a desk — and the bench is
// where the current draw actually gets measured. It only sleeps after 60 s of
// stillness off USB, so it does not interrupt hand testing.
#define IDLE_SLEEP_ENABLED       1
#define IDLE_WAKE_PERIOD_S       30
#define IDLE_MAX_ALT_M           25.0f    // above this, never sleep
#define IDLE_MAX_VSPEED_MPS      0.5f
#define IDLE_QUIET_BEFORE_MS     60000    // must be idle this long before first sleep
#define IDLE_WAKE_DISPLAY_MS     20000    // a button wake shows the screen this long

// ---- Automatic power off --------------------------------------------------
// Hygiene rather than a power measure — with idle sleep the cell lasts weeks.
// This is for "I left it in my gear bag". Deferred while moving, and warned.
#define AUTO_OFF_HOURS           16
#define AUTO_OFF_WARN_MS         30000

// ---- Display units --------------------------------------------------------
// Conversion happens ONLY at render time. Everything internal — filter, zones,
// thresholds, logs — stays in metres, so switching units cannot disturb any
// tuning constant, any test, or the meaning of a log file.
//
// Feet are rounded to 10, which is what production altimeters do: at 50 m/s the
// foot digit changes 164 times a second and is unreadable.
#define UNITS_FEET_DEFAULT   0        // 0 = metres, 1 = feet
#define METRES_TO_FEET       3.28084f   // also m/s -> ft/s
// Hysteresis on the displayed integer, as a fraction of one display step (1 m,
// or 10 ft). A reading parked on a rounding boundary flips the number back and
// forth on noise alone — the zone tracker has had hysteresis since early on for
// exactly this reason, and the number never did. Costs nothing in accuracy.
#define ALT_DISPLAY_HYST     0.15f

// ---- Battery sense --------------------------------------------------------
// GPIO12 through a 3:1 divider, per Waveshare's own battery example.
// analogReadMilliVolts() is factory-calibrated, so the only scaling needed is
// the divider ratio.
#define PIN_BATTERY        12
#define BATTERY_DIVIDER    3.0f
#define BATTERY_PERIOD_MS  1000   // it does not move fast
// The ADC is noisy sample to sample; smooth it so the readout is not twitchy.
#define BATTERY_EMA        0.2f
// Decimals shown on screen. 3 drops the "V" to keep the same 5-character width,
// and exists because a true battery reading can only be taken with USB
// unplugged — where the console is unavailable, so the screen is the only
// instrument. Set back to 2 (which restores the "V") once the power figures are
// settled.
#define BATTERY_SCREEN_DECIMALS  3

// ---- microSD (SDMMC, 4-bit) ----------------------------------------------
// Cards must be FAT32 with an MBR partition scheme. ESP-IDF returns
// FR_NO_FILESYSTEM on exFAT or GPT, which is what macOS defaults to for cards
// larger than 32 GB.
#define LOGGER_ENABLED    1
#define PIN_SD_CLK       16
#define PIN_SD_CMD       15
#define PIN_SD_D0        17
#define PIN_SD_D1        18
#define PIN_SD_D2        13
#define PIN_SD_D3        14

// Ring buffer sits in PSRAM. 28 bytes/record at 40 Hz is ~1.1 KB/s, so 32k
// records is roughly 13 minutes of slack — far more than any card stall.
#define LOG_RING_RECORDS  32768
#define LOG_FLUSH_MS      1000   // crash costs at most this much data
#define LOG_TICK_MS       20
#define LOG_TASK_CORE     0      // same core as the display; neither is timed
#define LOG_TASK_STACK    4096
#define LOG_TASK_PRIO     1

// The GY-63's mode straps. Set a pin to -1 when that strap is wired to a rail
// in hardware instead; the firmware then leaves the pin alone entirely.
//
// PS is hard-wired to 3V3. It selects I2C vs SPI and never changes at runtime,
// so driving it from a GPIO bought nothing but a window during MCU boot where
// the pin was an undriven input. A rail is definitive from the instant power is
// applied, and it is one fewer joint to fatigue under opening shock.
//
// CSB is still driven, LOW for address 0x77. Wire it to GND and set this to -1
// as well if you want the sensor down to four wires — but only if you actually
// wire it: floating CSB leaves the address undefined.
#define PIN_SENSOR_PS    -1    // wired to 3V3 = I2C mode
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
// Demo jump playback speed. 1.0 = real time, so the ~3.5 minute profile plays
// out exactly as it would in the air — which is the point: it shows how fast
// the altitude actually changes and whether that is readable.
#define DEMO_SPEED           1.0f

// Menu closes itself so it can never sit over the altitude during a jump.
#define MENU_TIMEOUT_MS      15000
// Power-off is refused above this vertical speed. Shutting down mid-jump is
// the one failure mode worth designing out.
#define SLEEP_MAX_VSPEED_MPS 2.0f

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

// Unbuckle reminder during the climb. 500-600 m at ~5 m/s is a 20 s window.
#define CLIMB_UNBUCKLE_LO_M     500.0f
#define CLIMB_UNBUCKLE_HI_M     600.0f
#define CLIMB_UNBUCKLE_BLINK_MS   700

// Once a climb passes this altitude the display stays in aircraft mode (blue,
// altitude shown) until freefall begins. Without the latch, levelling off on
// jump run drops vertical speed below the climb threshold, the phase machine
// calls it CANOPY, and the screen blanks moments before exit.
#define AIRCRAFT_LATCH_ALT_M    300.0f
#define AIRCRAFT_CLEAR_ALT_M    100.0f

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
// The flight ladders are defined unconditionally rather than inside the
// BENCH_MODE branch, because the demo jump replays a 4200 m profile and has to
// show flight behaviour even from a bench build — against metre-scale bench
// bands it would sit in a single colour the whole way down.
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
static const ZoneConfig kFlightZoneConfig = {
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
static const ZoneConfig kZoneConfig = kFlightZoneConfig;
#endif
