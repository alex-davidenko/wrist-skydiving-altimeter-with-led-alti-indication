// ===========================================================================
//  Wrist-mounted skydiving altimeter — firmware (see FW_VERSION in config.h)
//
//  MS5611 (GY-63) over I2C -> ISA pressure altitude -> Kalman filter ->
//  hysteresis/dwell zone state machine -> RGB LED.
//
//  NOT A SAFETY DEVICE. This is a secondary visual aid built on a single
//  uncertified sensor with no redundancy. Jump with your normal analogue
//  and audible altimeters; this one is for cross-checking on the ground and
//  for fun in the air.
// ===========================================================================

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>

#include <driver/gpio.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <sys/time.h>
#include <time.h>

#include <MS5611.h>

#include "altitude_filter.h"
#include "baro_math.h"
#include "config.h"
#include "demo.h"
#include "display.h"
#include "flight_mode.h"
#include "ground_ref.h"
#include "led.h"
#include "logger.h"
#include "touch.h"
#include "usb_msc.h"
#include "zones.h"

// ---------------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------------
static MS5611        g_sensor(MS5611_ADDR_PRIMARY);
static AltitudeFilter g_filter(FILTER_SIGMA_ACCEL, FILTER_SIGMA_MEAS);
static ZoneTracker   g_zones;      // freefall colour ladder
#if !BENCH_MODE
static ZoneTracker        g_landing; // landing ladder, used under canopy
static FlightModeTracker  g_modes;
static ClimbMarker        g_climb;
#endif
#if AUTOZERO_ENABLED
static GroundRef     g_groundRef;
#endif
static Preferences   g_prefs;

static float    g_qnhHpa      = baro::kISASeaLevel_hPa;
static float    g_groundPHpa  = 0.0f;    // 0 => not calibrated
static bool     g_calibrated  = false;

static float    g_pressureHpa = 0.0f;
static float    g_tempC       = 0.0f;
static float    g_rawAglM     = 0.0f;

static uint32_t g_nextSampleMs = 0;
static uint32_t g_nextCsvMs    = 0;
static uint32_t g_lastSampleUs = 0;
static uint16_t g_failStreak   = 0;
static uint32_t g_overruns     = 0;
static bool     g_csvEnabled   = true;
static bool     g_touchDump    = false;
static float    g_battV        = 0.0f;
static uint32_t g_nextBattMs   = 0;
static const char *g_resetReason = "?";
static uint32_t g_autoOffWarnMs = 0;
static uint32_t g_activeSinceMs = 0;    // last time we were NOT idle
static uint32_t g_wakeShowUntil = 0;    // button wake keeps the screen up
// The press that wakes the device from light sleep is still physically down
// when the loop resumes, so pollButton sees it as a fresh press: opening the
// menu and restarting the 60 s quiet timer, which overrode the 20 s wake window
// entirely. Traced: g_activeSinceMs was being set 1.1 s after each wake.
//
// This is a deadline rather than a pin read. The first attempt armed the
// swallow from digitalRead() at wake time, which does not reflect the button
// after an EXT0 light sleep — the pin is an RTC GPIO at that moment — so it
// never armed and the release went through anyway.
static uint32_t g_btnSwallowUntil = 0;
static uint32_t g_filterSettledMs = 0;
// Which site last restarted the quiet timer, recorded at the point of
// assignment. Reading the source has been wrong four times about this.
static const char *g_activeWho = "boot";
#define MARK_ACTIVE(t, who) do { g_activeSinceMs = (t); g_activeWho = (who); } while (0)
#if !BENCH_MODE
// Latched once a climb passes AIRCRAFT_LATCH_ALT_M. Without it, levelling off
// on jump run drops vertical speed below the climb threshold, the phase machine
// calls it CANOPY, and the screen blanks moments before exit.
static bool     g_inAircraft   = false;
#endif

// ---------------------------------------------------------------------------
//  Altitude helpers
// ---------------------------------------------------------------------------
static float aglFromPressure(float pressureHpa)
{
  const float msl = baro::pressureAltitude(pressureHpa, g_qnhHpa);
  if (!g_calibrated) return msl;
  return msl - baro::pressureAltitude(g_groundPHpa, g_qnhHpa);
}

// ---------------------------------------------------------------------------
//  Persistence
// ---------------------------------------------------------------------------
static void saveCalibration()
{
  g_prefs.putFloat("qnh", g_qnhHpa);
  g_prefs.putFloat("groundP", g_groundPHpa);
}

static void loadCalibration()
{
  // isKey() first: reading a missing key works fine but logs an ESP-IDF error,
  // which looks alarming in the boot output on a first-ever run.
  g_qnhHpa     = g_prefs.isKey("qnh")     ? g_prefs.getFloat("qnh")     : baro::kISASeaLevel_hPa;
  g_groundPHpa = g_prefs.isKey("groundP") ? g_prefs.getFloat("groundP") : 0.0f;
  g_calibrated = (g_groundPHpa > 300.0f && g_groundPHpa < 1200.0f);
  if (g_prefs.isKey("bright")) led::setBrightness(g_prefs.getUChar("bright"));
  display::setUnitsFeet(g_prefs.isKey("feet") ? g_prefs.getUChar("feet") != 0
                                              : UNITS_FEET_DEFAULT);
}

// ---------------------------------------------------------------------------
//  Sensor
// ---------------------------------------------------------------------------
static void i2cScan()
{
  Serial.println(F("I2C scan:"));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      Serial.printf("  device at 0x%02X%s\n", addr,
                    (addr == 0x77 || addr == 0x76) ? "   <- MS5611" : "");
      found++;
    }
  }
  if (!found) Serial.println(F("  nothing found — check wiring, pull-ups and PS pin"));
}

// Dump the factory calibration so a mis-identified or counterfeit die is
// obvious. A genuine MS5611 has all six constants in roughly 20000-60000;
// zeros, 0xFFFF, or wildly small values mean the PROM read is not working and
// no amount of maths downstream will save the readings.
static void dumpProm()
{
  Serial.print(F("  PROM:"));
  for (uint8_t i = 1; i <= 6; i++) Serial.printf(" C%u=%u", i, g_sensor.getProm(i));
  Serial.printf("\n  reserved=0x%04X  crc=0x%X\n",
                g_sensor.getProm(0), g_sensor.getCRC());
}

// Try both possible MS5611 addresses. Returns true once one answers.
//
// Note this deliberately does not call begin(): begin() hardcodes
// reset(0), and some dies need the mathMode 1 scaling (see config.h).
// Otherwise this is exactly what begin() does.
static bool sensorBegin()
{
  const uint8_t addrs[] = {MS5611_ADDR_PRIMARY, MS5611_ADDR_SECONDARY};
  for (uint8_t a : addrs)
  {
    g_sensor = MS5611(a);
    if (!g_sensor.isConnected()) continue;
    if (!g_sensor.reset(SENSOR_MATH_MODE)) continue;   // false => PROM read as zeros

    g_sensor.setOversampling(SENSOR_OSR);
    Serial.printf("MS5611 found at 0x%02X (device id 0x%08X, mathMode %d)\n",
                  a, (unsigned)g_sensor.getDeviceID(), SENSOR_MATH_MODE);
    dumpProm();
    return true;
  }
  return false;
}

// Sanity-check the very first reading against the range the atmosphere
// actually occupies. Getting this wrong is silent and expensive: a factor-of-2
// pressure error still produces a smooth, stable, entirely believable altitude
// trace — it is just wrong by kilometres.
static void checkPressurePlausible()
{
  // Roughly 500 hPa (~5500 m, above any drop zone) to 1085 hPa (record high).
  if (g_pressureHpa > 500.0f && g_pressureHpa < 1085.0f) return;

  Serial.println(F("\n*** PRESSURE IMPLAUSIBLE ***"));
  Serial.printf("Read %.2f hPa, which is outside the range the atmosphere occupies.\n",
                g_pressureHpa);
  Serial.printf("Doubling gives %.2f hPa; halving gives %.2f hPa.\n",
                g_pressureHpa * 2.0f, g_pressureHpa * 0.5f);
  Serial.printf("If one of those looks like your local pressure, flip\n"
                "SENSOR_MATH_MODE in config.h (currently %d) and reflash.\n",
                SENSOR_MATH_MODE);
  Serial.println(F("****************************\n"));
}

// One blocking conversion. Returns true on success and updates the globals.
static bool sensorRead()
{
  if (g_sensor.read() != MS5611_READ_OK) return false;
  g_pressureHpa = g_sensor.getPressure();
  g_tempC       = g_sensor.getTemperature();
  // A totally implausible reading means a bus glitch, not weather.
  if (!(g_pressureHpa > 100.0f && g_pressureHpa < 1200.0f)) return false;
  return true;
}

// ---------------------------------------------------------------------------
//  Ground zero
// ---------------------------------------------------------------------------
// Blocking operations (zeroing, the LED sweep) stall the sample loop for a
// second or more. Resync both the schedule and the dt reference afterwards —
// otherwise the next filter update sees a dt of ~1.5 s, which inflates the
// covariance and makes sigma_m jump for a few samples.
static void resyncSampleClock()
{
  g_nextSampleMs = millis();
  g_lastSampleUs = micros();
}

// `tick`, if given, is called once per sample and once per 20 ms of settle
// wait, so a caller can animate while this blocks. It must be cheap.
// "Setting the ground zero" plus 0-3 dots, one every 350 ms. Space-padded to a
// constant width so the shrinking case repaints its own tail — the banner draws
// opaque, but only over the glyphs it is given.
static const char kZeroLine[] = "Setting the ground zero   ";

static void zeroDotsTick()
{
  static uint32_t lastMs = 0;
  static uint8_t  dots   = 0;
  const uint32_t now = millis();
  if (lastMs && (now - lastMs) < 350) return;
  lastMs = now;

  char line[40];
  snprintf(line, sizeof(line), "Setting the ground zero%.*s%*s",
           dots, "...", 3 - dots, "");
  display::bannerLine1(line);
  dots = (dots + 1) % 4;
}

static bool zeroHere(void (*tick)() = nullptr)
{
  Serial.println(F("\nZeroing — hold still..."));
  led::set(Rgb{255, 255, 255});

  // Wait out the warm-up drift before averaging. The sensor falls about a metre
  // over the first seconds after a reset (see FILTER_SETTLE_MS), so a zero taken
  // straight after power-up stores a pressure the board is about to leave, and
  // the display then sits ~1 m low indefinitely. The half-vs-half drift test
  // below does not catch it: the flight limit is 1.00 m and the warm-up spreads
  // less than that across a 2 s average, so it passes and the error is stored.
  const int32_t settleLeft = static_cast<int32_t>(g_filterSettledMs - millis());
  if (settleLeft > 0)
  {
    Serial.printf("  sensor still warming, waiting %.1f s first.\n",
                  settleLeft / 1000.0);
    for (int32_t left = settleLeft; left > 0; left -= 20)
    {
      if (tick) tick();
      delay(20);
    }
  }

  double pSum = 0.0, aSum = 0.0, aSumSq = 0.0, aFirst = 0.0, aSecond = 0.0;
  uint16_t n = 0, nFirst = 0, nSecond = 0;

  for (uint16_t i = 0; i < ZERO_SAMPLE_COUNT; i++)
  {
    if (tick) tick();
    if (!sensorRead()) continue;
    const double msl = baro::pressureAltitude(g_pressureHpa, g_qnhHpa);

    pSum   += g_pressureHpa;
    aSum   += msl;
    aSumSq += msl * msl;
    n++;

    if (i < ZERO_SAMPLE_COUNT / 2) { aFirst  += msl; nFirst++;  }
    else                           { aSecond += msl; nSecond++; }
  }

  if (n < ZERO_SAMPLE_COUNT / 2 || nFirst == 0 || nSecond == 0)
  {
    Serial.println(F("Zero FAILED: too many bad reads."));
    return false;
  }

  const double mean  = aSum / n;
  const double var   = (aSumSq / n) - (mean * mean);
  const float  sigma = (var > 0.0) ? sqrtf(static_cast<float>(var)) : 0.0f;
  const float  drift = fabsf(static_cast<float>(aFirst / nFirst - aSecond / nSecond));
  const float  sem   = sigma / sqrtf(static_cast<float>(n));   // accuracy of the mean

  if (sigma > ZERO_MAX_SIGMA_M)
  {
    Serial.printf("Zero FAILED: sample noise %.3f m rms (limit %.2f).\n"
                  "  Sensor problem, or serious air movement.\n",
                  sigma, (double)ZERO_MAX_SIGMA_M);
    return false;
  }

  if (drift > ZERO_MAX_DRIFT_M)
  {
    Serial.printf("Zero FAILED: altitude drifted %.3f m during the average "
                  "(limit %.2f).\n"
                  "  Board moved, or a door/window changed the room pressure.\n",
                  drift, (double)ZERO_MAX_DRIFT_M);
    return false;
  }

  g_groundPHpa = static_cast<float>(pSum / n);
  g_calibrated = true;
  saveCalibration();

  g_filter.reset(0.0f);
  g_filterSettledMs = millis() + FILTER_SETTLE_MS;   // as after any filter reset
  g_zones.reset(0.0f);
#if AUTOZERO_ENABLED
  g_groundRef.reset(millis());     // a manual zero supersedes any drift progress
#endif
#if !BENCH_MODE
  g_landing.reset(0.0f);
  g_modes.reset(MODE_CANOPY);
  g_climb.update(0.0f, false);
#endif

  Serial.printf("Zero OK: ground = %.3f hPa over %u samples.\n"
                "  noise %.3f m rms, drift %.3f m, zero good to +/-%.3f m.\n",
                g_groundPHpa, n, sigma, drift, sem);

  // Two green flashes as visual confirmation.
  for (int i = 0; i < 2; i++)
  {
    led::set(Rgb{0, 255, 0}); delay(120);
    led::off();               delay(120);
  }
  return true;
}

static void clearZero()
{
  g_groundPHpa = 0.0f;
  g_calibrated = false;
  saveCalibration();
  Serial.println(F("Zero cleared — reporting raw ISA pressure altitude."));
}

// ---------------------------------------------------------------------------
//  Idle light sleep
// ---------------------------------------------------------------------------
#if IDLE_SLEEP_ENABLED
// May we sleep right now?
//
// The condition is DEMONSTRABLY ON THE GROUND, not merely "not climbing". An
// aircraft holding at 4000 m for clearance has ~zero vertical speed and is
// indistinguishable from sitting on a table by speed alone. Sleeping there
// would cost up to 30 s of blindness — 1500 m of freefall at terminal. So a low
// altitude is required as well, plus the same in-flight latch the drift
// correction uses.
static bool mayIdleSleep(uint32_t nowMs)
{
  if (!g_calibrated) return false;                    // nothing to compare against
#if !IDLE_SLEEP_IGNORE_USB
  if (Serial) return false;                           // a host is attached: stay up
#endif
  if (demo::active()) return false;
  if (display::screen() != display::UI_ALT) return false;
  if (static_cast<int32_t>(nowMs - g_wakeShowUntil) < 0) return false;
  if (g_failStreak) return false;                     // sensor unhappy: stay awake

  if (fabsf(g_filter.altitude()) > IDLE_MAX_ALT_M) return false;
  if (fabsf(g_filter.velocity()) > IDLE_MAX_VSPEED_MPS) return false;
  // The in-flight latch used to gate this too, and it was the 120 s hold seen on
  // hardware: the trace showed the reason switch from "wake window" to
  // "in-flight latch" the instant the 20 s window expired, then sit there for
  // the latch's full clear time. It bought nothing. That latch exists to stop
  // auto-zero running at altitude; sleep is already refused above 25 m and above
  // 0.5 m/s, which covers the aircraft case directly and more strictly.
  return static_cast<uint32_t>(nowMs - g_activeSinceMs) > IDLE_QUIET_BEFORE_MS;
}

#if IDLE_TRACE
// Which single condition is holding us awake. Speculating about this from the
// source went wrong twice; the device is a better witness than I am.
static const char *idleBlockReason(uint32_t nowMs)
{
  if (!g_calibrated) return "not calibrated";
#if !IDLE_SLEEP_IGNORE_USB
  if (Serial) return "USB host attached";
#endif
  if (demo::active()) return "demo running";
  if (display::screen() != display::UI_ALT) return "menu/banner on screen";
  if (static_cast<int32_t>(nowMs - g_wakeShowUntil) < 0) return "wake window";
  if (g_failStreak) return "sensor fail streak";
  if (fabsf(g_filter.altitude()) > IDLE_MAX_ALT_M) return "altitude > 25 m";
  if (fabsf(g_filter.velocity()) > IDLE_MAX_VSPEED_MPS) return "moving";
  if (static_cast<uint32_t>(nowMs - g_activeSinceMs) <= IDLE_QUIET_BEFORE_MS)
    return "quiet timer";
  return nullptr;
}

static void idleTrace(uint32_t nowMs)
{
  static uint32_t last = 0;
  if (static_cast<uint32_t>(nowMs - last) < 1000) return;
  last = nowMs;
  const char *why = idleBlockReason(nowMs);
  if (!why) return;
  Serial.printf("idle: awake — %s (wake %+ld ms, quiet %lu ms, set by: %s)\n", why,
                (long)(int32_t)(g_wakeShowUntil - nowMs),
                (unsigned long)(nowMs - g_activeSinceMs), g_activeWho);
}
#endif

static void idleSleep()
{
  Serial.printf("idle: sleeping %d s (alt %.1f m)\n",
                IDLE_WAKE_PERIOD_S, g_filter.altitude());
  Serial.flush();

  // Park the renderer and the SD writer first, so we never suspend in the
  // middle of an SPI frame or a card write.
  logger::pause();
  display::sleep();

  esp_sleep_enable_timer_wakeup((uint64_t)IDLE_WAKE_PERIOD_S * 1000000ULL);
  esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(PIN_BUTTON), 0);
  esp_light_sleep_start();
  const esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  logger::resume();

  // A 30 s gap makes the filter's velocity state meaningless, so start clean
  // rather than feeding it a dt of 30 seconds.
  resyncSampleClock();
  for (int i = 0; i < 4; i++)
    if (sensorRead()) g_rawAglM = aglFromPressure(g_pressureHpa);
  g_filter.reset(g_rawAglM);
  g_filterSettledMs = millis() + FILTER_SETTLE_MS;

  const bool byButton = (cause == ESP_SLEEP_WAKEUP_EXT0);
  if (byButton)
  {
    display::wake();
    g_wakeShowUntil = millis() + IDLE_WAKE_DISPLAY_MS;
    g_btnSwallowUntil = millis() + BUTTON_WAKE_SWALLOW_MS;
    // Deliberately NOT touching g_activeSinceMs. Sleep needs both the wake
    // window to expire AND the quiet timer to be satisfied, so resetting the
    // quiet timer here made the effective wake max(20 s, 60 s) = 60 s and the
    // 20 s setting did nothing. A glance at the altitude should cost 20 s, not
    // a minute; real interaction still restarts the quiet timer where it is
    // handled, in pollButton and the touch path.
    Serial.println(F("idle: woken by button"));
  }
  else if (fabsf(g_rawAglM) > IDLE_MAX_ALT_M)
  {
    // Something changed while we were out — most likely a climb starting.
    display::wake();
    MARK_ACTIVE(millis(), "woke-to-altitude");
    Serial.printf("idle: woken to %.1f m — staying awake\n", g_rawAglM);
  }
  // Otherwise the panel stays dark and we fall straight back to sleep.
}
#endif  // IDLE_SLEEP_ENABLED

// ---------------------------------------------------------------------------
//  Menu actions
// ---------------------------------------------------------------------------
static uint32_t g_menuIdleMs = 0;

static void openMenu()
{
  display::setScreen(display::UI_MENU);
  g_menuIdleMs = millis();
}

static void closeMenu()
{
  display::setScreen(display::UI_ALT);
}

static void unmountCard()
{
  logger::close();
  display::setBanner("CARD UNMOUNTED", "safe to remove");
  Serial.println(F("Card unmounted. Logging stays off until reboot."));
}

// "Power off" is deep sleep, not a true power cut: without a hardware latch the
// ESP32 cannot disconnect its own supply. Panel asleep, card unmounted, wake on
// the BOOT button or RST.
// Hold the touch and panel controllers in reset for the duration of deep sleep,
// so they stop drawing. GPIO40 and GPIO47 are outside the RTC bank (GPIO0-21),
// so rtc_gpio_hold does not apply to them — the digital-pad hold is what
// latches a level once the CPU is off. See config.h for the measurements.
static void holdPeripheralsForDeepSleep()
{
#if DEEPSLEEP_HOLD_TOUCH_RST
  pinMode(PIN_TOUCH_RST, OUTPUT);
  digitalWrite(PIN_TOUCH_RST, LOW);
  gpio_hold_en(static_cast<gpio_num_t>(PIN_TOUCH_RST));
#endif
#if DEEPSLEEP_HOLD_LCD_RST
  pinMode(PIN_LCD_RST, OUTPUT);
  digitalWrite(PIN_LCD_RST, LOW);
  gpio_hold_en(static_cast<gpio_num_t>(PIN_LCD_RST));
#endif
#if DEEPSLEEP_HOLD_TOUCH_RST || DEEPSLEEP_HOLD_LCD_RST
  gpio_deep_sleep_hold_en();
#endif
}

static void powerOff()
{
  // Refuse while moving. Shutting down in freefall is the one failure mode
  // worth designing out, and a stray tap under canopy should never do it.
  if (fabsf(g_filter.velocity()) > SLEEP_MAX_VSPEED_MPS)
  {
    display::setBanner("NOT WHILE", "MOVING");
    Serial.println(F("Power off refused: still moving."));
    delay(1500);
    closeMenu();
    return;
  }

  Serial.println(F("Powering down. Press BOOT or RST to wake."));
  logger::close();
  display::sleepFace();
  display::sleep();

  holdPeripheralsForDeepSleep();

  esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(PIN_BUTTON), 0);
  esp_deep_sleep_start();
}

static void zeroFromMenu()
{
  display::setBanner("ZEROING", "hold still");
  const bool ok = zeroHere();
  resyncSampleClock();
  display::setBanner(ok ? "ZERO OK" : "ZERO FAILED", ok ? "" : "try again");
  delay(1400);
  closeMenu();
}

static void handleGesture(const touch::Event &e)
{
  g_menuIdleMs = millis();
  const uint8_t scr = display::screen();

  // Swipes page between the menu screens; they mean nothing on a confirm.
  if (e.type == touch::EV_SWIPE_LEFT)
  {
    if (scr == display::UI_MENU)  { display::setScreen(display::UI_MENU2); return; }
    if (scr == display::UI_MENU2) { display::setScreen(display::UI_MENU3); return; }
  }
  if (e.type == touch::EV_SWIPE_RIGHT)
  {
    if (scr == display::UI_MENU3) { display::setScreen(display::UI_MENU2); return; }
    if (scr == display::UI_MENU2) { display::setScreen(display::UI_MENU);  return; }
  }
  if (e.type != touch::EV_TAP) return;

  switch (display::hitTest(e.x, e.y))
  {
    case display::ACT_ZERO:    display::setScreen(display::UI_CONFIRM_ZERO);    break;
    case display::ACT_DEMO:    closeMenu(); demo::start();                       break;
    case display::ACT_USB:     display::setScreen(display::UI_CONFIRM_USB);      break;
    case display::ACT_UNITS:
    {
      // Display-only: everything internal stays in metres, so this cannot
      // disturb a threshold, a test, or the meaning of a log.
      const bool feet = !display::unitsFeet();
      display::setUnitsFeet(feet);
      g_prefs.putUChar("feet", feet ? 1 : 0);
      Serial.printf("\nUnits: %s\n", feet ? "feet / mph" : "metres / m per s");
      closeMenu();
      break;
    }
    case display::ACT_UNMOUNT: display::setScreen(display::UI_CONFIRM_UNMOUNT); break;
    case display::ACT_POWER:   display::setScreen(display::UI_CONFIRM_POWER);   break;
    case display::ACT_CONFIRM:
      if      (scr == display::UI_CONFIRM_POWER) powerOff();
      else if (scr == display::UI_CONFIRM_ZERO)  zeroFromMenu();
      else if (scr == display::UI_CONFIRM_USB)
      {
        // Close the log first: the host is about to own this filesystem, and
        // two writers on one FAT corrupt it.
        logger::close();
        display::setBanner("REBOOTING", "to USB drive");
        delay(800);
        usbmsc::rebootIntoMode();          // does not return
      }
      else                                       unmountCard();
      break;
    case display::ACT_CANCEL:  closeMenu(); break;
    default: break;
  }
}

// ---------------------------------------------------------------------------
//  Button (BOOT): tap = menu, hold = re-zero
// ---------------------------------------------------------------------------
static void pollButton(uint32_t nowMs)
{
  static bool     wasDown = false;
  static uint32_t downAt  = 0;

  const int raw = digitalRead(PIN_BUTTON);
  const bool down = BUTTON_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);

  if (down && !wasDown) downAt = nowMs;

  // Acting on release, not while held, is what lets one button mean two things
  // without the short action firing on the way to the long one.
  if (!down && wasDown)
  {
    if (static_cast<int32_t>(nowMs - g_btnSwallowUntil) < 0)
    {
      g_btnSwallowUntil = 0;   // the wake press is spent; the next one counts
      wasDown = down;
      return;
    }
    const uint32_t held = nowMs - downAt;
    if (held >= ZERO_BUTTON_HOLD_MS)
    {
      zeroHere();
      resyncSampleClock();
    }
    else if (held >= 30)          // debounce
    {
      g_autoOffWarnMs = 0;                        // interaction cancels auto-off
      MARK_ACTIVE(nowMs, "button-release");
      if (demo::active())                            demo::stop();
      else if (display::screen() == display::UI_ALT) openMenu();
      else                                           closeMenu();
    }
  }
  wasDown = down;
}

// ---------------------------------------------------------------------------
//  Serial console
// ---------------------------------------------------------------------------
static void printHelp()
{
  Serial.println(F(
    "\nCommands:\n"
    "  z            zero here (average ground pressure)\n"
    "  r            clear zero, back to raw ISA pressure altitude\n"
    "  q <hPa>      set QNH reference, e.g. 'q 1017.4'\n"
    "  b <0-255>    LED brightness\n"
    "  t            LED self-test sweep\n"
    "  p            play every LED pattern (climb/zones/landing/fault)\n"
    "  c            toggle serial CSV streaming\n"
    "  l            toggle SD logging\n"
    "  s            status\n"
    "  u            toggle feet / metres\n"
    "  w <unix>     set the clock, so log files are not stamped 1980\n"
    "  T            touch coordinate dump (for calibrating the mapping)\n"
    "  ?            this help"));
}

static void printStatus()
{
  Serial.println(F("\n--- status ---"));
  Serial.printf("mode          : %s\n", BENCH_MODE ? "BENCH (metres)" : "FLIGHT (real thresholds)");
  Serial.printf("sensor addr   : 0x%02X\n", g_sensor.getAddress());
  Serial.printf("QNH           : %.2f hPa\n", g_qnhHpa);
  Serial.printf("ground ref    : %s", g_calibrated ? "" : "NOT SET\n");
  if (g_calibrated) Serial.printf("%.3f hPa\n", g_groundPHpa);
  Serial.printf("pressure      : %.3f hPa @ %.2f C\n", g_pressureHpa, g_tempC);
  Serial.printf("AGL raw/filt  : %.3f / %.3f m  (sigma %.3f)\n",
                g_rawAglM, g_filter.altitude(), g_filter.sigma());
  Serial.printf("vertical speed: %.2f m/s\n", g_filter.velocity());
  Serial.printf("zone          : %s\n", zoneName(g_zones.zone()));
#if !BENCH_MODE
  Serial.printf("flight phase  : %s%s\n", flightModeName(g_modes.mode()),
                g_inAircraft ? " (in aircraft)" : "");
  Serial.printf("landing band  : %s\n", landingName(g_landing.zone()));
#endif
  Serial.printf("fail streak   : %u   loop overruns: %u\n", g_failStreak, (unsigned)g_overruns);
  Serial.printf("LED brightness: %u\n", led::brightness());
  Serial.printf("units         : %s (display only; logs stay metric)\n",
                display::unitsFeet() ? "feet / mph" : "metres / m per s");
  // Three decimals here, two on screen. A 2-decimal readout resolves 0.01 V,
  // which over a 6 h idle run is a single LSB — too coarse to measure a few
  // milliamps. The ADC has far more resolution than that, so the console gets
  // it and the estimate stops being guesswork.
  Serial.printf("battery       : %.3f V\n", g_battV);
  Serial.printf("reset reason  : %s\n", g_resetReason);
#if IDLE_SLEEP_ENABLED
  Serial.printf("idle sleep    : %s (needs alt<%.0fm, still, %ds quiet)\n",
                mayIdleSleep(millis()) ? "eligible now" : "held awake",
                (double)IDLE_MAX_ALT_M, (int)(IDLE_QUIET_BEFORE_MS / 1000));
#endif
  Serial.printf("uptime        : %.1f h of %d h before auto-off\n",
                millis() / 3600000.0, AUTO_OFF_HOURS);
#if AUTOZERO_ENABLED
  Serial.printf("auto-zero     : %s%s (needs %.1f min settled at this altitude)\n",
                g_groundRef.inFlight() ? "LOCKED OUT (in flight)" : "armed",
                g_groundRef.correcting() ? ", correcting now" : "",
                g_groundRef.requiredSettleMs(g_filter.altitude()) / 60000.0);
#endif
  {
    const time_t now = time(nullptr);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", localtime(&now));
    Serial.printf("clock         : %s\n", ts);
  }
  if (display::available())
    Serial.printf("display       : ok, full fill %lu us\n",
                  (unsigned long)display::lastFillUs());
  if (logger::available())
    Serial.printf("log           : %s (%llu MB card) %lu rows, %lu dropped, %s\n",
                  logger::filename(), (unsigned long long)logger::cardSizeMb(),
                  (unsigned long)logger::rowsWritten(),
                  (unsigned long)logger::rowsDropped(),
                  logger::enabled() ? "on" : "off");
  else
    Serial.println(F("log           : no card"));
  Serial.println(F("--------------"));
}

static void handleCommand(char *line)
{
  while (*line == ' ') line++;
  const char cmd = *line;
  const char *arg = line + 1;

  switch (cmd)
  {
    case 'z': zeroHere(); resyncSampleClock(); break;
    case 'r': clearZero(); break;
    case 't': led::selfTest(); resyncSampleClock(); break;
    case 'p':
      Serial.println(F("\nPattern preview:"));
      led::patternPreview();
      resyncSampleClock();
      break;
    case 's': printStatus(); break;
    case 'u':
    {
      const bool feet = !display::unitsFeet();
      display::setUnitsFeet(feet);
      g_prefs.putUChar("feet", feet ? 1 : 0);
      Serial.printf("\nUnits: %s\n", feet ? "feet / mph" : "metres / m per s");
      break;
    }
    case 'w':
    {
      const long epoch = atol(arg);
      if (epoch > 1700000000L)          // sanity: later than Nov 2023
      {
        const struct timeval tv = {(time_t)epoch, 0};
        settimeofday(&tv, nullptr);
        Serial.printf("\nClock set. New log files will be stamped correctly.\n");
      }
      else Serial.println(F("\nusage: w <unix seconds>  (see the README for a one-liner)"));
      break;
    }
    case 'T':
      g_touchDump = !g_touchDump;
      Serial.printf("\nTouch dump %s — tap the corners; set TOUCH_SWAP_XY / "
                    "TOUCH_FLIP_X / TOUCH_FLIP_Y in config.h so the mapped\n"
                    "coordinates match where you actually tapped "
                    "(0,0 = top-left, %d,%d = bottom-right).\n",
                    g_touchDump ? "on" : "off", DISPLAY_W - 1, DISPLAY_H - 1);
      break;
    case 'l':
      logger::setEnabled(!logger::enabled());
      Serial.printf("\nSD logging %s\n", logger::enabled() ? "on" : "off");
      break;
    case '?': case 'h': printHelp(); break;
    case 'c':
      g_csvEnabled = !g_csvEnabled;
      Serial.printf("\nCSV %s\n", g_csvEnabled ? "on" : "off");
      break;
    case 'q':
    {
      const float v = atof(arg);
      if (v > 800.0f && v < 1100.0f)
      {
        g_qnhHpa = v;
        saveCalibration();
        Serial.printf("\nQNH = %.2f hPa\n", g_qnhHpa);
      }
      else Serial.println(F("\nQNH out of range (800-1100 hPa)"));
      break;
    }
    case 'b':
    {
      const int v = atoi(arg);
      if (v >= 0 && v <= 255)
      {
        led::setBrightness(static_cast<uint8_t>(v));
        g_prefs.putUChar("bright", static_cast<uint8_t>(v));
        Serial.printf("\nbrightness = %d\n", v);
      }
      break;
    }
    case '\0': break;
    default: Serial.println(F("\n? unknown command — '?' for help")); break;
  }
}

static void pollSerial()
{
  static char buf[32];
  static uint8_t len = 0;

  while (Serial.available())
  {
    const char c = Serial.read();
    // Accept CR, LF or CRLF as the line terminator. Terminals disagree about
    // what Enter sends, and only honouring '\n' means a CR-only terminal
    // silently swallows every command. Empty lines are skipped, which is what
    // makes the CRLF case work.
    if (c == '\r' || c == '\n')
    {
      if (len > 0)
      {
        buf[len] = '\0';
        handleCommand(buf);
        len = 0;
      }
    }
    else if (len < sizeof(buf) - 1)
    {
      buf[len++] = c;
    }
  }
}

// ---------------------------------------------------------------------------
//  Setup / loop
// ---------------------------------------------------------------------------
void setup()
{
  // Before anything else: did the last boot ask for USB drive mode? The check
  // consumes the request, so a crash in that mode cannot trap the device — any
  // reset comes back here and continues normally.
  if (usbmsc::bootRequested()) usbmsc::runForever();   // never returns

  // A pad hold latched before deep sleep survives the wake, so release both
  // reset lines before anything initialises a peripheral — otherwise the panel
  // and touch controller stay in reset for the whole session. Unconditional:
  // the previous boot may have been a build with the holds enabled.
  gpio_hold_dis(static_cast<gpio_num_t>(PIN_TOUCH_RST));
  gpio_hold_dis(static_cast<gpio_num_t>(PIN_LCD_RST));
  gpio_deep_sleep_hold_dis();

  Serial.begin(115200);
  delay(300);

  led::begin();
  led::selfTest();

  display::begin();

  pinMode(PIN_BUTTON, BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);

  Serial.printf("\n\n=== Skydiving altimeter v%s (%s) ===\n",
                FW_VERSION, BENCH_MODE ? "BENCH" : "FLIGHT");
  // The power LEDs are hard-wired and stay lit in deep sleep, so they say
  // nothing about whether sleep worked. The reset reason does: waking from
  // deep sleep reports DEEPSLEEP, a plain power-up or RST does not.
  {
    const esp_reset_reason_t rr = esp_reset_reason();
    g_resetReason = rr == ESP_RST_DEEPSLEEP ? "DEEPSLEEP (woke from power-off)"
                  : rr == ESP_RST_POWERON   ? "POWERON"
                  : rr == ESP_RST_SW        ? "SOFTWARE (reboot)"
                  : rr == ESP_RST_PANIC     ? "PANIC (crash!)"
                  : rr == ESP_RST_EXT       ? "EXTERNAL (RST button)"
                                            : "other";
    Serial.printf("Reset reason : %s\n", g_resetReason);
  }

  // The board has no battery-backed clock and no network, so on a cold start
  // it believes it is 1980 — which is what FAT then stamps on every log file.
  // Seed from the firmware build time so files at least land in the right week,
  // and use 'w <unix>' to set it exactly. The RTC keeps running through deep
  // sleep, so a set time survives a power-off/wake cycle.
  {
    static const char kMon[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char mon[4] = {__DATE__[0], __DATE__[1], __DATE__[2], 0};
    struct tm bt = {};
    bt.tm_mon  = (int)((strstr(kMon, mon) - kMon) / 3);
    bt.tm_mday = atoi(__DATE__ + 4);
    bt.tm_year = atoi(__DATE__ + 7) - 1900;
    bt.tm_hour = atoi(__TIME__);
    bt.tm_min  = atoi(__TIME__ + 3);
    bt.tm_sec  = atoi(__TIME__ + 6);
    const time_t built = mktime(&bt);
    if (time(nullptr) < built)
    {
      const struct timeval tv = {built, 0};
      settimeofday(&tv, nullptr);
      Serial.printf("Clock seeded from build time; 'w <unix>' to set exactly.\n");
    }
  }
  Serial.println(F("NOT A SAFETY DEVICE — secondary visual aid only."));
  Serial.printf("Mode: %s\n", BENCH_MODE ? "BENCH (metre-scale thresholds)"
                                         : "FLIGHT (real thresholds)");

  // Drive the GY-63 mode straps before touching the bus, so the sensor is
  // guaranteed to be in I2C mode at a known address by the time we scan.
  // Skipped when the pins are set to -1 and the straps are wired to rails.
  if (PIN_SENSOR_PS >= 0)
  {
    pinMode(PIN_SENSOR_PS, OUTPUT);
    digitalWrite(PIN_SENSOR_PS, HIGH);          // HIGH = I2C
  }
  if (PIN_SENSOR_CSB >= 0)
  {
    pinMode(PIN_SENSOR_CSB, OUTPUT);
    digitalWrite(PIN_SENSOR_CSB, SENSOR_CSB_LEVEL ? HIGH : LOW);
  }
  if (PIN_SENSOR_PS >= 0 || PIN_SENSOR_CSB >= 0)
  {
    delay(20);                                   // let the sensor latch the mode
    Serial.printf("Straps driven: PS=GPIO%d HIGH, CSB=GPIO%d %s\n",
                  PIN_SENSOR_PS, PIN_SENSOR_CSB, SENSOR_CSB_LEVEL ? "HIGH" : "LOW");
  }

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_FREQ_HZ);
  i2cScan();

  if (!sensorBegin())
  {
    Serial.println(F("MS5611 NOT FOUND at 0x77 or 0x76."));
    Serial.printf("Check: 3V3/GND, SDA->GPIO%d, SCL->GPIO%d, PS HIGH for I2C.\n",
                  PIN_I2C_SDA, PIN_I2C_SCL);
    // Sit in the fault pattern rather than pretending everything is fine.
    display::message("SENSOR FAIL", "no MS5611");
    while (true)
    {
      led::render(led::faultPattern(), millis());
      delay(10);
    }
  }

  // The warm-up drift starts here, not at the end of setup, so this is where
  // the settle window belongs. The boot animation then runs inside it and the
  // boot zero below costs no extra waiting.
  g_filterSettledMs = millis() + BOOT_ZERO_SETTLE_MS;

  g_prefs.begin("altimeter", false);
  loadCalibration();

  g_filter.configureAdaptive(FILTER_GATE_SIGMA, FILTER_MAX_INFLATE);
#if AUTOZERO_ENABLED
  g_groundRef.begin({AUTOZERO_SETTLED_MPS, AUTOZERO_BAND_M, AUTOZERO_BASE_MIN,
                     AUTOZERO_PER_M_MIN, AUTOZERO_SLEW_M_PER_MIN,
                     AUTOZERO_LATCH_VS_MPS, AUTOZERO_LATCH_ALT_M,
                     AUTOZERO_LATCH_CLEAR_MS});
#endif
  g_zones.begin(kZoneConfig);
#if !BENCH_MODE
  g_landing.begin(kLandingConfig);
  g_modes.begin({MODE_FREEFALL_ENTER_MPS, MODE_FREEFALL_EXIT_MPS,
                 MODE_CLIMB_ENTER_MPS, MODE_CLIMB_EXIT_MPS, MODE_DWELL_MS});
  g_climb.configure(CLIMB_MARK_INTERVAL_M);
#endif

#if BOOT_ZERO_ENABLED
  if (g_calibrated)
    Serial.printf("Restored ground ref %.3f hPa from flash; the boot zero below supersedes it.\n",
                  g_groundPHpa);
#else
  if (g_calibrated)
    Serial.printf("Restored ground ref %.3f hPa from flash — RE-ZERO before use.\n", g_groundPHpa);
  else
    Serial.println(F("No ground reference stored. Press 'z' or hold BOOT for 1.5 s."));
#endif

  // Prime the filter so the first zone decision is not made from a cold start.
  for (int i = 0; i < 8; i++)
  {
    if (sensorRead()) g_rawAglM = aglFromPressure(g_pressureHpa);
  }
  checkPressurePlausible();
  g_filter.reset(g_rawAglM);
  g_zones.reset(g_rawAglM);

  display::bootFace();
  display::bootFaceOut();
#if BOOT_ZERO_ENABLED
  // The dots are driven by the zero's own sample loop rather than by a timer
  // running alongside it, so they cannot keep animating past a stall or stop
  // early — what is on screen is what the device is actually doing.
  display::bannerIn(kZeroLine, "");
  if (!zeroHere(zeroDotsTick))
    Serial.println(F("Boot zero rejected — use 'z' once it is still."));
  resyncSampleClock();
  display::bannerLine1("Setting the ground zero...");
  display::bannerLine2In(BENCH_MODE ? "Bench mode activated"
                                    : "Flight mode activated");
  delay(900);
  display::bannerOut();
#endif
#if !BENCH_MODE
  g_landing.reset(g_rawAglM);
  g_modes.reset(MODE_CANOPY);
  g_climb.update(g_rawAglM, false);
#endif

  display::startTask();

  logger::begin(g_qnhHpa, g_groundPHpa);
  logger::startTask();

  touch::begin();

  g_filterSettledMs = millis() + FILTER_SETTLE_MS;

  printHelp();
#if BENCH_MODE
  Serial.println(F("\nt_ms,p_hpa,temp_c,raw_m,filt_m,vs_mps,sigma_m,zone"));
#else
  Serial.println(F("\nt_ms,p_hpa,temp_c,raw_m,filt_m,vs_mps,sigma_m,zone,phase,land"));
#endif

  g_lastSampleUs = micros();
  g_nextSampleMs = millis();
  g_nextCsvMs    = millis();
}

// Which pattern should the LED and screen be showing right now?
static LedPattern currentPattern()
{
  if (g_failStreak >= SENSOR_FAIL_LIMIT) return led::faultPattern();

#if BENCH_MODE
  return led::freefallPattern(g_zones.zone());
#else
  return led::flightPattern(g_modes.mode(), g_zones.zone(), g_landing.zone(),
                            g_filter.altitude(), g_inAircraft);
#endif
}

void loop()
{
  const uint32_t now = millis();

  pollSerial();
  pollButton(now);

  if (touch::available())
  {
    const touch::Event e = touch::takeEvent();
    if (e.type != touch::EV_NONE)
    {
      if (g_touchDump)
      {
        int16_t rx, ry;
        touch::rawLast(&rx, &ry);
        Serial.printf("touch %s raw=(%4d,%4d) -> start=(%4d,%4d)\n",
                      e.type == touch::EV_TAP        ? "TAP  "
                    : e.type == touch::EV_SWIPE_LEFT ? "SWIPE<"
                                                     : "SWIPE>",
                      rx, ry, e.x, e.y);
      }
      g_autoOffWarnMs = 0;                        // interaction cancels auto-off
      MARK_ACTIVE(now, "touch");
      if (demo::active())                          demo::stop();
      else if (display::screen() != display::UI_ALT) handleGesture(e);
    }
  }

  if (static_cast<int32_t>(now - g_nextBattMs) >= 0)
  {
    g_nextBattMs = now + BATTERY_PERIOD_MS;
    const float v = analogReadMilliVolts(PIN_BATTERY) * BATTERY_DIVIDER * 0.001f;
    // EMA: a single ADC sample jitters by tens of millivolts.
    g_battV = (g_battV <= 0.0f) ? v : g_battV + BATTERY_EMA * (v - g_battV);
    display::setBattery(g_battV);
  }

  // Auto power-off. Hygiene, not a power measure — but it must never fire
  // mid-jump, so it defers while moving and warns before acting.
  if (now > (uint32_t)AUTO_OFF_HOURS * 3600000UL)
  {
    if (fabsf(g_filter.velocity()) > SLEEP_MAX_VSPEED_MPS)
    {
      g_autoOffWarnMs = 0;                       // moving: defer, no warning
    }
    else if (g_autoOffWarnMs == 0)
    {
      g_autoOffWarnMs = now;
      display::setBanner("AUTO OFF", "touch to cancel");
      Serial.printf("Auto power-off in %lu s — touch or press BOOT to cancel.\n",
                    (unsigned long)(AUTO_OFF_WARN_MS / 1000));
    }
    else if (static_cast<uint32_t>(now - g_autoOffWarnMs) > AUTO_OFF_WARN_MS)
    {
      Serial.println(F("Auto power-off."));
      powerOff();
    }
  }

  // Never leave a menu covering the altitude in the air.
  if (display::screen() != display::UI_ALT &&
      static_cast<uint32_t>(now - g_menuIdleMs) > MENU_TIMEOUT_MS)
  {
    closeMenu();
  }

  if (static_cast<int32_t>(now - g_nextSampleMs) >= 0)
  {
    // If we fell more than a full period behind, note it and resync rather
    // than trying to catch up with a burst of reads.
    if (static_cast<int32_t>(now - g_nextSampleMs) > SAMPLE_PERIOD_MS)
    {
      g_overruns++;
      g_nextSampleMs = now;
    }
    g_nextSampleMs += SAMPLE_PERIOD_MS;

    const uint32_t nowUs = micros();
    const float dt = (nowUs - g_lastSampleUs) * 1e-6f;
    g_lastSampleUs = nowUs;

    if (sensorRead())
    {
      g_failStreak = 0;
      g_rawAglM = aglFromPressure(g_pressureHpa);
      g_filter.update(g_rawAglM, dt);
      g_zones.update(g_filter.altitude(), now);
#if !BENCH_MODE
      // Update every tracker every sample, even the one not currently driving
      // the LED. Otherwise a tracker goes stale while idle and shows a wrong
      // band for a moment when its mode comes back.
      g_landing.update(g_filter.altitude(), now);
      const FlightMode mode = g_modes.update(g_filter.velocity(), now);
      if (g_climb.update(g_filter.altitude(), mode == MODE_CLIMB))
      {
        led::flashOnce(led::kGreen, CLIMB_FLASH_MS, now);
      }

      // Aircraft latch: set by a climb well clear of the ground, released by
      // freefall (the jump) or by coming back down low (a ride down).
      const float alt = g_filter.altitude();
      if (mode == MODE_CLIMB && alt > AIRCRAFT_LATCH_ALT_M) g_inAircraft = true;
      if (mode == MODE_FREEFALL || alt < AIRCRAFT_CLEAR_ALT_M) g_inAircraft = false;

      // UNBUCKLE reminder, shown only on the way up.
      const bool unbuckle = g_inAircraft && mode != MODE_FREEFALL &&
                            alt >= CLIMB_UNBUCKLE_LO_M && alt <= CLIMB_UNBUCKLE_HI_M;
      display::setTopText(unbuckle ? "UNBUCKLE" : "");
#endif
    }
    else if (g_failStreak < SENSOR_FAIL_LIMIT)
    {
      g_failStreak++;
    }

    const bool settling = static_cast<int32_t>(now - g_filterSettledMs) < 0;
    if (fabsf(g_filter.altitude()) > IDLE_MAX_ALT_M ||
        (!settling && fabsf(g_filter.velocity()) > IDLE_MAX_VSPEED_MPS))
    {
      MARK_ACTIVE(now, fabsf(g_filter.altitude()) > IDLE_MAX_ALT_M
                       ? "altitude" : "velocity");
    }

#if AUTOZERO_ENABLED
    // Weather-drift correction. Returns metres to add to the ground reference,
    // almost always zero; see lib/altimeter_core/ground_ref.h for why the
    // settle time scales with altitude and why the in-flight latch exists.
    if (g_calibrated)
    {
      // Velocity is suppressed while the sensor settles, for the same reason
      // the idle gate above ignores it: the post-reset warm-up drift reads as
      // ~1 m/s, past AUTOZERO_SETTLED_MPS. Idle sleep resets the filter every
      // 30 s, so that transient restarted the settle timer before it could
      // ever reach the 3+ minutes required — auto-zero never ran at all, and
      // a standing offset stayed put indefinitely. Passing 0 leaves the
      // altitude-band test in ground_ref.cpp to detect a real pickup, which is
      // the check that actually matters while the device is sitting still.
      const float corr = g_groundRef.update(
          g_filter.altitude(), settling ? 0.0f : g_filter.velocity(), now);
      if (corr != 0.0f)
      {
        const float ga = baro::pressureAltitude(g_groundPHpa, g_qnhHpa) + corr;
        g_groundPHpa = baro::pressureAtAltitude(ga, g_qnhHpa);
      }
    }
#endif

#if BENCH_MODE
    const uint8_t phase = 0;
#else
    const uint8_t phase = static_cast<uint8_t>(g_modes.mode());
#endif
    logger::push(now, g_pressureHpa, g_tempC, g_rawAglM, g_filter.altitude(),
                 g_filter.velocity(), g_zones.zone(), phase, g_groundPHpa);
  }

  const LedPattern pattern = currentPattern();
  led::render(pattern, now);

  // While the demo is playing it owns the screen; the sensor keeps sampling and
  // logging underneath, so nothing about the live pipeline is disturbed.
  const bool demoRunning = demo::active() && demo::update(now);

  // Hand the renderer the same pattern the LED is showing, so both outputs
  // agree by construction. This is a spinlock-protected struct copy — the
  // sample loop never touches SPI.
  if (!demoRunning)
    display::publish(pattern, g_filter.altitude(), g_filter.velocity());

#if IDLE_SLEEP_ENABLED
#if IDLE_TRACE
  idleTrace(now);
#endif
  if (mayIdleSleep(now)) idleSleep();
#endif

  if (g_csvEnabled && static_cast<int32_t>(now - g_nextCsvMs) >= 0)
  {
    g_nextCsvMs = now + CSV_PERIOD_MS;
#if BENCH_MODE
    Serial.printf("%lu,%.3f,%.2f,%.3f,%.3f,%.2f,%.3f,%s\n",
                  (unsigned long)now, g_pressureHpa, g_tempC,
                  g_rawAglM, g_filter.altitude(), g_filter.velocity(),
                  g_filter.sigma(), zoneName(g_zones.zone()));
#else
    Serial.printf("%lu,%.3f,%.2f,%.3f,%.3f,%.2f,%.3f,%s,%s,%s\n",
                  (unsigned long)now, g_pressureHpa, g_tempC,
                  g_rawAglM, g_filter.altitude(), g_filter.velocity(),
                  g_filter.sigma(), zoneName(g_zones.zone()),
                  flightModeName(g_modes.mode()), landingName(g_landing.zone()));
#endif
  }
}
