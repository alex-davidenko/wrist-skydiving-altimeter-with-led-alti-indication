// ===========================================================================
//  Wrist-mounted skydiving altimeter — phase 1 firmware
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

#include <MS5611.h>

#include "altitude_filter.h"
#include "baro_math.h"
#include "config.h"
#include "display.h"
#include "flight_mode.h"
#include "led.h"
#include "logger.h"
#include "touch.h"
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

static bool zeroHere()
{
  Serial.println(F("\nZeroing — hold still..."));
  led::set(Rgb{255, 255, 255});

  double pSum = 0.0, aSum = 0.0, aSumSq = 0.0, aFirst = 0.0, aSecond = 0.0;
  uint16_t n = 0, nFirst = 0, nSecond = 0;

  for (uint16_t i = 0; i < ZERO_SAMPLE_COUNT; i++)
  {
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
  g_zones.reset(0.0f);
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
//  Button (BOOT): long press = re-zero
// ---------------------------------------------------------------------------
static void pollButton(uint32_t nowMs)
{
  static bool     wasDown  = false;
  static uint32_t downAt   = 0;
  static bool     fired    = false;

  const int raw = digitalRead(PIN_BUTTON);
  const bool down = BUTTON_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);

  if (down && !wasDown)
  {
    downAt = nowMs;
    fired  = false;
  }
  else if (down && !fired && (nowMs - downAt) >= ZERO_BUTTON_HOLD_MS)
  {
    fired = true;
    zeroHere();
    resyncSampleClock();
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
  Serial.printf("flight phase  : %s\n", flightModeName(g_modes.mode()));
  Serial.printf("landing band  : %s\n", landingName(g_landing.zone()));
#endif
  Serial.printf("fail streak   : %u   loop overruns: %u\n", g_failStreak, (unsigned)g_overruns);
  Serial.printf("LED brightness: %u\n", led::brightness());
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
  Serial.begin(115200);
  delay(300);

  led::begin();
  led::selfTest();

  display::begin();
  display::message("Altimeter", BENCH_MODE ? "BENCH mode" : "FLIGHT mode");

  pinMode(PIN_BUTTON, BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);

  Serial.println(F("\n\n=== Skydiving altimeter — phase 1 ==="));
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

  g_prefs.begin("altimeter", false);
  loadCalibration();

  g_filter.configureAdaptive(FILTER_GATE_SIGMA, FILTER_MAX_INFLATE);
  g_zones.begin(kZoneConfig);
#if !BENCH_MODE
  g_landing.begin(kLandingConfig);
  g_modes.begin({MODE_FREEFALL_ENTER_MPS, MODE_FREEFALL_EXIT_MPS,
                 MODE_CLIMB_ENTER_MPS, MODE_CLIMB_EXIT_MPS, MODE_DWELL_MS});
  g_climb.configure(CLIMB_MARK_INTERVAL_M);
#endif

  if (g_calibrated)
    Serial.printf("Restored ground ref %.3f hPa from flash — RE-ZERO before use.\n", g_groundPHpa);
  else
    Serial.println(F("No ground reference stored. Press 'z' or hold BOOT for 1.5 s."));

  // Prime the filter so the first zone decision is not made from a cold start.
  for (int i = 0; i < 8; i++)
  {
    if (sensorRead()) g_rawAglM = aglFromPressure(g_pressureHpa);
  }
  checkPressurePlausible();
  g_filter.reset(g_rawAglM);
  g_zones.reset(g_rawAglM);
#if !BENCH_MODE
  g_landing.reset(g_rawAglM);
  g_modes.reset(MODE_CANOPY);
  g_climb.update(g_rawAglM, false);
#endif

  display::startTask();

  logger::begin(g_qnhHpa, g_groundPHpa);
  logger::startTask();

  touch::begin();

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

// Which pattern should the LED be showing right now?
static LedPattern currentPattern()
{
  if (g_failStreak >= SENSOR_FAIL_LIMIT) return led::faultPattern();

#if BENCH_MODE
  return led::freefallPattern(g_zones.zone());
#else
  switch (g_modes.mode())
  {
    case MODE_FREEFALL:
      return led::freefallPattern(g_zones.zone());

    // Under canopy the freefall colours mean nothing — a good canopy at 900 m
    // does not warrant a red light. The landing ladder takes over, and it is
    // dark above 300 m and below 10 m, so the LED stays out of the way for the
    // whole middle of the canopy ride.
    case MODE_CANOPY:
      return led::landingPattern(g_landing.zone());

    // Climbing: dark, except the one-shot green flash every 100 m gained.
    case MODE_CLIMB:
    default:
      return led::offPattern();
  }
#endif
}

void loop()
{
  const uint32_t now = millis();

  pollSerial();
  pollButton(now);

  if (touch::available())
  {
    const touch::Point t = touch::takeTouch();
    if (t.valid && g_touchDump)
    {
      int16_t rx, ry;
      touch::rawLast(&rx, &ry);
      Serial.printf("touch raw=(%4d,%4d) -> mapped=(%4d,%4d)\n", rx, ry, t.x, t.y);
    }
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
#endif
    }
    else if (g_failStreak < SENSOR_FAIL_LIMIT)
    {
      g_failStreak++;
    }

#if BENCH_MODE
    const uint8_t phase = 0;
#else
    const uint8_t phase = static_cast<uint8_t>(g_modes.mode());
#endif
    logger::push(now, g_pressureHpa, g_tempC, g_rawAglM, g_filter.altitude(),
                 g_filter.velocity(), g_zones.zone(), phase);
  }

  const LedPattern pattern = currentPattern();
  led::render(pattern, now);

  // Hand the renderer the same pattern the LED is showing, so both outputs
  // agree by construction. This is a spinlock-protected struct copy — the
  // sample loop never touches SPI.
  display::publish(pattern, g_filter.altitude(), g_filter.velocity());

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
