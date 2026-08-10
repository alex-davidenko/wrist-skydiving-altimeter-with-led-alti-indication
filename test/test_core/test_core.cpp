// Host-side tests for the parts that are miserable to debug on hardware:
// the barometric maths, the Kalman filter's lag/noise behaviour, and the
// zone state machine's flicker resistance.
//
//   .venv/bin/pio test -e native

#include <unity.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>

#include "altitude_filter.h"
#include "baro_math.h"
#include "flight_mode.h"
#include "ground_ref.h"
#include "velocity_window.h"
#include "zones.h"

// Deterministic pseudo-noise so failures are reproducible.
static uint32_t s_rng = 12345;
static float noise(float sigma)
{
  // Irwin-Hall(4) approximation of a normal, mean 0, adjusted to unit sigma.
  float acc = 0.0f;
  for (int i = 0; i < 4; i++)
  {
    s_rng = s_rng * 1664525u + 1013904223u;
    acc += static_cast<float>((s_rng >> 8) & 0xFFFF) / 65535.0f - 0.5f;
  }
  return acc * sigma / 0.5773f;
}

// Zone configs mirroring config.h.
// Bench keeps symmetric damping; flight is urgent-asymmetric. Mirrors config.h.
static const ZoneConfig kBench = {
    {0.15f, 0.80f, 1.20f, 1.50f, 4.50f}, 0.07f, 0.07f, 300, 300};
static const ZoneConfig kFlight = {
    {kZoneBoundDisabledLow, 800.0f, 1200.0f, 1500.0f, 4500.0f}, 0.0f, 8.0f, 50, 400};
static const ZoneConfig kLanding = {
    {10.0f, 100.0f, 200.0f, 300.0f, kZoneBoundDisabledHigh}, 2.0f, 8.0f, 100, 400};
static const FlightModeConfig kModes = {20.0f, 15.0f, 2.0f, 0.5f, 400};

// ---------------------------------------------------------------------------
//  Barometric maths
// ---------------------------------------------------------------------------
static void test_isa_reference_points()
{
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, baro::pressureAltitude(1013.25f));
  // ISA table: 1000 m -> 898.75 hPa, 5000 m -> 540.48 hPa.
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 1000.0f, baro::pressureAltitude(898.75f));
  TEST_ASSERT_FLOAT_WITHIN(15.0f, 5000.0f, baro::pressureAltitude(540.48f));
}

static void test_sea_level_pressure_roundtrip()
{
  const float p = 850.0f;
  const float h = baro::pressureAltitude(p);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, baro::kISASeaLevel_hPa, baro::seaLevelPressure(p, h));
}

static void test_agl_is_qnh_insensitive()
{
  // AGL is a difference of two pressure altitudes, so a QNH error should
  // barely move it. This is why the firmware stores ground *pressure*.
  const float pGround = 1000.0f, pAir = 900.0f;
  const float aglIsa = baro::pressureAltitude(pAir, 1013.25f) -
                       baro::pressureAltitude(pGround, 1013.25f);
  const float aglOff = baro::pressureAltitude(pAir, 1030.0f) -
                       baro::pressureAltitude(pGround, 1030.0f);
  TEST_ASSERT_FLOAT_WITHIN(20.0f, aglIsa, aglOff);
}

// ---------------------------------------------------------------------------
//  Filter
// ---------------------------------------------------------------------------
// Bench tuning must pull the MS5611's ~25 cm of raw noise down below the 7 cm
// hysteresis band, otherwise the LED chatters on the desk. Measured: 0.037 m
// RMS, a ~7x improvement. RMS is the meaningful bound here — isolated peaks
// still reach ~0.2 m and are the dwell timer's job, and it is the end-to-end
// chatter test below that actually proves the LED holds still.
static void test_filter_suppresses_bench_noise()
{
  AltitudeFilter f(0.05f, 0.25f);
  f.configureAdaptive(2.0f, 500.0f);
  f.reset(1.0f);

  double sq = 0.0;
  int n = 0;
  for (int i = 0; i < 4000; i++)
  {
    f.update(1.0f + noise(0.25f), 0.025f);
    if (i > 400)   // let it settle
    {
      const float err = f.altitude() - 1.0f;
      sq += err * err;
      n++;
    }
  }
  const float rms = std::sqrt(sq / n);
  TEST_ASSERT_TRUE_MESSAGE(rms < 0.05f, "bench filter RMS noise too high");
}

// The constant-velocity model coasts when real motion stops. Without the
// innovation-bias detector this overshoot took ~4 s to bleed off, which is
// longer than the dwell timer and would show a wrong zone. Assert it now
// settles inside the dwell window.
static void test_filter_recovers_quickly_after_lift_and_hold()
{
  AltitudeFilter f(0.05f, 0.25f);
  f.configureAdaptive(2.0f, 500.0f);
  f.reset(0.0f);

  const float dt = 0.025f, rampS = 0.5f, top = 1.0f;
  float settle = -1.0f;
  for (int i = 0; i < 4000; i++)
  {
    const float t = i * dt;
    const float truth = (t < rampS) ? (t / rampS) * top : top;
    f.update(truth + noise(0.25f), dt);
    if (t > rampS + 0.1f && settle < 0.0f && std::fabs(f.altitude() - top) < 0.05f)
      settle = t - rampS;
  }
  TEST_ASSERT_TRUE_MESSAGE(settle > 0.0f && settle < 0.25f,
                           "lift-and-hold overshoot outlives the dwell window");
}

// Canopy opening: 50 m/s -> 5 m/s over 2.5 s. The filter must not lag badly
// through the deceleration, or the LED reports too low right at the moment
// the jumper is checking it.
static void test_filter_tracks_canopy_opening()
{
  AltitudeFilter f(3.0f, 0.60f);
  f.configureAdaptive(2.0f, 500.0f);

  const float dt = 0.025f, decelS = 2.5f;
  float truth = 1500.0f;
  f.reset(truth);
  for (int i = 0; i < 200; i++) { truth -= 50.0f * dt; f.update(truth + noise(0.6f), dt); }

  float worst = 0.0f;
  for (int i = 0; i < 2400; i++)
  {
    const float t = i * dt;
    const float v = (t < decelS) ? (-50.0f + 45.0f * (t / decelS)) : -5.0f;
    truth += v * dt;
    f.update(truth + noise(0.6f), dt);
    const float err = std::fabs(f.altitude() - truth);
    if (err > worst) worst = err;
  }
  TEST_ASSERT_TRUE_MESSAGE(worst < 3.0f, "canopy-opening tracking error too large");
}

// End-to-end: sensor noise -> filter -> zone tracker, parked at a range of
// heights. This is the real acceptance criterion for "the LED must not
// flicker", and it exercises the filter, hysteresis and dwell together.
//
// Note the criterion is the *gap* between zone changes, not zero changes.
// Parked exactly on a threshold the device genuinely cannot tell which side
// it is on, so an occasional slow change is physically unavoidable and looks
// fine; what must never happen is rapid chatter.
static void test_no_led_chatter_when_parked()
{
  float worstGap = 1e9f;

  for (int step = 0; step <= 40; step++)
  {
    const float truth = step * 0.05f;    // 0.00 .. 2.00 m

    s_rng = 99;
    AltitudeFilter f(0.05f, 0.25f);
    f.configureAdaptive(2.0f, 500.0f);
    ZoneTracker t;
    t.begin(kBench);
    f.reset(truth);
    t.reset(truth);

    uint32_t now = 0, lastChange = 0;
    uint8_t prev = t.zone();
    for (int i = 0; i < 4000; i++)       // 100 s at each height
    {
      now += 25;
      f.update(truth + noise(0.25f), 0.025f);
      const uint8_t z = t.update(f.altitude(), now);
      if (i > 400 && z != prev)
      {
        if (lastChange)
        {
          const float gap = (now - lastChange) / 1000.0f;
          if (gap < worstGap) worstGap = gap;
        }
        lastChange = now;
        prev = z;
      }
    }
  }

  TEST_ASSERT_TRUE_MESSAGE(worstGap > 2.0f, "LED chattered while parked");
}

// Every bench zone must be reachable and stable when the board is held at the
// middle of the band, regardless of whether it was approached from above or
// below. This is the test that maps directly onto the manual bench procedure.
static void test_every_bench_zone_is_reachable_from_both_directions()
{
  struct Case { float height; uint8_t expect; };
  const Case cases[] = {
      {0.05f, ZONE_OFF},
      {0.50f, ZONE_BLINK_RED},
      {0.95f, ZONE_RED},
      {1.35f, ZONE_YELLOW},
      {2.00f, ZONE_GREEN},
  };

  for (const Case &c : cases)
  {
    for (int dir = 0; dir < 2; dir++)
    {
      const float start = dir ? 2.6f : 0.0f;

      s_rng = 7;
      AltitudeFilter f(0.05f, 0.25f);
      f.configureAdaptive(2.0f, 500.0f);
      ZoneTracker t;
      t.begin(kBench);
      f.reset(start);
      t.reset(start);

      uint32_t now = 0;
      const float dt = 0.025f;
      for (int i = 0; i < 4000; i++)     // 1 s move, then hold
      {
        const float tt = i * dt;
        now += 25;
        const float truth = (tt < 1.0f) ? start + (c.height - start) * tt : c.height;
        f.update(truth + noise(0.25f), dt);
        t.update(f.altitude(), now);
      }
      TEST_ASSERT_EQUAL_MESSAGE(c.expect, t.zone(),
                                "bench zone not reached/held from this direction");
    }
  }
}

static void test_filter_tracks_freefall_without_lag()
{
  // 50 m/s descent. A constant-velocity Kalman should have no steady-state
  // lag on a ramp — this is the whole reason for not using a moving average.
  AltitudeFilter f(3.0f, 0.60f);
  const float v = -50.0f, dt = 0.025f;
  float truth = 4000.0f;
  f.reset(truth);

  float worst = 0.0f;
  for (int i = 0; i < 2400; i++)   // 60 s
  {
    truth += v * dt;
    f.update(truth + noise(0.60f), dt);
    if (i > 200)
    {
      const float err = std::fabs(f.altitude() - truth);
      if (err > worst) worst = err;
    }
  }
  TEST_ASSERT_TRUE_MESSAGE(worst < 3.0f, "freefall tracking error too large");
  TEST_ASSERT_FLOAT_WITHIN(2.0f, -50.0f, f.velocity());
}

static void test_filter_rejects_bad_input()
{
  AltitudeFilter f(3.0f, 0.6f);
  f.reset(100.0f);
  f.update(NAN, 0.025f);
  TEST_ASSERT_TRUE(std::isfinite(f.altitude()));
  f.update(100.0f, -1.0f);          // bad dt must not poison the state
  TEST_ASSERT_TRUE(std::isfinite(f.altitude()));
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, f.altitude());
}


// Regression: a sharp move must change the LED promptly.
//
// Reported from the bench: dropping the board from 2.2 m into the yellow band
// took ~1.5 s to change colour. Cause was the dwell clock restarting on every
// candidate change. The filter briefly undershoots to ~1.05 m (inside RED), so
// the candidate reads GREEN -> RED -> YELLOW while settling, and settling time
// and dwell compounded instead of overlapping.
static void test_sharp_move_changes_zone_promptly()
{
  const float from = 2.20f, to = 1.35f;

  for (float moveS : {0.15f, 0.30f, 0.50f})
  {
    s_rng = 31337;
    AltitudeFilter f(0.05f, 0.25f);
    f.configureAdaptive(2.0f, 500.0f);
    ZoneTracker t;
    t.begin(kBench);
    f.reset(from);
    t.reset(from);

    uint32_t now = 0;
    const float dt = 0.025f;
    float committed = -1.0f;
    bool sawWrongZone = false;

    for (int i = 0; i < 400; i++)
    {
      const float tt = i * dt;
      now += 25;
      const float truth = (tt < moveS) ? from + (to - from) * (tt / moveS) : to;
      f.update(truth + noise(0.25f), dt);
      const uint8_t z = t.update(f.altitude(), now);

      // It must never flash RED on the way — the undershoot is the filter's,
      // and the dwell timer exists precisely to hide it.
      if (z != ZONE_GREEN && z != ZONE_YELLOW) sawWrongZone = true;
      if (committed < 0.0f && z == ZONE_YELLOW) committed = tt;
      if (committed > 0.0f && z != ZONE_YELLOW) committed = -1.0f;
    }

    TEST_ASSERT_FALSE_MESSAGE(sawWrongZone, "flashed a wrong zone during the move");
    TEST_ASSERT_TRUE_MESSAGE(committed > 0.0f && committed < 0.80f,
                             "sharp move took too long to change zone");
  }
}

// ---------------------------------------------------------------------------
//  Zone state machine
// ---------------------------------------------------------------------------
static void test_plain_zone_mapping()
{
  ZoneTracker t;
  t.begin(kFlight);
  TEST_ASSERT_EQUAL(ZONE_BLINK_RED, t.zoneFor(500.0f));
  TEST_ASSERT_EQUAL(ZONE_RED,       t.zoneFor(1000.0f));
  TEST_ASSERT_EQUAL(ZONE_YELLOW,    t.zoneFor(1300.0f));
  TEST_ASSERT_EQUAL(ZONE_GREEN,     t.zoneFor(3000.0f));
  TEST_ASSERT_EQUAL(ZONE_ABOVE,     t.zoneFor(5000.0f));
  // Off band is disabled in flight config — never reachable.
  TEST_ASSERT_EQUAL(ZONE_BLINK_RED, t.zoneFor(0.0f));
}

static void test_bench_thresholds_match_spec()
{
  ZoneTracker t;
  t.begin(kBench);
  TEST_ASSERT_EQUAL(ZONE_OFF,       t.zoneFor(0.05f));
  TEST_ASSERT_EQUAL(ZONE_BLINK_RED, t.zoneFor(0.50f));
  TEST_ASSERT_EQUAL(ZONE_RED,       t.zoneFor(0.95f));
  TEST_ASSERT_EQUAL(ZONE_YELLOW,    t.zoneFor(1.35f));
  TEST_ASSERT_EQUAL(ZONE_GREEN,     t.zoneFor(2.00f));
  // Bench bounds are the flight bounds / 1000.
  TEST_ASSERT_EQUAL(ZONE_GREEN,     t.zoneFor(1.50f * 1.001f));
}

// With urgent-asymmetric damping, an altitude parked on a threshold settles
// into the LOWER (more urgent) band and stays there. That bias is intentional
// for a safety indicator. What must still never happen is chatter.
static void test_boundary_noise_settles_low_without_chatter()
{
  ZoneTracker t;
  t.begin(kFlight);
  t.reset(1250.0f);                       // yellow

  uint32_t now = 0, lastChange = 0;
  float worstGap = 1e9f;
  uint8_t prev = t.zone();
  for (int i = 0; i < 8000; i++)          // 200 s straddling the 1200 m line
  {
    now += 25;
    const uint8_t z = t.update(1200.5f + noise(1.5f), now);
    if (z != prev)
    {
      if (lastChange)
      {
        const float gap = (now - lastChange) / 1000.0f;
        if (gap < worstGap) worstGap = gap;
      }
      lastChange = now;
      prev = z;
    }
  }
  TEST_ASSERT_EQUAL_MESSAGE(ZONE_RED, t.zone(), "should settle to the more urgent band");
  TEST_ASSERT_TRUE_MESSAGE(worstGap > 2.0f, "chattered on the boundary");
}

static void test_dwell_rejects_single_sample_spike()
{
  ZoneTracker t;
  t.begin(kFlight);
  t.reset(1300.0f);

  uint32_t now = 0;
  // One wild outlier, well past the hysteresis band, then back to normal.
  now += 25; t.update(1300.0f, now);
  now += 25; TEST_ASSERT_EQUAL(ZONE_YELLOW, t.update(400.0f, now));
  now += 25; TEST_ASSERT_EQUAL(ZONE_YELLOW, t.update(1300.0f, now));
}

static void test_sustained_change_does_commit()
{
  ZoneTracker t;
  t.begin(kFlight);
  t.reset(1300.0f);

  uint32_t now = 0;
  for (int i = 0; i < 20; i++)   // 500 ms, comfortably past the 120 ms dwell
  {
    now += 25;
    t.update(1000.0f, now);
  }
  TEST_ASSERT_EQUAL(ZONE_RED, t.zone());
}

static void test_multi_zone_jump_in_one_step()
{
  // Freefall at 50 m/s covers 1.25 m per sample, but after a dropout the
  // altitude can move several zones between updates.
  ZoneTracker t;
  t.begin(kFlight);
  t.reset(4000.0f);

  uint32_t now = 0;
  for (int i = 0; i < 20; i++)
  {
    now += 25;
    t.update(600.0f, now);
  }
  TEST_ASSERT_EQUAL(ZONE_BLINK_RED, t.zone());
}

static void test_descent_visits_every_zone_in_order()
{
  ZoneTracker t;
  t.begin(kFlight);
  t.reset(4400.0f);

  uint8_t seen[8];
  int  n = 0;
  seen[n++] = t.zone();

  uint32_t now = 0;
  float alt = 4400.0f;
  const float dt = 0.025f;
  while (alt > 100.0f)
  {
    alt -= 50.0f * dt;
    now += 25;
    const uint8_t z = t.update(alt + noise(0.5f), now);
    if (z != seen[n - 1] && n < 8) seen[n++] = z;
  }

  TEST_ASSERT_EQUAL(4, n);
  TEST_ASSERT_EQUAL(ZONE_GREEN,     seen[0]);
  TEST_ASSERT_EQUAL(ZONE_YELLOW,    seen[1]);
  TEST_ASSERT_EQUAL(ZONE_RED,       seen[2]);
  TEST_ASSERT_EQUAL(ZONE_BLINK_RED, seen[3]);
}

// The threshold overshoot at 50 m/s is what actually matters for a jumper:
// hysteresis + dwell must not delay the red warning by a dangerous margin.
static void test_threshold_lag_at_freefall_speed()
{
  ZoneTracker t;
  t.begin(kFlight);
  t.reset(1400.0f);

  uint32_t now = 0;
  float alt = 1400.0f;
  const float dt = 0.025f;
  float crossedAt = 0.0f;
  while (alt > 900.0f)
  {
    alt -= 50.0f * dt;
    now += 25;
    if (t.update(alt, now) == ZONE_RED && crossedAt == 0.0f) crossedAt = alt;
  }
  // Boundary is 1200 m. With urgent-asymmetric damping the only cost is the
  // 50 ms dwell (2.5 m at 50 m/s) plus one sample period, so this must now
  // trigger within a few metres. Symmetric damping cost 11 m.
  TEST_ASSERT_TRUE_MESSAGE(crossedAt > 1190.0f, "red zone triggered too late");
  TEST_ASSERT_TRUE_MESSAGE(crossedAt < 1200.0f, "red zone triggered early");
}


// ---------------------------------------------------------------------------
//  Flight phase / landing ladder
// ---------------------------------------------------------------------------
static void test_landing_ladder_bands()
{
  ZoneTracker t;
  t.begin(kLanding);
  TEST_ASSERT_EQUAL(LAND_DARK_HIGH, t.zoneFor(350.0f));
  TEST_ASSERT_EQUAL(LAND_SLOW,      t.zoneFor(250.0f));
  TEST_ASSERT_EQUAL(LAND_FAST,      t.zoneFor(150.0f));
  TEST_ASSERT_EQUAL(LAND_STEADY,    t.zoneFor(50.0f));
  TEST_ASSERT_EQUAL(LAND_DARK_LOW,  t.zoneFor(5.0f));
}

static void test_flight_mode_transitions_with_hysteresis()
{
  FlightModeTracker m;
  m.begin(kModes);
  m.reset(MODE_CANOPY);

  uint32_t now = 0;
  auto hold = [&](float v, int ms) {
    for (int i = 0; i < ms / 25; i++) { now += 25; m.update(v, now); }
  };

  hold(+5.0f, 2000);   TEST_ASSERT_EQUAL(MODE_CLIMB,    m.mode());  // aircraft
  hold(-50.0f, 2000);  TEST_ASSERT_EQUAL(MODE_FREEFALL, m.mode());  // exit
  hold(-5.0f, 2000);   TEST_ASSERT_EQUAL(MODE_CANOPY,   m.mode());  // open

  // 17 m/s sits between the 15 m/s exit and 20 m/s enter thresholds, so it
  // must NOT drag a canopy descent back into freefall.
  hold(-17.0f, 3000);  TEST_ASSERT_EQUAL(MODE_CANOPY,   m.mode());
}

// The case that matters: a high-speed malfunction is still descending fast, so
// the freefall colour zones stay in charge and keep warning. It must not be
// mistaken for a canopy ride and go dark.
static void test_high_speed_malfunction_keeps_freefall_warnings()
{
  FlightModeTracker m;
  m.begin(kModes);
  m.reset(MODE_FREEFALL);
  ZoneTracker z;
  z.begin(kFlight);
  z.reset(1500.0f);

  float alt = 1500.0f;
  const float dt = 0.025f, v = -45.0f;     // streamer / bag lock
  uint32_t now = 0;
  bool sawBlinkRed = false;

  while (alt > 400.0f)
  {
    alt += v * dt;
    now += 25;
    TEST_ASSERT_EQUAL_MESSAGE(MODE_FREEFALL, m.update(v, now),
                              "a high-speed mal must stay in FREEFALL");
    const uint8_t band = z.update(alt, now);
    if (alt < 780.0f && band == ZONE_BLINK_RED) sawBlinkRed = true;
  }
  TEST_ASSERT_TRUE_MESSAGE(sawBlinkRed, "no blinking red during a high-speed mal");
}

static void test_climb_marker_fires_every_100m()
{
  ClimbMarker c;
  c.configure(100.0f);
  c.update(0.0f, false);

  int flashes = 0;
  for (float alt = 0.0f; alt <= 4000.0f; alt += 0.125f)   // 5 m/s at 40 Hz
  {
    if (c.update(alt, true)) flashes++;
  }
  TEST_ASSERT_EQUAL_MESSAGE(40, flashes, "expected one flash per 100 m to 4000 m");

  // Descending must not fire, and must not re-arm a repeat on the way back up.
  for (float alt = 4000.0f; alt >= 3500.0f; alt -= 0.125f) c.update(alt, false);
  int spurious = 0;
  for (float alt = 3500.0f; alt <= 3900.0f; alt += 0.125f)
    if (c.update(alt, true)) spurious++;
  TEST_ASSERT_EQUAL_MESSAGE(4, spurious, "re-climbing should mark each new 100 m once");
}

// A complete jump through the real filter and every tracker: climb to 4000 m,
// exit, freefall, opening, canopy descent, landing, touchdown. This is the only
// test that exercises the whole chain the way an actual jump does.
static void test_full_jump_profile()
{
  s_rng = 20260801;

  AltitudeFilter f(3.0f, 0.60f);
  f.configureAdaptive(2.0f, 500.0f);
  ZoneTracker zones, landing;
  FlightModeTracker modes;
  ClimbMarker climb;

  zones.begin(kFlight);
  landing.begin(kLanding);
  modes.begin(kModes);
  climb.configure(100.0f);

  float alt = 0.0f;
  f.reset(alt);
  zones.reset(alt);
  landing.reset(alt);
  modes.reset(MODE_CANOPY);
  climb.update(alt, false);

  const float dt = 0.025f;
  uint32_t now = 0;
  int climbFlashes = 0;

  // Observations captured as the jump passes through each altitude.
  uint8_t ffAt1400 = 255, ffAt1100 = 255;
  uint8_t canopyBandAt900 = 255, at250 = 255, at150 = 255, at50 = 255, at5 = 255;
  FlightMode modeAt900 = MODE_COUNT, modeAt3000 = MODE_COUNT;

  enum Phase { CLIMB, LEVEL, FREEFALL, OPENING, CANOPY, DOWN } phase = CLIMB;
  float v = 5.0f, phaseT = 0.0f;

  for (int i = 0; i < 60000 && phase != DOWN; i++)
  {
    // --- truth trajectory ---
    switch (phase)
    {
      case CLIMB:
        v = 5.0f;
        if (alt >= 4000.0f) { phase = LEVEL; phaseT = 0.0f; }
        break;
      case LEVEL:
        v = 0.0f;
        if ((phaseT += dt) > 5.0f) { phase = FREEFALL; phaseT = 0.0f; }
        break;
      case FREEFALL:
        v -= 9.8f * dt;                       // accelerate to terminal
        if (v < -50.0f) v = -50.0f;
        if (alt <= 1000.0f) { phase = OPENING; phaseT = 0.0f; }
        break;
      case OPENING:
        phaseT += dt;
        v = -50.0f + 45.0f * (phaseT / 3.0f); // 50 -> 5 m/s over 3 s
        if (phaseT >= 3.0f) { phase = CANOPY; v = -5.0f; }
        break;
      case CANOPY:
        v = -5.0f;
        if (alt <= 0.0f) phase = DOWN;
        break;
      default: break;
    }
    alt += v * dt;
    now += 25;

    // --- the real signal chain ---
    f.update(alt + noise(0.60f), dt);
    const uint8_t zBand = zones.update(f.altitude(), now);
    const uint8_t lBand = landing.update(f.altitude(), now);
    const FlightMode m  = modes.update(f.velocity(), now);
    if (climb.update(f.altitude(), m == MODE_CLIMB)) climbFlashes++;

    // --- observations ---
    if (phase == CLIMB && alt > 2990.0f && alt < 3010.0f) modeAt3000 = m;
    if (phase == FREEFALL && alt < 1400.0f && ffAt1400 == 255) ffAt1400 = zBand;
    if (phase == FREEFALL && alt < 1100.0f && ffAt1100 == 255) ffAt1100 = zBand;
    if (phase == CANOPY && alt < 900.0f && canopyBandAt900 == 255)
    { canopyBandAt900 = lBand; modeAt900 = m; }
    if (phase == CANOPY && alt < 250.0f && at250 == 255) at250 = lBand;
    if (phase == CANOPY && alt < 150.0f && at150 == 255) at150 = lBand;
    if (phase == CANOPY && alt <  50.0f && at50  == 255) at50  = lBand;
    if (phase == CANOPY && alt <   5.0f && at5   == 255) at5   = lBand;
  }

  TEST_ASSERT_EQUAL_MESSAGE(MODE_CLIMB, modeAt3000, "should read CLIMB in the aircraft");
  TEST_ASSERT_TRUE_MESSAGE(climbFlashes >= 38 && climbFlashes <= 41,
                           "expected ~40 climb flashes on the way to 4000 m");

  TEST_ASSERT_EQUAL_MESSAGE(ZONE_YELLOW, ffAt1400, "freefall at 1400 m should be yellow");
  TEST_ASSERT_EQUAL_MESSAGE(ZONE_RED,    ffAt1100, "freefall at 1100 m should be red");

  // The whole point of the canopy mode: a good canopy at 900 m must be dark,
  // not screaming RED the way the freefall ladder alone would.
  TEST_ASSERT_EQUAL_MESSAGE(MODE_CANOPY, modeAt900, "should be CANOPY after opening");
  TEST_ASSERT_EQUAL_MESSAGE(LAND_DARK_HIGH, canopyBandAt900, "LED must be dark at 900 m under canopy");

  TEST_ASSERT_EQUAL_MESSAGE(LAND_SLOW,     at250, "250 m should be 3 blinks/s");
  TEST_ASSERT_EQUAL_MESSAGE(LAND_FAST,     at150, "150 m should be 6 blinks/s");
  TEST_ASSERT_EQUAL_MESSAGE(LAND_STEADY,   at50,  "50 m should be steady green");
  TEST_ASSERT_EQUAL_MESSAGE(LAND_DARK_LOW, at5,   "below 10 m assistance should stop");
}


// ---------------------------------------------------------------------------
//  Ground-reference auto-correction (weather drift)
// ---------------------------------------------------------------------------
static const GroundRefConfig kGref = {
    0.5f,      // settled below 0.5 m/s
    1.0f,      // must hold within 1 m — a 3 m walk upstairs restarts the timer
    3.0f,      // base minutes
    0.05f,     // per-metre minutes
    0.5f,      // slew m/min: 17x faster than real drift needs, and slow enough
               // that a 5-minute excursion moves the reference under a metre
    1.0f,      // sustained 1 m/s sets the in-flight latch
    150.0f,    // clear below 150 m
    120000     // after 2 min stationary
};

// Drive the tracker for `secs` at a fixed altitude and speed, applying the
// corrections it asks for. Returns the total correction applied.
static float runGref(GroundRef &g, float alt, float vs, float secs,
                     uint32_t *clock, bool moveAltWithCorrection = true)
{
  float applied = 0.0f;
  for (float t = 0; t < secs; t += 0.5f)
  {
    *clock += 500;
    const float c = g.update(alt, vs, *clock);
    applied += c;
    if (moveAltWithCorrection) alt -= c;   // correcting the reference lowers AGL
  }
  return applied;
}

// A periodic velocity transient must not starve the settle timer.
//
// Idle sleep resets the filter every 30 s, and the sensor's warm-up drift then
// reads as ~1 m/s for about a second — measured on hardware, and larger than
// the 0.5 m/s settled threshold, so it lands in the settle-restart branch. With
// a 3-minute settle requirement and a 30 s wake period the timer can never
// mature: auto-zero stops running entirely and a standing offset lives forever,
// which is exactly what a -1 m reading that never came back did. main.cpp now
// suppresses the velocity while the filter settles; this is why it has to.
static void test_periodic_wake_transient_does_not_starve_autozero()
{
  GroundRef g;
  g.begin(kGref);
  uint32_t clock = 0;
  g.reset(clock);

  // 20 minutes sitting still at 1 m, with the transient 1 s in every 30.
  float alt     = 1.0f;
  float applied = 0.0f;
  for (int i = 0; i < 20 * 60 * 2; ++i)          // 0.5 s steps
  {
    clock += 500;
    const bool spike = (clock % 30000) < 1000;
    const float c = g.update(alt, spike ? 1.02f : 0.0f, clock);
    applied += c;
    alt -= c;
  }
  TEST_ASSERT_EQUAL_FLOAT(0.0f, applied);        // starved: never corrects at all
  TEST_ASSERT_FALSE(g.correcting());

  // Suppressed the way the firmware now does it, the same 20 minutes converge.
  GroundRef g2;
  g2.begin(kGref);
  clock = 0;
  g2.reset(clock);
  applied = runGref(g2, 1.0f, 0.0f, 20 * 60, &clock);
  TEST_ASSERT_TRUE(g2.correcting());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, applied);
}


// ---------------------------------------------------------------------------
//  Velocity window and the aircraft latch
// ---------------------------------------------------------------------------
// The filter's velocity state is unusable in freefall: measured noise rises to
// 9.5-16.5 m RMS, and differencing adjacent samples turns 10 m over 25 ms into
// 400 m/s. Logged peaks hit 2326 m/s with 38-45% of freefall samples reading as
// a CLIMB. A window works because the signal grows with it and the noise does
// not. This feeds the worst measured noise at the real freefall rate.
static void test_velocity_window_survives_freefall_noise()
{
  VelocityWindow w;
  w.begin(1500, 96);
  s_rng = 4242;

  float alt = 3500.0f;
  uint32_t t = 0;
  int falseClimb = 0, n = 0;
  float worst = 0.0f;
  for (int i = 0; i < 2000; i++)          // 50 s at 40 Hz
  {
    alt -= 50.0f * 0.025f;
    t += 25;
    const float v = w.update(alt + noise(16.5f), t);
    if (!w.ready()) continue;
    n++;
    if (v > 2.0f) falseClimb++;           // would latch CLIMB
    const float err = std::fabs(v - (-50.0f));
    if (err > worst) worst = err;
  }
  TEST_ASSERT_TRUE_MESSAGE(n > 1800, "window never became ready");
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, falseClimb,
                                "freefall noise still reads as a climb");
  TEST_ASSERT_TRUE_MESSAGE(worst < 25.0f, "windowed velocity error too large");
}

// Sitting still must not invent motion, and ready() must not be true before the
// window has actually spanned its own duration — an empty window reads zero,
// which looks exactly like standing still.
static void test_velocity_window_quiet_and_ready()
{
  VelocityWindow w;
  w.begin(1500, 96);
  TEST_ASSERT_FALSE(w.ready());
  uint32_t t = 0;
  for (int i = 0; i < 20; i++) { w.update(1.0f + noise(0.22f), t); t += 25; }
  TEST_ASSERT_FALSE_MESSAGE(w.ready(), "ready before the window is spanned");
  for (int i = 0; i < 200; i++) { w.update(1.0f + noise(0.22f), t); t += 25; }
  TEST_ASSERT_TRUE(w.ready());
  TEST_ASSERT_TRUE_MESSAGE(std::fabs(w.velocity()) < 1.0f,
                           "invented motion while stationary");
}

// The bug jump 1 actually showed: the zone ladder was perfectly correct while
// the screen stayed blue, because one committed CLIMB during the descent
// re-armed the aircraft latch and it only released on FREEFALL or below 100 m.
static void test_aircraft_latch_ignores_a_climb_during_descent()
{
  AircraftLatch l;
  l.begin(300.0f, 100.0f, 150.0f);

  for (float a = 0; a < 3500; a += 50) l.update(a, MODE_CLIMB);
  TEST_ASSERT_TRUE_MESSAGE(l.inAircraft(), "latch never armed on the way up");

  l.update(3400.0f, MODE_FREEFALL);
  TEST_ASSERT_FALSE(l.inAircraft());

  // Under canopy at 1200 m, a riser input or a thermal reads as a real climb.
  for (int i = 0; i < 40; i++) l.update(1200.0f, MODE_CLIMB);
  TEST_ASSERT_FALSE_MESSAGE(l.inAircraft(),
                            "a climb under canopy re-armed the aircraft latch");
}

// A hop-and-pop may never reach freefall speed, so a latch that only releases
// on FREEFALL stays armed the whole way down and hides the ladder. Keying off
// the altitude trend covers it; keying off having seen FREEFALL does not.
static void test_aircraft_latch_releases_without_freefall()
{
  AircraftLatch l;
  l.begin(300.0f, 100.0f, 150.0f);

  for (float a = 0; a < 1200; a += 25) l.update(a, MODE_CLIMB);
  TEST_ASSERT_TRUE(l.inAircraft());

  // Straight out under canopy: never FREEFALL, just down.
  bool released = false;
  for (float a = 1200; a > 900; a -= 5)
  {
    l.update(a, MODE_CANOPY);
    if (!l.inAircraft()) { released = true; break; }
  }
  TEST_ASSERT_TRUE_MESSAGE(released,
      "latch held through a hop-and-pop that never reached freefall");
}

static void test_autozero_timeout_scales_with_altitude()
{
  GroundRef g;
  g.begin(kGref);
  // 3 + 0.05*alt minutes, per Alti-2's published behaviour.
  TEST_ASSERT_UINT32_WITHIN(2000,  (uint32_t)(3.0 * 60000), g.requiredSettleMs(0.0f));
  TEST_ASSERT_UINT32_WITHIN(2000,  (uint32_t)(4.25 * 60000), g.requiredSettleMs(25.0f));
  TEST_ASSERT_UINT32_WITHIN(5000,  (uint32_t)(10.6 * 60000), g.requiredSettleMs(152.0f));
  // 4000 m must be hours, not minutes — this is the aircraft-hold guard.
  TEST_ASSERT_TRUE(g.requiredSettleMs(4000.0f) > (uint32_t)(3.0 * 3600000));
}

// Sitting on the ground with a couple of metres of drift: correct it, slowly.
static void test_autozero_corrects_ground_drift()
{
  GroundRef g;
  g.begin(kGref);
  uint32_t clock = 0;
  g.reset(clock);

  // Nothing for the first few minutes.
  float applied = runGref(g, 2.0f, 0.0f, 150.0f, &clock, false);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, applied);
  TEST_ASSERT_FALSE(g.correcting());

  // Past the ~3 min threshold it starts. Feed the correction back into the
  // altitude, as the real device does, so it can actually converge.
  float alt = 2.0f;
  for (int i = 0; i < 2400; i++)          // 20 minutes at 0.5 s steps
  {
    clock += 500;
    const float c = g.update(alt, 0.0f, clock);
    alt -= c;
    applied += c;
  }
  TEST_ASSERT_TRUE_MESSAGE(g.correcting(), "should be correcting by now");
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, 0.0f, alt,
                                   "drift should have been pulled out to zero");
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, 2.0f, applied,
                                   "total correction should equal the drift");
}

// THE hazard: an aircraft holding at altitude must never be re-zeroed.
static void test_autozero_never_fires_at_altitude()
{
  GroundRef g;
  g.begin(kGref);
  uint32_t clock = 0;
  g.reset(clock);

  // Climb to 4000 m, then hold dead still for a full hour.
  runGref(g, 2000.0f, 5.0f, 400.0f, &clock, false);        // climbing
  const float applied = runGref(g, 4000.0f, 0.0f, 3600.0f, &clock, false);

  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, applied,
      "re-zeroed while holding at altitude — this is the dangerous case");
  TEST_ASSERT_TRUE_MESSAGE(g.inFlight(), "in-flight latch should still be set");
}

// The latch alone must block correction even if the timeout somehow elapsed.
static void test_inflight_latch_blocks_and_clears()
{
  GroundRef g;
  g.begin(kGref);
  uint32_t clock = 0;
  g.reset(clock);

  runGref(g, 100.0f, 5.0f, 60.0f, &clock, false);       // moving -> latched
  TEST_ASSERT_TRUE(g.inFlight());

  // Back on the ground but not yet for long enough.
  runGref(g, 1.0f, 0.0f, 60.0f, &clock, false);
  TEST_ASSERT_TRUE_MESSAGE(g.inFlight(), "latch cleared too eagerly");

  // After the full clear time it releases.
  runGref(g, 1.0f, 0.0f, 120.0f, &clock, false);
  TEST_ASSERT_FALSE_MESSAGE(g.inFlight(), "latch never cleared");
}

// A step offset must converge smoothly, not in chunks. Applying a correction
// moves the reported altitude; if the tracker mistakes that for the device being
// picked up it restarts its own settle timer, and a 2 m offset then corrects one
// band at a time with a full settle wait between each — ten minutes with a
// four-minute stall in the middle, which reads as a hang.
static void test_step_offset_converges_without_stalling()
{
  GroundRef g;
  g.begin(kGref);
  uint32_t clock = 0;
  g.reset(clock);

  float alt = -2.0f;                       // zeroed 2 m up, then set down
  uint32_t startedMs = 0, doneMs = 0;
  uint32_t longestPause = 0, lastCorrMs = 0;

  for (int i = 0; i < 4800; i++)           // 40 minutes at 0.5 s steps
  {
    clock += 500;
    const float c = g.update(alt, 0.0f, clock);
    alt -= c;

    if (c != 0.0f)
    {
      if (!startedMs) { startedMs = clock; lastCorrMs = clock; }
      const uint32_t gap = clock - lastCorrMs;
      if (startedMs && gap > longestPause) longestPause = gap;
      lastCorrMs = clock;
    }
    if (startedMs && !doneMs && std::fabs(alt) < 0.05f) { doneMs = clock; break; }
  }

  TEST_ASSERT_TRUE_MESSAGE(startedMs > 0, "never started correcting");
  TEST_ASSERT_TRUE_MESSAGE(doneMs > 0, "never converged");

  // 2 m at 0.5 m/min is 4 minutes of slewing. Allow a little slack, but nothing
  // like the extra settle wait a restart would cost.
  const uint32_t slewMs = doneMs - startedMs;
  TEST_ASSERT_TRUE_MESSAGE(slewMs < 5u * 60000u,
                           "convergence took far longer than the slew rate implies");
  // No pause anywhere near a settle period.
  TEST_ASSERT_TRUE_MESSAGE(longestPause < 30000u,
                           "stalled mid-correction — the settle timer restarted");
}

// The sandwich case: a few metres up for a few minutes, then back down. The
// point of slewing is that this costs a small error, not a wrong zero.
static void test_short_excursion_costs_little()
{
  GroundRef g;
  g.begin(kGref);
  uint32_t clock = 0;
  g.reset(clock);

  runGref(g, 0.0f, 0.0f, 900.0f, &clock, false);        // settled at home
  // Walk up 3 m and linger 5 minutes. The 1 m settle band restarts the timer,
  // so ~3.15 min passes before any correction, and the slow slew then moves the
  // reference under a metre before you come back down.
  const float up = runGref(g, 3.0f, 0.0f, 300.0f, &clock, false);
  TEST_ASSERT_TRUE_MESSAGE(up < 1.5f,
      "a five-minute excursion should not meaningfully move the reference");
}

// ---------------------------------------------------------------------------
int main(int, char **)
{
  UNITY_BEGIN();

  RUN_TEST(test_isa_reference_points);
  RUN_TEST(test_sea_level_pressure_roundtrip);
  RUN_TEST(test_agl_is_qnh_insensitive);

  RUN_TEST(test_filter_suppresses_bench_noise);
  RUN_TEST(test_filter_recovers_quickly_after_lift_and_hold);
  RUN_TEST(test_filter_tracks_freefall_without_lag);
  RUN_TEST(test_filter_tracks_canopy_opening);
  RUN_TEST(test_filter_rejects_bad_input);

  RUN_TEST(test_plain_zone_mapping);
  RUN_TEST(test_bench_thresholds_match_spec);
  RUN_TEST(test_boundary_noise_settles_low_without_chatter);
  RUN_TEST(test_dwell_rejects_single_sample_spike);
  RUN_TEST(test_sustained_change_does_commit);
  RUN_TEST(test_multi_zone_jump_in_one_step);
  RUN_TEST(test_descent_visits_every_zone_in_order);
  RUN_TEST(test_threshold_lag_at_freefall_speed);
  RUN_TEST(test_no_led_chatter_when_parked);
  RUN_TEST(test_sharp_move_changes_zone_promptly);
  RUN_TEST(test_every_bench_zone_is_reachable_from_both_directions);

  RUN_TEST(test_landing_ladder_bands);
  RUN_TEST(test_flight_mode_transitions_with_hysteresis);
  RUN_TEST(test_high_speed_malfunction_keeps_freefall_warnings);
  RUN_TEST(test_climb_marker_fires_every_100m);
  RUN_TEST(test_full_jump_profile);

  RUN_TEST(test_velocity_window_survives_freefall_noise);
  RUN_TEST(test_velocity_window_quiet_and_ready);
  RUN_TEST(test_aircraft_latch_ignores_a_climb_during_descent);
  RUN_TEST(test_aircraft_latch_releases_without_freefall);
  RUN_TEST(test_periodic_wake_transient_does_not_starve_autozero);
  RUN_TEST(test_autozero_timeout_scales_with_altitude);
  RUN_TEST(test_autozero_corrects_ground_drift);
  RUN_TEST(test_autozero_never_fires_at_altitude);
  RUN_TEST(test_inflight_latch_blocks_and_clears);
  RUN_TEST(test_step_offset_converges_without_stalling);
  RUN_TEST(test_short_excursion_costs_little);

  return UNITY_END();
}

void setUp() {}
void tearDown() {}
