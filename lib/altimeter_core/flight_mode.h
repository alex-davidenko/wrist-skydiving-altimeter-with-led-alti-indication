#pragma once
//
// Which phase of the jump are we in? Decided from vertical speed alone.
//
//   CLIMB    — going up. In the aircraft.
//   FREEFALL — descending faster than the canopy threshold. Freefall, or a
//              high-speed malfunction (which is exactly when the freefall
//              colour zones are still the correct thing to show).
//   CANOPY   — descending slower than that, or stationary. Under a good
//              canopy, sitting on the ground, or in level flight.
//
// The freefall colour zones are only meaningful in FREEFALL. Under canopy the
// altitude bands that matter are the landing ladder instead, and above the
// landing ladder the LED should be dark rather than shouting "RED" at someone
// flying a perfectly good canopy at 900 m.
//
// Pure C++ (time is passed in), so it unit-tests on the host.
//

#include <cstdint>

enum FlightMode : uint8_t
{
  MODE_CANOPY = 0,
  MODE_FREEFALL,
  MODE_CLIMB,
  MODE_COUNT
};

const char *flightModeName(uint8_t m);

struct FlightModeConfig
{
  // Descent rate (positive = falling) to enter / stay in FREEFALL.
  // Separate enter/exit values give hysteresis, so a descent hovering near the
  // threshold does not flap the whole display between two different meanings.
  float    freefallEnterMps;
  float    freefallExitMps;

  // Climb rate (positive = rising) to enter / stay in CLIMB.
  float    climbEnterMps;
  float    climbExitMps;

  // A mode change must hold this long before it commits. Mode changes swap the
  // entire meaning of the LED, so this is deliberately slower than the
  // within-mode dwell.
  uint32_t dwellMs;
};

class FlightModeTracker
{
public:
  void begin(const FlightModeConfig &cfg) { _cfg = cfg; _has = false; }
  void reset(FlightMode m);

  // `verticalSpeedMps` is positive upward, straight from AltitudeFilter.
  FlightMode update(float verticalSpeedMps, uint32_t nowMs);

  FlightMode mode() const { return _mode; }

private:
  FlightModeConfig _cfg{};
  FlightMode _mode         = MODE_CANOPY;
  FlightMode _pending      = MODE_CANOPY;
  uint32_t   _pendingSince = 0;
  bool       _has          = false;
};


// Fires once every `intervalM` of altitude gained while climbing, so the
// aircraft ride gets a progress marker without any altitude readout.
class ClimbMarker
{
public:
  void  configure(float intervalM) { _interval = (intervalM > 1.0f) ? intervalM : 100.0f; }

  // Returns true on the single sample where a new interval is crossed upward.
  // While not climbing it just tracks, so a climb never starts with a stray
  // flash and a descent cannot re-arm one.
  bool  update(float altitude, bool climbing);

private:
  float   _interval = 100.0f;
  int32_t _lastBand = 0;
};
