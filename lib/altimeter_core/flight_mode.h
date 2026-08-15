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

  // Leaving FREEFALL is slower than everything else, and the asymmetry is the
  // point. Measured on jump 185: twice mid-freefall the barometric altitude
  // stopped descending for about a second — once climbing 87 m before resuming —
  // while body position changed the flow over the port. The samples were clean
  // and evenly spaced; the pressure really did that. The windowed velocity fell
  // under the exit threshold, held past a 400 ms dwell, and the phase machine
  // correctly concluded the fall had stopped. The strip went dark twice during
  // the part of the jump it exists for.
  //
  // Nothing is lost by being slow here. A canopy takes seconds to open, so
  // showing freefall colours a moment longer costs nothing, while dropping out
  // for a second in the middle of freefall costs exactly the thing the device
  // is for. Entering FREEFALL stays fast.
  uint32_t exitFreefallDwellMs;
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

// Are we still riding up in the aircraft?
//
// This exists because it decides the colour before the zone is ever consulted:
// led::flightPattern() paints blue for "in aircraft" regardless of altitude, so
// a latch stuck on hides the entire ladder. On jump 1 that made 58% of the
// descent blue while the zone ladder underneath was perfectly correct.
//
// It cannot key off vertical speed alone. A 1.5 s window still holds climb data
// for a moment after exit, and under canopy a riser input or a thermal is a
// genuine brief climb — either re-armed the old latch, which then held for
// hundreds of metres because it only released on FREEFALL or below 100 m.
//
// So the rule is altitude trend, Alex's suggestion: you cannot be riding up
// while you are `descendConfirmM` below where you just were. That both blocks
// re-arming and clears an armed latch, and unlike keying off FREEFALL it also
// covers a hop-and-pop that never reaches freefall speed.
class AircraftLatch
{
 public:
  void begin(float latchAltM, float clearAltM, float descendConfirmM);
  void reset();

  // Feed the current altitude and phase each sample. Returns the latch state.
  bool update(float altitudeM, uint8_t mode);

  bool inAircraft() const { return _in; }
  float peak() const { return _peak; }

 private:
  float _latchAlt = 300.0f, _clearAlt = 100.0f, _confirm = 150.0f;
  float _peak = -1e9f;
  bool  _in = false;
};
