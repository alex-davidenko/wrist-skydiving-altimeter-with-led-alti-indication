#pragma once
//
// Automatic ground-reference correction, i.e. tracking weather drift.
//
// WHY THIS EXISTS. Uncorrected QNH drift over a jumping day is tens of metres,
// so leaving the reference fixed is itself the error. Production kit all does
// this: CYPRES re-measures ground pressure twice a minute while on the ground,
// and the Vigil Cuatro recalibrates every two minutes "progressively".
//
// WHY IT IS SAFE. The obvious hazard is re-zeroing while parked at altitude —
// an aircraft holding at 4000 m for clearance would otherwise have its AGL
// zeroed there. Alti-2 solve this by scaling the settle time with altitude:
// roughly ten minutes at 500 ft after a slow climb, but "several hours" at
// 12,000 ft. Fitting those two published points gives
//
//     settle_minutes = 3 + 0.05 * altitude_m
//
// which is 4 minutes at 25 m and 3.3 hours at 4000 m. A short excursion — a
// walk upstairs, a trip to get a sandwich — corrects in minutes because it is
// only a few metres up. The aircraft case cannot fire.
//
// A second, independent guard: once a real climb or descent is seen, an
// in-flight latch disables correction entirely until the device is back low and
// stationary. That makes the safe behaviour structural rather than dependent on
// the constants above being right.
//
// Corrections SLEW rather than step, as the Vigil does, so a brief excursion
// partially corrects and then corrects back instead of latching onto the wrong
// floor.
//
// Pure C++ (time passed in), so it unit-tests on the host.
//

#include <cstdint>

struct GroundRefConfig
{
  float    settledVspeedMps;   // below this counts as stationary
  float    settleBandM;        // altitude must hold within this to keep counting
  float    baseMinutes;        // constant term of the settle time
  float    perMetreMinutes;    // altitude-scaled term
  float    slewMPerMin;        // how fast the reference is allowed to move
  float    latchSetVspeedMps;  // sustained motion above this means "in flight"
  float    latchClearAltM;     // must be below this to clear the latch
  uint32_t latchClearMs;       // and stationary for this long
};

class GroundRef
{
public:
  void begin(const GroundRefConfig &cfg) { _cfg = cfg; reset(0); }

  // Forget all progress. Call after a manual zero.
  void reset(uint32_t nowMs);

  // Feed the filtered altitude and vertical speed. Returns how many metres to
  // ADD to the stored ground reference this step — positive when the reported
  // altitude is drifting high and should be pulled back down. Zero most of the
  // time.
  float update(float aglM, float vspeedMps, uint32_t nowMs);

  bool     inFlight()   const { return _inFlight; }
  bool     correcting() const { return _correcting; }
  uint32_t settledMs(uint32_t nowMs) const { return nowMs - _settleSince; }

  // How long this altitude must hold still before correction may begin.
  uint32_t requiredSettleMs(float aglM) const;

private:
  GroundRefConfig _cfg{};
  bool     _inFlight    = false;
  bool     _correcting  = false;
  float    _settleAlt   = 0.0f;
  uint32_t _settleSince = 0;
  uint32_t _latchSince  = 0;
  uint32_t _lastSlewMs  = 0;
};
