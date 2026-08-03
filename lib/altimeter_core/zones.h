#pragma once
//
// Altitude -> band mapping with asymmetric hysteresis and dwell.
//
// Used twice: once for the freefall colour zones, once for the landing ladder
// under canopy. Both are "which altitude band am I in", so they share the
// machinery and differ only in configuration.
//
// ASYMMETRY IS THE POINT. Moving to a LOWER band means danger increasing, and
// must commit fast. Moving to a HIGHER band means the situation improved, and
// can stay heavily damped. A safety indicator should fail toward urgency.
// Measured on a 50 m/s descent through the 800 m threshold: symmetric 5 m
// hysteresis + 120 ms dwell lights the LED 11 m late; urgent-asymmetric lights
// it 2-3 m late.
//
// Two damping mechanisms, because they stop different things:
//
//   hysteresis — the altitude must overshoot a boundary before the band
//                changes. Kills flicker from noise on a drifting altitude.
//
//   dwell      — the new band must hold before it is committed. Kills flicker
//                from single-sample outliers big enough to clear hysteresis.
//
// Pure C++ (time is passed in), so it unit-tests on the host.
//

#include <cstdint>

// Bands are numbered from the ground up: 0 is the lowest, kMaxZoneBounds the
// highest. A boundary set to a huge value makes the band above it unreachable.
constexpr uint8_t kMaxZoneBounds = 5;
constexpr float   kZoneBoundDisabledHigh =  1.0e9f;
constexpr float   kZoneBoundDisabledLow  = -1.0e9f;

struct ZoneConfig
{
  float    bounds[kMaxZoneBounds];  // ascending; bounds[i] splits band i from i+1
  float    hystUrgent;              // overshoot needed to DROP a band (m)
  float    hystRelax;               // overshoot needed to CLIMB a band (m)
  uint32_t dwellUrgentMs;           // hold time before committing a drop
  uint32_t dwellRelaxMs;            // hold time before committing a climb
};

// ---------------------------------------------------------------------------
//  Freefall colour zones (bands 0..5, so 5 boundaries)
// ---------------------------------------------------------------------------
enum Zone : uint8_t
{
  ZONE_OFF = 0,     // below the lowest boundary — LED dark
  ZONE_BLINK_RED,
  ZONE_RED,
  ZONE_YELLOW,
  ZONE_GREEN,
  ZONE_ABOVE,       // above the top of the green band
  ZONE_COUNT
};
const char *zoneName(uint8_t z);

// ---------------------------------------------------------------------------
//  Landing ladder under canopy (bands 0..4; top boundary disabled)
// ---------------------------------------------------------------------------
enum LandingZone : uint8_t
{
  LAND_DARK_LOW = 0,  // below ~10 m — assistance finished, LED off
  LAND_STEADY,        // ~10-100 m  — bright steady green
  LAND_FAST,          // ~100-200 m — 6 blinks/s
  LAND_SLOW,          // ~200-300 m — 3 blinks/s
  LAND_DARK_HIGH,     // above ~300 m — nothing to do yet, LED off
  LAND_COUNT
};
const char *landingName(uint8_t z);

class ZoneTracker
{
public:
  void begin(const ZoneConfig &cfg) { _cfg = cfg; _has = false; }

  // Snap straight to the band containing `altitude`, ignoring hysteresis and
  // dwell. Use at startup and after re-zeroing.
  void reset(float altitude);

  // Feed a filtered altitude. Returns the (possibly unchanged) committed band.
  uint8_t update(float altitude, uint32_t nowMs);

  uint8_t zone() const { return _zone; }

  // Band containing `altitude`, ignoring all hysteresis/dwell state.
  uint8_t zoneFor(float altitude) const;

private:
  ZoneConfig _cfg{};
  uint8_t    _zone         = 0;
  uint8_t    _pending      = 0;
  uint32_t   _pendingSince = 0;
  bool       _has          = false;
};
