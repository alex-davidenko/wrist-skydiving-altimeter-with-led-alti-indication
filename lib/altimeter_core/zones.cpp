#include "zones.h"

const char *zoneName(uint8_t z)
{
  switch (z)
  {
    case ZONE_OFF:       return "OFF";
    case ZONE_BLINK_RED: return "BLINK_RED";
    case ZONE_RED:       return "RED";
    case ZONE_YELLOW:    return "YELLOW";
    case ZONE_GREEN:     return "GREEN";
    case ZONE_ABOVE:     return "ABOVE";
    default:             return "?";
  }
}

const char *landingName(uint8_t z)
{
  switch (z)
  {
    case LAND_DARK_LOW:  return "DARK_LOW";
    case LAND_STEADY:    return "STEADY";
    case LAND_FAST:      return "FAST_6HZ";
    case LAND_SLOW:      return "SLOW_3HZ";
    case LAND_DARK_HIGH: return "DARK_HIGH";
    default:             return "?";
  }
}

uint8_t ZoneTracker::zoneFor(float altitude) const
{
  for (uint8_t i = 0; i < kMaxZoneBounds; i++)
  {
    if (altitude < _cfg.bounds[i]) return i;
  }
  return kMaxZoneBounds;
}

void ZoneTracker::reset(float altitude)
{
  _zone         = zoneFor(altitude);
  _pending      = _zone;
  _pendingSince = 0;
  _has          = true;
}

uint8_t ZoneTracker::update(float altitude, uint32_t nowMs)
{
  if (!_has)
  {
    reset(altitude);
    return _zone;
  }

  // Leave the current band only once the altitude has cleared the boundary by
  // the full hysteresis for that direction. Loops rather than single-stepping
  // so a large jump can cross several bands at once.
  int z = _zone;
  while (z < kMaxZoneBounds && altitude >= _cfg.bounds[z] + _cfg.hystRelax)  z++;
  while (z > 0              && altitude <  _cfg.bounds[z - 1] - _cfg.hystUrgent) z--;

  const uint8_t candidate = static_cast<uint8_t>(z);

  if (candidate == _zone)
  {
    // Back inside the committed band — cancel any pending change.
    _pending = _zone;
    return _zone;
  }

  // The dwell clock measures time since we *left the committed band*, not time
  // the candidate has held one particular value.
  //
  // The difference matters on a fast move. The filter briefly overshoots past
  // its target, so the candidate can read GREEN -> RED -> YELLOW while it
  // settles. Restarting the clock on every candidate change made settling time
  // and dwell compound: a sharp 2.2 m -> 1.35 m bench move took ~1.0-1.45 s to
  // show yellow instead of the ~0.5 s the dwell alone should cost.
  //
  // Spike rejection is unaffected: a transient that returns to the committed
  // band resets _pending above and cancels the clock. What changed is only that
  // an excursion which settles somewhere *else* no longer pays twice.
  if (_pending == _zone) _pendingSince = nowMs;   // first sample outside
  _pending = candidate;

  const uint32_t dwell = (candidate < _zone) ? _cfg.dwellUrgentMs : _cfg.dwellRelaxMs;
  if (static_cast<uint32_t>(nowMs - _pendingSince) >= dwell)
  {
    _zone = candidate;
  }
  return _zone;
}
