#include "flight_mode.h"

#include <cmath>

const char *flightModeName(uint8_t m)
{
  switch (m)
  {
    case MODE_CANOPY:   return "CANOPY";
    case MODE_FREEFALL: return "FREEFALL";
    case MODE_CLIMB:    return "CLIMB";
    default:            return "?";
  }
}

void FlightModeTracker::reset(FlightMode m)
{
  _mode         = m;
  _pending      = m;
  _pendingSince = 0;
  _has          = true;
}

FlightMode FlightModeTracker::update(float v, uint32_t nowMs)
{
  if (!std::isfinite(v)) return _mode;

  const float descent = -v;   // positive when falling

  // Each test uses the exit threshold when already in that mode, and the
  // (stricter) enter threshold otherwise. FREEFALL wins over CLIMB — you
  // cannot be doing both, and mistaking freefall for a climb is the worse
  // error by a wide margin.
  FlightMode candidate;
  if (_mode == MODE_FREEFALL ? (descent > _cfg.freefallExitMps)
                             : (descent >= _cfg.freefallEnterMps))
  {
    candidate = MODE_FREEFALL;
  }
  else if (_mode == MODE_CLIMB ? (v > _cfg.climbExitMps)
                               : (v >= _cfg.climbEnterMps))
  {
    candidate = MODE_CLIMB;
  }
  else
  {
    candidate = MODE_CANOPY;
  }

  if (!_has)
  {
    reset(candidate);
    return _mode;
  }

  if (candidate == _mode)
  {
    _pending = _mode;
    return _mode;
  }

  // Same "time since leaving the committed value" semantics as ZoneTracker —
  // see zones.cpp for why the clock must not restart on every candidate change.
  if (_pending == _mode) _pendingSince = nowMs;
  _pending = candidate;

  const uint32_t need = (_mode == MODE_FREEFALL && _cfg.exitFreefallDwellMs)
                          ? _cfg.exitFreefallDwellMs
                          : _cfg.dwellMs;
  if (static_cast<uint32_t>(nowMs - _pendingSince) >= need)
  {
    _mode = candidate;
  }
  return _mode;
}


bool ClimbMarker::update(float altitude, bool climbing)
{
  const int32_t band = static_cast<int32_t>(std::floor(altitude / _interval));

  if (!climbing)
  {
    _lastBand = band;
    return false;
  }
  if (band > _lastBand)
  {
    _lastBand = band;
    return true;
  }
  return false;
}

void AircraftLatch::begin(float latchAltM, float clearAltM, float descendConfirmM)
{
  _latchAlt = latchAltM;
  _clearAlt = clearAltM;
  _confirm  = descendConfirmM;
  reset();
}

void AircraftLatch::reset()
{
  _peak = -1e9f;
  _in   = false;
}

bool AircraftLatch::update(float altitudeM, uint8_t mode)
{
  if (altitudeM > _peak) _peak = altitudeM;
  // Back on the ground: forget the peak, so the next climb starts clean.
  if (altitudeM < _clearAlt) _peak = altitudeM;

  const bool descending = altitudeM < _peak - _confirm;

  if (mode == MODE_CLIMB && altitudeM > _latchAlt && !descending) _in = true;
  if (mode == MODE_FREEFALL || altitudeM < _clearAlt || descending) _in = false;
  return _in;
}
