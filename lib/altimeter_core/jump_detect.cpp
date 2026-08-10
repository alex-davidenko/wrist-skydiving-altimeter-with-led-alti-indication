#include "jump_detect.h"

#include <cmath>

#include "flight_mode.h"

void JumpDetector::begin(const Config &c) { _cfg = c; reset(); }

void JumpDetector::reset()
{
  _rec = false; _finished = false; _aborted = false;
  _sawDescent = false; _sawFreefall = false;
  _peak = 0.0f; _lowSince = 0; _low = false; _startedMs = 0;
}

bool JumpDetector::update(float altM, float vspeedMps, uint8_t mode, uint32_t nowMs)
{
  _finished = false;
  _aborted = false;
  const float descent = -vspeedMps;

  if (!_rec)
  {
    const bool climbing = (mode == MODE_CLIMB) && altM > _cfg.startAltM;
    const bool falling  = descent >= _cfg.freefallMps;
    if (climbing || falling)
    {
      _rec = true;
      _startedMs = nowMs;
      _sawFreefall = falling;
      _sawDescent = falling;
      _peak = altM;
      _low = false;
    }
    return _rec;
  }

  if (descent >= _cfg.freefallMps) _sawFreefall = true;
  if (altM > _peak) _peak = altM;

  // No freefall in this long means it was never a jump — a drive home, or a
  // climb that got abandoned. Give the file back rather than keep growing it.
  if (!_sawFreefall && (uint32_t)(nowMs - _startedMs) >= _cfg.maxMs)
  {
    _rec = false; _aborted = true; _low = false;
    return false;
  }
  // Only a real descent from height counts. Without this, arming on the ground
  // and never climbing would satisfy the stop test immediately and produce a
  // stream of empty files.
  if (_peak > _cfg.startAltM && altM < _peak - _cfg.startAltM) _sawDescent = true;

  const bool lowAndStill = altM < _cfg.stopAltM && std::fabs(vspeedMps) < _cfg.stillMps;
  if (lowAndStill)
  {
    if (!_low) { _low = true; _lowSince = nowMs; }
    else if (_sawDescent && (uint32_t)(nowMs - _lowSince) >= _cfg.settleMs)
    {
      _rec = false;
      _low = false;
      // A recording that never saw freefall is not a jump, whatever else it
      // did. Landing back where you started after driving up a hill qualifies.
      if (_sawFreefall) _finished = true; else _aborted = true;
    }
  }
  else
  {
    _low = false;
  }
  return _rec;
}
