#include "ground_ref.h"

#include <cmath>

void GroundRef::reset(uint32_t nowMs)
{
  _inFlight    = false;
  _correcting  = false;
  _settleAlt   = 0.0f;
  _settleSince = nowMs;
  _latchSince  = nowMs;
  _lastSlewMs  = nowMs;
}

uint32_t GroundRef::requiredSettleMs(float aglM) const
{
  const float minutes = _cfg.baseMinutes + _cfg.perMetreMinutes * std::fabs(aglM);
  return static_cast<uint32_t>(minutes * 60000.0f);
}

float GroundRef::update(float aglM, float vspeedMps, uint32_t nowMs)
{
  const float speed = std::fabs(vspeedMps);

  // ---- in-flight latch ---------------------------------------------------
  // Any sustained motion means we are no longer sitting on the ground, and the
  // reference must not move until we are demonstrably back down and still.
  if (speed >= _cfg.latchSetVspeedMps)
  {
    _inFlight   = true;
    _latchSince = nowMs;
  }
  else if (_inFlight)
  {
    const bool low  = std::fabs(aglM) < _cfg.latchClearAltM;
    const bool calm = speed < _cfg.settledVspeedMps;
    if (low && calm)
    {
      if (static_cast<uint32_t>(nowMs - _latchSince) >= _cfg.latchClearMs)
        _inFlight = false;
    }
    else
    {
      _latchSince = nowMs;
    }
  }

  // ---- settle tracking ---------------------------------------------------
  // Moving, or having moved away from where the timer started, restarts it.
  // Restarting on altitude change is what makes an aircraft hold safe even
  // before the scaled timeout is considered: the altitude never sits still
  // enough for long enough.
  if (speed >= _cfg.settledVspeedMps ||
      std::fabs(aglM - _settleAlt) > _cfg.settleBandM)
  {
    _settleAlt   = aglM;
    _settleSince = nowMs;
    _lastSlewMs  = nowMs;
    _correcting  = false;
    return 0.0f;
  }

  if (_inFlight) { _lastSlewMs = nowMs; return 0.0f; }

  if (static_cast<uint32_t>(nowMs - _settleSince) < requiredSettleMs(aglM))
  {
    _lastSlewMs = nowMs;
    return 0.0f;
  }

  // ---- slew, do not step -------------------------------------------------
  _correcting = true;
  const float dtMin = static_cast<float>(nowMs - _lastSlewMs) / 60000.0f;
  _lastSlewMs = nowMs;

  float step = _cfg.slewMPerMin * dtMin;
  if (step > std::fabs(aglM)) step = std::fabs(aglM);   // never overshoot zero
  const float correction = (aglM >= 0.0f) ? step : -step;

  // Follow our own output. Applying a correction moves the reported altitude,
  // and without this the settle band sees that movement, concludes the device
  // was picked up, and restarts the timer — so a large offset was corrected in
  // one-band chunks separated by full settle waits. A 2 m step took ten minutes
  // with a four-minute stall in the middle, which looks indistinguishable from
  // a hang. Real drift never triggered it (0.03 m/min against a 1 m band), but
  // any step change did.
  _settleAlt -= correction;

  return correction;
}
