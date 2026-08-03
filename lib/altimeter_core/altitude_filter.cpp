#include "altitude_filter.h"

#include <cmath>

AltitudeFilter::AltitudeFilter(float sigmaAccel, float sigmaMeas)
{
  configure(sigmaAccel, sigmaMeas);
}

void AltitudeFilter::configure(float sigmaAccel, float sigmaMeas)
{
  _sigmaAccel = (sigmaAccel > 0.0f) ? sigmaAccel : 1e-3f;
  _sigmaMeas  = (sigmaMeas  > 0.0f) ? sigmaMeas  : 1e-3f;
}

void AltitudeFilter::configureAdaptive(float gate, float maxInflate, float biasAlpha)
{
  _gate       = gate;
  _maxInflate = (maxInflate > 1.0f) ? maxInflate : 1.0f;
  _biasAlpha  = (biasAlpha > 0.0f && biasAlpha < 1.0f) ? biasAlpha : 0.05f;
  _biasSigmaScale = sqrtf(_biasAlpha / (2.0f - _biasAlpha));
}

void AltitudeFilter::reset(float altitude)
{
  _h = altitude;
  _v = 0.0f;
  // Start with a modest, non-zero covariance so the first few samples still
  // pull the state — a hard zero would make the filter ignore them.
  _p00 = _sigmaMeas * _sigmaMeas;
  _p01 = 0.0f;
  _p11 = 1.0f;
  _innovBias = 0.0f;
  _initialised = true;
}

void AltitudeFilter::update(float z, float dt)
{
  if (!std::isfinite(z)) return;

  if (!_initialised)
  {
    reset(z);
    return;
  }

  // ---- Predict -----------------------------------------------------------
  if (std::isfinite(dt) && dt > 0.0f)
  {
    _h += _v * dt;

    // Discrete white-noise acceleration model:
    //   Q = sigmaAccel^2 * [ dt^4/4  dt^3/2 ; dt^3/2  dt^2 ]
    const float qa   = _sigmaAccel * _sigmaAccel;
    const float dt2  = dt * dt;
    const float q00  = qa * dt2 * dt2 * 0.25f;
    const float q01  = qa * dt2 * dt  * 0.5f;
    const float q11  = qa * dt2;

    // P = F P F^T + Q,  with F = [ 1 dt ; 0 1 ]
    _p00 += dt * (2.0f * _p01 + dt * _p11) + q00;
    _p01 += dt * _p11 + q01;
    _p11 += q11;
  }

  // ---- Update (measure altitude directly, H = [1 0]) ---------------------
  const float r = _sigmaMeas * _sigmaMeas;
  float s = _p00 + r;
  if (s <= 0.0f) return;

  float y = z - _h;

  // Adaptive process noise.
  //
  // A constant-velocity model coasts: when the real motion stops, the velocity
  // state is still wound up, so the estimate overshoots and then takes seconds
  // to bleed off. Lowering sigmaAccel does not help — it just trades overshoot
  // amplitude for settling time.
  //
  // Note this cannot be detected from a single sample: the innovation
  // covariance S = P00 + R is dominated by the sensor noise R, so a coast
  // error of a few tenths of a metre sits well inside 1 sigma and any
  // instantaneous gate stays quiet. What gives it away is that the innovations
  // stop looking like zero-mean noise and acquire a persistent sign.
  //
  // So: track a slow EMA of the innovation. White noise averages to ~zero with
  // a known standard error; a real model bias does not. When the EMA stands
  // out from that noise floor, inflate P, which raises the Kalman gain and lets
  // the filter snap onto the measurements instead of coasting. It drops back to
  // heavy smoothing as soon as the innovations re-centre.
  if (_gate > 0.0f)
  {
    _innovBias += _biasAlpha * (y - _innovBias);

    // Standard error of an EMA of white noise with weight a is
    // sigma * sqrt(a / (2 - a)).
    const float biasSigma = _sigmaMeas * _biasSigmaScale;
    const float excess    = fabsf(_innovBias) / (_gate * biasSigma);

    if (excess > 1.0f)
    {
      float lambda = excess * excess;
      if (lambda > _maxInflate) lambda = _maxInflate;
      _p00 *= lambda;
      _p01 *= lambda;
      _p11 *= lambda;
      s = _p00 + r;
    }
  }

  const float k0 = _p00 / s;
  const float k1 = _p01 / s;

  _h += k0 * y;
  _v += k1 * y;

  // P = (I - K H) P — capture the old top row before overwriting it.
  const float p00 = _p00;
  const float p01 = _p01;
  _p00 -= k0 * p00;
  _p01 -= k0 * p01;
  _p11 -= k1 * p01;
}

float AltitudeFilter::sigma() const
{
  return (_p00 > 0.0f) ? sqrtf(_p00) : 0.0f;
}
