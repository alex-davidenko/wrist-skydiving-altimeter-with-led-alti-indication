#pragma once
//
// Two-state (altitude, vertical speed) Kalman filter with a constant-velocity
// motion model.
//
// Why not a moving average: on a steady descent a boxcar of N samples lags by
// (N-1)/2 samples. At 40 Hz and 50 m/s freefall, a 25-tap average is ~15 m
// behind the truth — that is a real error at the 800 m threshold. A CV Kalman
// has *zero* steady-state lag on a constant-velocity ramp, because the velocity
// state absorbs it. It only lags during acceleration (exit, canopy opening),
// which is exactly where a little smoothing is wanted anyway.
//
// Falling back to a plain moving average is still possible for comparison —
// see MovingAverage at the bottom of this header.
//

#include <cstddef>
#include <cstdint>

class AltitudeFilter
{
public:
  // sigmaAccel : how much vertical acceleration (m/s^2, 1-sigma) the model
  //              should expect between samples. Larger = trusts the sensor
  //              more, tracks faster, smooths less.
  // sigmaMeas  : sensor noise (m, 1-sigma) on a single altitude sample.
  AltitudeFilter(float sigmaAccel = 3.0f, float sigmaMeas = 0.6f);

  void  configure(float sigmaAccel, float sigmaMeas);

  // Innovation gate for the adaptive process noise (see the .cpp for why this
  // exists). `gate` is in sigmas: measurements landing further out than this
  // inflate the covariance so the filter re-locks instead of coasting.
  // Set gate <= 0 to disable and get a plain constant-velocity Kalman.
  void  configureAdaptive(float gate, float maxInflate, float biasAlpha = 0.05f);

  // Jump the state straight to `altitude` and clear the covariance.
  // Call on startup and after re-zeroing.
  void  reset(float altitude = 0.0f);

  // Feed one measurement. `dt` in seconds; non-finite or non-positive dt is
  // ignored (the sample is still applied as a pure update).
  void  update(float measuredAltitude, float dt);

  float altitude()  const { return _h; }
  float velocity()  const { return _v; }   // m/s, positive = climbing
  bool  ready()     const { return _initialised; }

  // 1-sigma uncertainty in the altitude estimate (m). Handy for deciding
  // whether a ground-zero average has settled.
  float sigma() const;

private:
  float _sigmaAccel;
  float _sigmaMeas;
  float _gate       = 2.0f;
  float _maxInflate = 500.0f;
  float _biasAlpha  = 0.05f;   // EMA weight for the innovation-bias detector
  float _biasSigmaScale = 0.1601f;  // sqrt(a/(2-a)) for a = 0.05
  float _innovBias  = 0.0f;

  float _h = 0.0f;
  float _v = 0.0f;

  // Symmetric 2x2 covariance: [ _p00 _p01 ; _p01 _p11 ]
  float _p00 = 0.0f;
  float _p01 = 0.0f;
  float _p11 = 0.0f;

  bool  _initialised = false;
};


// Simple ring-buffer moving average, kept around so the Kalman output can be
// compared against the obvious baseline during bench testing.
template <size_t N>
class MovingAverage
{
public:
  void reset()
  {
    _count = 0;
    _idx   = 0;
    _sum   = 0.0f;
  }

  float push(float x)
  {
    if (_count == N) _sum -= _buf[_idx];
    else             _count++;
    _buf[_idx] = x;
    _sum += x;
    _idx = (_idx + 1) % N;
    return value();
  }

  float value() const { return _count ? _sum / static_cast<float>(_count) : 0.0f; }
  bool  full()  const { return _count == N; }

private:
  float  _buf[N] = {};
  size_t _count  = 0;
  size_t _idx    = 0;
  float  _sum    = 0.0f;
};
