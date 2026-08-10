#pragma once
#include <stdint.h>

// Vertical speed as a least-squares slope over a time window of altitude.
//
// WHY THIS EXISTS. The Kalman filter's velocity state is unusable in freefall.
// Measured over four jumps, raw altitude noise rises from 0.22 m on the ground
// to 9.5-16.5 m RMS in freefall — broadband turbulence around the wrist, not
// outliers — and the adaptive gate responds to the sustained innovation of a
// 50 m/s descent by inflating to its cap and staying there, which leaves the
// filter copying the measurement. Velocity then degenerates to the difference
// of adjacent noisy samples: 10 m over 25 ms is 400 m/s. Logged peaks reached
// 2326 m/s, and 38-45% of freefall samples read as CLIMBING.
//
// A window fixes it because the signal grows with the window and the noise does
// not. Falling at 50 m/s moves 75 m in 1.5 s against ~16 m of noise. Measured on
// the two worst jumps, false-climb samples in freefall:
//
//   0.25 s window   27.2%        1.0 s window    4.1%
//   0.5 s  window   15.7%        1.5 s window    0.0%
//
// A median filter was tried first and rejected: median-5 barely beat mean-5,
// which is the signature of broadband noise rather than spikes, and once the
// window is in place it contributes nothing (36.0 vs 36.7 m/s error).
//
// THE LAG IS DELIBERATE AND CHEAP. A trailing window estimates the slope at its
// centre, so this reads ~half a window stale — 0.75 s, or 37 m at freefall
// speed. That is 37 m of *velocity* staleness, not altitude error, and nothing
// safety-critical is downstream of it: the colour zones are driven directly by
// altitude, which was never broken. It costs a slightly late FREEFALL detection
// three kilometres up. If the zones ever come to depend on velocity, revisit
// this trade.
class VelocityWindow
{
 public:
  // `windowMs` of history; `maxSamples` caps the ring.
  void begin(uint32_t windowMs, uint16_t maxSamples);
  void reset();

  // Feed one altitude sample. Returns the current slope in m/s, positive up.
  float update(float altitudeM, uint32_t nowMs);

  float velocity() const { return _v; }
  // False until the window has filled enough to mean anything. Callers should
  // not make mode decisions before this — an empty window reads zero, which
  // looks exactly like sitting still.
  bool ready() const { return _ready; }

 private:
  static const uint16_t kCap = 96;
  float    _alt[kCap] = {0};
  uint32_t _t[kCap]   = {0};
  uint16_t _head = 0, _count = 0, _max = kCap;
  uint32_t _windowMs = 1500;
  float    _v = 0.0f;
  bool     _ready = false;
};
