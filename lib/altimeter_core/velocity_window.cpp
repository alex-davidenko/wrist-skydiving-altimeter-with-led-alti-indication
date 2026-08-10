#include "velocity_window.h"

void VelocityWindow::begin(uint32_t windowMs, uint16_t maxSamples)
{
  _windowMs = windowMs;
  _max = maxSamples > kCap ? kCap : (maxSamples ? maxSamples : 1);
  reset();
}

void VelocityWindow::reset()
{
  _head = 0; _count = 0; _v = 0.0f; _ready = false;
}

float VelocityWindow::update(float altitudeM, uint32_t nowMs)
{
  _alt[_head] = altitudeM;
  _t[_head]   = nowMs;
  _head = static_cast<uint16_t>((_head + 1) % _max);
  if (_count < _max) _count++;

  // Least squares over everything inside the window. Time is taken relative to
  // the newest sample so the sums stay small and the arithmetic stays in float
  // even after millis() has run for hours.
  const uint32_t newest = nowMs;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  uint16_t n = 0;
  for (uint16_t k = 0; k < _count; k++)
  {
    const uint16_t idx = static_cast<uint16_t>((_head + _max - 1 - k) % _max);
    const uint32_t age = newest - _t[idx];
    if (age > _windowMs) break;
    const double x = -static_cast<double>(age) * 0.001;   // seconds, <= 0
    const double y = _alt[idx];
    sx += x; sy += y; sxx += x * x; sxy += x * y;
    n++;
  }

  // Two points on the same millisecond carry no slope information.
  const double den = n * sxx - sx * sx;
  if (n < 3 || den <= 1e-9)
  {
    _ready = false;
    return _v;
  }
  _v = static_cast<float>((n * sxy - sx * sy) / den);

  // Ready once the span covers most of the window, not merely once the ring is
  // full — a burst of samples in 50 ms fills the ring and means nothing.
  const uint16_t oldest = static_cast<uint16_t>((_head + _max - n) % _max);
  _ready = (newest - _t[oldest]) * 10 >= _windowMs * 8;
  return _v;
}
