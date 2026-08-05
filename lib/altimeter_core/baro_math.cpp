#include "baro_math.h"

#include <cmath>

namespace baro {

namespace {
constexpr float kT0overL   = 44330.77f;
constexpr float kExponent  = 0.1902632f;
}  // namespace

float pressureAltitude(float pressure_hPa, float seaLevel_hPa)
{
  if (pressure_hPa <= 0.0f || seaLevel_hPa <= 0.0f) return 0.0f;
  return kT0overL * (1.0f - powf(pressure_hPa / seaLevel_hPa, kExponent));
}

float pressureAtAltitude(float altitude_m, float seaLevel_hPa)
{
  const float base = 1.0f - altitude_m / kT0overL;
  if (base <= 0.0f) return 0.0f;
  return seaLevel_hPa * powf(base, 1.0f / kExponent);
}

float seaLevelPressure(float pressure_hPa, float altitude_m)
{
  if (pressure_hPa <= 0.0f) return kISASeaLevel_hPa;
  return pressure_hPa / powf(1.0f - altitude_m / kT0overL, 1.0f / kExponent);
}

}  // namespace baro
