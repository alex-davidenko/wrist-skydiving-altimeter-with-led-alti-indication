#pragma once
//
// Pressure -> altitude conversion (ISA barometric formula).
// Pure math, no Arduino dependency, so it can be unit-tested on the host.
//

namespace baro {

// Standard sea-level pressure, hPa (== mbar).
constexpr float kISASeaLevel_hPa = 1013.25f;

// ISA pressure altitude in metres for a given static pressure.
//
//   h = (T0/L) * (1 - (p/p0)^(R*L/(g*M)))
//
// with T0 = 288.15 K, L = 0.0065 K/m  ->  T0/L = 44330.77 m
// and the exponent = 0.1902632.
//
// `seaLevel_hPa` is the QNH reference. Leave it at ISA unless you actually
// know the local QNH; for AGL work the ground-reference offset (below) is
// what matters, not the absolute number.
float pressureAltitude(float pressure_hPa, float seaLevel_hPa = kISASeaLevel_hPa);

// Inverse: what sea-level pressure would put `pressure_hPa` at `altitude_m`.
// Useful if you know the DZ elevation and want a real QNH rather than a
// zeroed AGL reference.
float seaLevelPressure(float pressure_hPa, float altitude_m);

}  // namespace baro
