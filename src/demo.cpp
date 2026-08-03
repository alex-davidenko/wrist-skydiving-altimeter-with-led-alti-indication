#include "demo.h"

#include "config.h"
#include "display.h"
#include "flight_mode.h"
#include "led.h"
#include "zones.h"

namespace demo {

namespace {

// One leg of the jump: how long it lasts and the vertical speed across it,
// ramped linearly from vStart to vEnd. Altitude is the integral, so the
// profile only has to describe speeds and the heights come out right.
struct Leg
{
  float       secs;
  float       vStart, vEnd;   // m/s, negative = descending
  const char *label;
};

// 4200 m exit, pilot chute at 1100, settled by ~800, five 5-second spirals at
// 15 m/s, then the landing ladder from 300 m down.
const Leg kLegs[] = {
    { 8.0f,   0.0f, -50.0f, "EXIT"},      // accelerating to terminal, -200 m
    {58.0f, -50.0f, -50.0f, "FREEFALL"},  // -2900 m  -> 1100 m
    { 4.0f, -50.0f,  -5.0f, "DEPLOY"},    // -110 m   -> ~990 m
    {38.0f,  -5.0f,  -5.0f, "CANOPY"},    // -190 m   -> 800 m
    {25.0f, -15.0f, -15.0f, "SPIRALS"},   // 5 x 5 s  -> 425 m
    {25.0f,  -5.0f,  -5.0f, "CANOPY"},    // -125 m   -> 300 m
    {60.0f,  -5.0f,  -5.0f, "LANDING"},   // -300 m   -> 0 m
};
constexpr int kLegCount = sizeof(kLegs) / sizeof(kLegs[0]);
constexpr float kStartAlt = 4200.0f;

bool     g_active   = false;
uint32_t g_startMs  = 0;
float    g_totalSecs = 0.0f;

ZoneTracker       g_zones;
ZoneTracker       g_landing;
FlightModeTracker g_modes;

// Altitude and speed at virtual time t, by integrating the legs.
void profileAt(float t, float *altOut, float *vOut, const char **legOut)
{
  float alt = kStartAlt;
  for (int i = 0; i < kLegCount; i++)
  {
    const Leg &l = kLegs[i];
    if (t > l.secs)
    {
      alt += 0.5f * (l.vStart + l.vEnd) * l.secs;   // whole leg
      t -= l.secs;
      continue;
    }
    const float v = l.vStart + (l.vEnd - l.vStart) * (t / l.secs);
    alt += 0.5f * (l.vStart + v) * t;               // partial leg
    *altOut = alt;
    *vOut   = v;
    *legOut = l.label;
    return;
  }
  *altOut = alt;
  *vOut   = 0.0f;
  *legOut = "DOWN";
}

}  // namespace

bool active() { return g_active; }

int secondsLeft()
{
  if (!g_active) return 0;
  const float elapsed = (millis() - g_startMs) * 0.001f * DEMO_SPEED;
  const int left = (int)((g_totalSecs - elapsed) / DEMO_SPEED);
  return left > 0 ? left : 0;
}

void start()
{
  g_totalSecs = 0.0f;
  for (int i = 0; i < kLegCount; i++) g_totalSecs += kLegs[i].secs;

  g_zones.begin(kFlightZoneConfig);   // always flight scale, even in a bench build
  g_landing.begin(kLandingConfig);
  g_modes.begin({MODE_FREEFALL_ENTER_MPS, MODE_FREEFALL_EXIT_MPS,
                 MODE_CLIMB_ENTER_MPS, MODE_CLIMB_EXIT_MPS, MODE_DWELL_MS});
  g_zones.reset(kStartAlt);
  g_landing.reset(kStartAlt);
  // CANOPY, not FREEFALL: you leave the aircraft at zero vertical speed, so
  // the screen really is dark for the first seconds until you accelerate past
  // the 20 m/s threshold. Starting in FREEFALL would show a green flash that
  // does not happen in the air.
  g_modes.reset(MODE_CANOPY);

  g_startMs = millis();
  g_active  = true;
  Serial.printf("demo: jump from %.0f m, %.0f s of profile at %.0fx\n",
                (double)kStartAlt, (double)g_totalSecs, (double)DEMO_SPEED);
}

void stop()
{
  if (!g_active) return;
  g_active = false;
  display::setScreen(display::UI_ALT);
  Serial.println(F("demo: stopped"));
}

bool update(uint32_t nowMs)
{
  if (!g_active) return false;

  const float t = (nowMs - g_startMs) * 0.001f * DEMO_SPEED;
  if (t >= g_totalSecs)
  {
    stop();
    return false;
  }

  float alt, v;
  const char *leg;
  profileAt(t, &alt, &v, &leg);

  // Same trackers, same thresholds, same pattern selection as the real thing.
  const uint8_t zone = g_zones.update(alt, nowMs);
  const uint8_t land = g_landing.update(alt, nowMs);
  const FlightMode mode = g_modes.update(v, nowMs);

  LedPattern p;
  switch (mode)
  {
    case MODE_FREEFALL: p = led::freefallPattern(zone); break;
    case MODE_CANOPY:   p = led::landingPattern(land);  break;
    default:            p = led::offPattern();          break;
  }

  display::publish(p, alt, v);
  return true;
}

}  // namespace demo
