#include "replay.h"

#include "config.h"

#if !LOGGER_ENABLED || BENCH_MODE

namespace replay {
bool load(const char *) { return false; }
void start() {}
void stop() {}
bool active() { return false; }
void setSpeed(uint8_t) {}
uint8_t speed() { return 1; }
bool update(uint32_t) { return false; }
uint8_t percent() { return 0; }
}  // namespace replay

#else

#include <SD_MMC.h>

#include "display.h"
#include "flight_mode.h"
#include "led.h"
#include "zones.h"

namespace replay {
namespace {

// 10 Hz is plenty to watch: the altitude digits update at 20 Hz live and the
// number only moves a few metres between frames even in freefall. A five-minute
// jump is ~3000 samples, 12 KB.
constexpr uint32_t kHz       = 10;
constexpr uint32_t kStepMs   = 1000 / kHz;
constexpr uint16_t kMaxRows  = 4000;          // ~6.5 minutes
constexpr float    kLeadInS  = 3.0f;          // seen before the exit

struct Sample { float alt; float vs; };

Sample  *g_buf = nullptr;
uint16_t g_len = 0;
uint16_t g_pos = 0;
uint16_t g_start = 0;                          // where playback begins
bool     g_active = false;
uint8_t  g_speed = 1;
uint32_t g_lastMs = 0;
uint32_t g_accumMs = 0;

ZoneTracker       g_zones;
ZoneTracker       g_landing;
FlightModeTracker g_modes;
bool              g_inAircraft = false;

}  // namespace

bool load(const char *path)
{
  if (!path || !*path) return false;

  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return false;

  if (!g_buf)
  {
    g_buf = static_cast<Sample *>(ps_malloc(sizeof(Sample) * kMaxRows));
    if (!g_buf) g_buf = static_cast<Sample *>(malloc(sizeof(Sample) * kMaxRows));
    if (!g_buf) { f.close(); return false; }
  }
  g_len = 0;

  // Keep one sample per kStepMs of logged time. The log is 40 Hz, so this takes
  // every fourth row — but by timestamp rather than by count, because a dropped
  // row would otherwise shift everything after it.
  char line[160];
  size_t n = 0;
  uint32_t nextT = 0;
  bool first = true;
  float peak = -1e9f;
  uint16_t peakIdx = 0;

  while (f.available() && g_len < kMaxRows)
  {
    const int c = f.read();
    if (c < 0) break;
    if (c != '\n')
    {
      if (n < sizeof(line) - 1) line[n++] = (char)c;
      continue;
    }
    line[n] = '\0';
    n = 0;
    if (line[0] == '#' || line[0] == 't') continue;

    // t_ms,p_hpa,temp_c,alt_raw_m,alt_filt_m,vs_mps,zone,phase,ground_p
    uint32_t t = 0; float alt = 0, vs = 0;
    int field = 0; const char *p = line;
    for (;;)
    {
      if      (field == 0) t   = strtoul(p, nullptr, 10);
      else if (field == 4) alt = strtof(p, nullptr);
      else if (field == 5) { vs = strtof(p, nullptr); break; }
      const char *comma = strchr(p, ',');
      if (!comma) break;
      p = comma + 1;
      field++;
    }
    if (field < 5) continue;

    if (first) { nextT = t; first = false; }
    if (t + 1 < nextT) continue;               // not yet time for another sample
    nextT = t + kStepMs;

    g_buf[g_len] = {alt, vs};
    if (alt > peak) { peak = alt; peakIdx = g_len; }
    g_len++;

    if ((g_len & 0x3F) == 0) yield();          // a 3 MB read is a long time to hold the CPU
  }
  f.close();

  if (g_len < 20) { g_len = 0; return false; }

  // Start a few seconds before apogee. Apogee is the exit — the device is on
  // the jumper — so this opens on the aircraft and then the jump, rather than
  // on a wall of green already in freefall.
  const uint16_t lead = (uint16_t)(kLeadInS * kHz);
  g_start = (peakIdx > lead) ? (uint16_t)(peakIdx - lead) : 0;
  return true;
}

void start()
{
  if (!g_len) return;
  g_pos = g_start;
  g_active = true;
  g_speed = 1;
  g_lastMs = millis();
  g_accumMs = 0;

  g_zones.begin(kFlightZoneConfig);     // flight scale even in a bench build
  g_landing.begin(kLandingConfig);
  g_modes.begin({MODE_FREEFALL_ENTER_MPS, MODE_FREEFALL_EXIT_MPS,
                 MODE_CLIMB_ENTER_MPS, MODE_CLIMB_EXIT_MPS, MODE_DWELL_MS,
                 MODE_EXIT_FREEFALL_DWELL_MS});
  const float a0 = g_buf[g_pos].alt;
  g_zones.reset(a0);
  g_landing.reset(a0);
  g_modes.reset(MODE_CLIMB);
  g_inAircraft = true;                  // we open on the aircraft, by construction
}

void stop()
{
  g_active = false;
  display::setTopText("");
}

bool active() { return g_active; }

void setSpeed(uint8_t mult)
{ g_speed = (mult < 1) ? 1 : (mult > 3 ? 3 : mult); }

uint8_t speed() { return g_speed; }

uint8_t percent()
{
  if (g_len <= g_start + 1) return 0;
  return (uint8_t)(100UL * (g_pos - g_start) / (g_len - g_start - 1));
}

bool update(uint32_t nowMs)
{
  if (!g_active || !g_len) return false;

  // Advance the profile clock, which is the only thing speed affects. Blink
  // rates below run on nowMs, so 3 Hz and 6 Hz stay honest at 3x.
  const uint32_t dt = nowMs - g_lastMs;
  g_lastMs = nowMs;
  g_accumMs += dt * g_speed;
  while (g_accumMs >= kStepMs && g_pos + 1 < g_len)
  {
    g_accumMs -= kStepMs;
    g_pos++;
  }
  if (g_pos + 1 >= g_len) { stop(); return false; }

  const float alt = g_buf[g_pos].alt;
  const float v   = g_buf[g_pos].vs;

  // Same trackers, same thresholds, same pattern selection as the live loop.
  const uint8_t zone = g_zones.update(alt, nowMs);
  const uint8_t land = g_landing.update(alt, nowMs);
  const FlightMode mode = g_modes.update(v, nowMs);

  if (mode == MODE_CLIMB && alt > AIRCRAFT_LATCH_ALT_M) g_inAircraft = true;
  if (mode == MODE_FREEFALL || alt < AIRCRAFT_CLEAR_ALT_M) g_inAircraft = false;

  display::publish(led::flightPattern(mode, zone, land, alt, g_inAircraft), alt, v);
  return true;
}

}  // namespace replay

#endif
