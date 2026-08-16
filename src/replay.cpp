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
// 40 minutes at 10 Hz. Sized for the whole FLIGHT, not the jump: recording arms
// at 50 m on the climb, and a climb to altitude is twenty minutes. The first
// cut allowed 4000 samples — 6.7 minutes — so it stopped partway up, decided
// the highest point it had seen was the apogee, started playback 31 samples
// from the end and finished instantly. 24000 samples is 192 KB in PSRAM, which
// is nothing against 8 MB.
constexpr uint16_t kMaxRows  = 24000;
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
  // Block reads, not byte-at-a-time. The first version called f.read() per
  // character: three million SD calls for a 3 MB file, about ten seconds, long
  // enough that the menu timeout fired and dropped the whole thing back to the
  // altitude screen before playback ever started.
  static uint8_t io[4096];
  char line[160];
  size_t n = 0;
  uint32_t nextT = 0;
  bool first = true;
  float peak = -1e9f;
  uint16_t peakIdx = 0;
  uint32_t rows = 0;
  const uint32_t t0 = millis();

  while (g_len < kMaxRows)
  {
    const int got = f.read(io, sizeof(io));
    if (got <= 0) break;

    for (int i = 0; i < got && g_len < kMaxRows; i++)
    {
      const char c = (char)io[i];
      if (c != '\n')
      {
        if (n < sizeof(line) - 1) line[n++] = c;
        continue;
      }
      line[n] = '\0';
      n = 0;
      if (line[0] == '#' || line[0] == 't' || line[0] == '\0') continue;
      rows++;

      // t_ms,p_hpa,temp_c,alt_raw_m,alt_filt_m,vs_mps,zone,phase,ground_p
      uint32_t t = 0; float alt = 0, vs = 0;
      int field = 0; const char *p = line; bool ok = false;
      for (;;)
      {
        if      (field == 0) t   = strtoul(p, nullptr, 10);
        else if (field == 4) alt = strtof(p, nullptr);
        else if (field == 5) { vs = strtof(p, nullptr); ok = true; break; }
        const char *comma = strchr(p, ',');
        if (!comma) break;
        p = comma + 1;
        field++;
      }
      if (!ok) continue;

      if (first) { nextT = t; first = false; }
      if (t + 1 < nextT) continue;
      nextT = t + kStepMs;

      g_buf[g_len] = {alt, vs};
      if (alt > peak) { peak = alt; peakIdx = g_len; }
      g_len++;
    }
    yield();
  }
  const uint32_t took = millis() - t0;
  f.close();

  Serial.printf("replay: %s — %lu rows read, %u kept, %lu ms\n",
                path, (unsigned long)rows, g_len, (unsigned long)took);
  if (g_len >= kMaxRows)
    Serial.println(F("replay: WARNING buffer full — file truncated, apogee may be wrong"));
  if (g_len < 20)
  {
    Serial.println(F("replay: too few usable samples — is this a jump file?"));
    g_len = 0;
    return false;
  }

  // Start a few seconds before apogee. Apogee is the exit — the device is on
  // the jumper — so this opens on the aircraft and then the jump, rather than
  // on a wall of green already in freefall.
  const uint16_t lead = (uint16_t)(kLeadInS * kHz);
  g_start = (peakIdx > lead) ? (uint16_t)(peakIdx - lead) : 0;
  Serial.printf("replay: apogee %.0f m at sample %u; starting at %u of %u\n",
                (double)peak, peakIdx, g_start, g_len);
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
  Serial.printf("replay: START pos=%u len=%u alt=%.0f active=%d\n",
                g_pos, g_len, (double)a0, (int)g_active);
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

  // Once a second while playing, so a replay that is running but invisible can
  // be told apart from one that is not running at all. Those two have looked
  // identical from the outside three times now.
  static uint32_t lastLog = 0;
  if (nowMs - lastLog > 1000)
  {
    lastLog = nowMs;
    Serial.printf("replay: pos=%u/%u alt=%.0f vs=%.1f x%u\n",
                  g_pos, g_len, (double)g_buf[g_pos].alt,
                  (double)g_buf[g_pos].vs, g_speed);
  }

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
  if (g_pos + 1 >= g_len)
  {
    Serial.printf("replay: END at pos=%u/%u\n", g_pos, g_len);
    stop();
    return false;
  }

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
