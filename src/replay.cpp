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

  // Derive velocity here rather than trust the logged vs_mps column, and smooth
  // the altitude a little.
  //
  // The log's vs_mps is the Kalman velocity state, which is the thing four
  // jumps proved unusable: it peaked at 2326 m/s and read as CLIMBING for 38%
  // of freefall. Replaying it feeds the phase machine the exact garbage the
  // fix exists to avoid, so a replay would show the old bugs rather than what
  // the device does now. The live firmware takes velocity from a window of
  // altitude; so does this, over the same 1.5 s.
  //
  // The altitude gets a light 0.5 s mean for the same reason: alt_filt_m in
  // freefall is nearly raw, because the adaptive gate had degenerated into a
  // pass-through. This is what a working filter would have produced.
  {
    // 2.5 s, centred. Far heavier than the live path would tolerate, and it
    // can be: a replay has no latency requirement, so the window costs nothing
    // that matters. Measured on jump 185 across the freefall, worst second:
    //
    //   0.5 s smoothing   -7 m to 142 m      (true is about 50 m)
    //   1.5 s             +8 m to 108 m
    //   2.5 s             +8 m to  89 m
    //
    // Those plateaus and 100 m lurches are what reads as the display freezing
    // and then jumping. They are not a stall — the loop was measured clean
    // throughout — they are the +/-40 m of turbulence the static port is meant
    // to fix, and no amount of smoothing removes them entirely. Going past
    // ~2.5 s starts blurring the canopy opening, which is a real 2-4 s event
    // worth seeing.
    constexpr int kSmooth = 25;              // 2.5 s at 10 Hz
    constexpr int kWin    = 15;              // 1.5 s, matching VELOCITY_WINDOW_MS
    static float tmp[kMaxRows];
    for (uint16_t i = 0; i < g_len; i++)
    {
      int lo = (i >= kSmooth / 2) ? i - kSmooth / 2 : 0;
      int hi = (i + kSmooth / 2 < g_len) ? i + kSmooth / 2 : g_len - 1;
      float acc = 0; int n2 = 0;
      for (int k = lo; k <= hi; k++) { acc += g_buf[k].alt; n2++; }
      tmp[i] = acc / n2;
    }
    for (uint16_t i = 0; i < g_len; i++) g_buf[i].alt = tmp[i];

    // Least-squares slope over the trailing window, same shape as
    // VelocityWindow. Uniform sample spacing, so the x-terms are constants.
    for (uint16_t i = 0; i < g_len; i++)
    {
      const int lo = (i >= (uint16_t)kWin) ? i - kWin : 0;
      const int n2 = i - lo + 1;
      if (n2 < 3) { g_buf[i].vs = 0.0f; continue; }
      const float mx = (n2 - 1) * 0.5f;
      float my = 0;
      for (int k = 0; k < n2; k++) my += g_buf[lo + k].alt;
      my /= n2;
      float num = 0, den = 0;
      for (int k = 0; k < n2; k++)
      {
        const float dx = k - mx;
        num += dx * (g_buf[lo + k].alt - my);
        den += dx * dx;
      }
      g_buf[i].vs = den > 0 ? (num / den) * kHz : 0.0f;
    }
  }


  // Find the EXIT, and find it from the derived velocity rather than from
  // altitude. Peak and exit are not the same point: the aircraft levels off on
  // jump run, so maximum altitude lands somewhere in a minute of level flight
  // decided by noise — two reasonable implementations put it 47 s apart on this
  // jump, both "correct". An altitude threshold does not work either, because
  // with the door open the reading wobbles +/-40 m while perfectly level.
  //
  // Sustained descent is unambiguous. Find a second of real freefall, then walk
  // back to where the descent started, then back the lead-in.
  uint16_t exitIdx = peakIdx;
  for (uint16_t i = peakIdx; i + 10 < g_len; i++)
  {
    bool sustained = true;
    for (int k = 0; k < 10; k++)
      if (g_buf[i + k].vs > -25.0f) { sustained = false; break; }
    if (sustained) { exitIdx = i; break; }
  }
  while (exitIdx > 0 && g_buf[exitIdx].vs < -5.0f) exitIdx--;

  const uint16_t lead = (uint16_t)(kLeadInS * kHz);
  g_start = (exitIdx > lead) ? (uint16_t)(exitIdx - lead) : 0;

  Serial.printf("replay: peak %.0f m at %u, exit at %u, starting at %u of %u\n",
                (double)peak, peakIdx, exitIdx, g_start, g_len);
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
  //
  // dt is clamped because nowMs can be OLDER than g_lastMs. loop() takes its
  // timestamp once at the top, the gesture handler then blocks for ~2.7 s
  // loading the card inside that same iteration, and start() sets g_lastMs
  // after that — so the first update() arrives with a stale `now`, the
  // unsigned subtraction underflows to about four billion, and the whole
  // descent is consumed in a single frame. Which is exactly what it did: the
  // replay ran perfectly, once, in under a millisecond.
  int32_t dt = static_cast<int32_t>(nowMs - g_lastMs);
  if (dt < 0 || dt > 500) dt = kStepMs;   // stale, wrapped, or a long stall
  g_lastMs = nowMs;
  g_accumMs += static_cast<uint32_t>(dt) * g_speed;
  while (g_accumMs >= kStepMs && g_pos + 1 < g_len)
  {
    g_accumMs -= kStepMs;
    g_pos++;
  }
  if (g_pos + 1 >= g_len)
  {
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
