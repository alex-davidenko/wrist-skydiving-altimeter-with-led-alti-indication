// Replay a jump log through the firmware's own filter and zone code.
//
// This links lib/altimeter_core directly — the same translation units the
// firmware builds — so a replay cannot silently drift from what the device
// actually did. That is the whole reason the log stores derived columns as
// well as raw ones: recompute from the raw, compare against what the device
// produced in the air, and you know whether the tooling is trustworthy before
// you use it to justify a tuning change.
//
// Build:
//   c++ -std=gnu++17 -O2 -I lib/altimeter_core tools/replay.cpp \
//       lib/altimeter_core/*.cpp -o /tmp/replay
// Run:
//   /tmp/replay LOG0001.CSV

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <algorithm>
#include <cctype>
#include <vector>

#include "altitude_filter.h"
#include "baro_math.h"
#include "flight_mode.h"
#include "zones.h"

namespace {

struct Row
{
  uint32_t tMs;
  float    pHpa, tempC, rawM, filtM, vsMps;
  int      zone, phase;
  float    groundP;      // 0 if the log predates this column
};

struct Header
{
  std::string mode = "?";
  float sigmaAccel = 3.0f, sigmaMeas = 0.6f, gate = 2.0f, inflate = 500.0f;
  float bounds[kMaxZoneBounds] = {0, 0, 0, 0, 0};
  float hystUrgent = 0.0f, hystRelax = 8.0f;
  uint32_t dwellUrgent = 50, dwellRelax = 400;
  float qnh = 1013.25f, groundP = 0.0f;
  int   sampleHz = 40;
};

// Pull "key=value" out of a header line if present.
bool kv(const std::string &line, const char *key, float *out)
{
  const size_t p = line.find(std::string(key) + "=");
  if (p == std::string::npos) return false;
  *out = strtof(line.c_str() + p + strlen(key) + 1, nullptr);
  return true;
}

const char *zname(int z) { return zoneName(static_cast<uint8_t>(z)); }

}  // namespace

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    fprintf(stderr, "usage: %s <log.csv>\n", argv[0]);
    return 2;
  }
  FILE *f = fopen(argv[1], "r");
  if (!f) { perror(argv[1]); return 1; }

  Header h;
  std::vector<Row> rows;
  char line[512];

  while (fgets(line, sizeof(line), f))
  {
    if (line[0] == '#')
    {
      std::string s(line);
      float v;
      if (kv(s, "sigma_accel", &v))  h.sigmaAccel = v;
      if (kv(s, "sigma_meas", &v))   h.sigmaMeas = v;
      if (kv(s, "gate", &v))         h.gate = v;
      if (kv(s, "inflate", &v))      h.inflate = v;
      if (kv(s, "hyst_urgent", &v))  h.hystUrgent = v;
      if (kv(s, "hyst_relax", &v))   h.hystRelax = v;
      if (kv(s, "dwell_urgent", &v)) h.dwellUrgent = (uint32_t)v;
      if (kv(s, "dwell_relax", &v))  h.dwellRelax = (uint32_t)v;
      if (kv(s, "qnh", &v))          h.qnh = v;
      if (kv(s, "ground_p", &v))     h.groundP = v;
      if (kv(s, "sample_hz", &v))    h.sampleHz = (int)v;
      const size_t m = s.find("mode=");
      if (m != std::string::npos) h.mode = s.substr(m + 5, s.find(' ', m) - m - 5);
      const size_t b = s.find("bounds=");
      if (b != std::string::npos)
        sscanf(s.c_str() + b + 7, "%f,%f,%f,%f,%f", &h.bounds[0], &h.bounds[1],
               &h.bounds[2], &h.bounds[3], &h.bounds[4]);
      continue;
    }
    if (!isdigit((unsigned char)line[0])) continue;   // column header

    Row r{};
    const int n = sscanf(line, "%u,%f,%f,%f,%f,%f,%d,%d,%f", &r.tMs, &r.pHpa,
                         &r.tempC, &r.rawM, &r.filtM, &r.vsMps, &r.zone,
                         &r.phase, &r.groundP);
    if (n >= 8)
    {
      if (n < 9) r.groundP = h.groundP;   // older log: header value is all we have
      rows.push_back(r);
    }
  }
  fclose(f);

  if (rows.size() < 2) { fprintf(stderr, "no data rows\n"); return 1; }

  printf("%s: %zu rows, %s mode, %d Hz, QNH %.2f, ground %.3f hPa\n\n",
         argv[1], rows.size(), h.mode.c_str(), h.sampleHz, h.qnh, h.groundP);

  // ---- re-run the real firmware code ------------------------------------
  AltitudeFilter filt(h.sigmaAccel, h.sigmaMeas);
  filt.configureAdaptive(h.gate, h.inflate);
  ZoneConfig zc{};
  memcpy(zc.bounds, h.bounds, sizeof(zc.bounds));
  zc.hystUrgent = h.hystUrgent;
  zc.hystRelax  = h.hystRelax;
  zc.dwellUrgentMs = h.dwellUrgent;
  zc.dwellRelaxMs  = h.dwellRelax;
  ZoneTracker zones;
  zones.begin(zc);

  float lastGround = 0.0f;
  bool primed = false;
  double worstRaw = 0, worstFilt = 0;
  size_t zoneMismatch = 0;
  std::vector<double> filtErr;
  filtErr.reserve(rows.size());

  // ---- stats ------------------------------------------------------------
  float apogee = -1e9f, maxDescent = 0.0f;
  uint32_t exitMs = 0, openMs = 0, landMs = 0;
  float exitAlt = 0, openAlt = 0;

  for (size_t i = 0; i < rows.size(); i++)
  {
    const Row &r = rows[i];

    // Re-zeroing changes the reference and resets the device's filter. The
    // per-row ground_p is how a replay sees that happen; without it the
    // recomputed altitude drifts from the device's by the size of the
    // correction, for the rest of the file.
    // Threshold, not exact inequality: floats that merely round differently
    // must not look like a re-zero. A real one moves the reference far more
    // than this — 0.005 hPa is about 4 cm of altitude.
    if (std::fabs(r.groundP - lastGround) > 0.005f)
    {
      lastGround = r.groundP;
      // Reset to exactly zero, which is what zeroHere() does on the device —
      // not to the first post-zero sample, which is merely near zero.
      if (primed) { filt.reset(0.0f); zones.reset(0.0f); }
    }

    // Does the barometric maths reproduce what the device logged?
    const float groundAlt =
        r.groundP > 0.0f ? baro::pressureAltitude(r.groundP, h.qnh) : 0.0f;
    const float rawRecomputed = baro::pressureAltitude(r.pHpa, h.qnh) - groundAlt;
    worstRaw = std::max<double>(worstRaw, std::fabs(rawRecomputed - r.rawM));

    if (!primed) { filt.reset(r.rawM); zones.reset(r.rawM); primed = true; }
    const float dt = (i == 0) ? 1.0f / h.sampleHz
                              : (r.tMs - rows[i - 1].tMs) * 1e-3f;
    filt.update(r.rawM, dt);
    const uint8_t z = zones.update(filt.altitude(), r.tMs);

    const double fe = std::fabs(filt.altitude() - r.filtM);
    worstFilt = std::max(worstFilt, fe);
    filtErr.push_back(fe);
    if (z != r.zone) zoneMismatch++;

    if (r.filtM > apogee) apogee = r.filtM;
    const float descent = -r.vsMps;
    if (descent > maxDescent) maxDescent = descent;
    if (!exitMs && descent > 20.0f) { exitMs = r.tMs; exitAlt = r.filtM; }
    if (exitMs && !openMs && descent < 15.0f && r.tMs > exitMs + 3000)
    { openMs = r.tMs; openAlt = r.filtM; }
    if (openMs && !landMs && r.filtM < 5.0f) landMs = r.tMs;
  }

  // ---- verification ------------------------------------------------------
  // Judge on a high percentile, not the peak. Zeroing blocks the device for
  // ~1.6 s and logs nothing, so a replay meets a large dt there and diverges
  // briefly. That is a known discontinuity, not evidence the tooling is wrong —
  // failing a whole jump on it would make the check useless.
  std::sort(filtErr.begin(), filtErr.end());
  const double p999 = filtErr.empty() ? 0.0
                    : filtErr[(size_t)(filtErr.size() * 0.999)];

  printf("VERIFICATION (replay vs what the device logged)\n");
  printf("  baro maths  : max %.4f m\n", worstRaw);
  printf("  filter      : %.4f m at the 99.9th percentile (peak %.4f m)\n",
         p999, worstFilt);
  printf("  zone        : %zu / %zu samples differ\n", zoneMismatch, rows.size());
  const bool trust = p999 < 0.05 && worstRaw < 0.10 &&
                     zoneMismatch * 200 < rows.size();
  printf("  -> %s\n\n", trust
             ? "match. Replay is faithful; tuning experiments are meaningful."
             : "MISMATCH. Check the firmware build matches this log's header "
               "before trusting any conclusion from it.");

  // ---- jump summary ------------------------------------------------------
  printf("JUMP\n");
  printf("  duration    : %.1f s\n", (rows.back().tMs - rows.front().tMs) / 1000.0);
  printf("  apogee      : %.0f m\n", apogee);
  if (exitMs)
  {
    printf("  exit        : %.0f m at t=%.1f s\n", exitAlt, exitMs / 1000.0);
    printf("  max descent : %.1f m/s\n", maxDescent);
  }
  else printf("  exit        : none detected (never exceeded 20 m/s)\n");
  if (openMs)
  {
    printf("  opening     : %.0f m at t=%.1f s\n", openAlt, openMs / 1000.0);
    printf("  freefall    : %.1f s, %.0f m\n", (openMs - exitMs) / 1000.0,
           exitAlt - openAlt);
  }
  if (landMs && openMs)
    printf("  canopy      : %.1f s, %.0f m\n", (landMs - openMs) / 1000.0, openAlt);

  // Time spent in each colour zone — what the jumper actually saw.
  printf("\nTIME PER ZONE\n");
  size_t count[kMaxZoneBounds + 1] = {0};
  for (const Row &r : rows)
    if (r.zone >= 0 && r.zone <= kMaxZoneBounds) count[r.zone]++;
  for (int z = 0; z <= kMaxZoneBounds; z++)
    if (count[z])
      printf("  %-10s %6.1f s\n", zname(z), count[z] / (double)h.sampleHz);

  return trust ? 0 : 1;
}
