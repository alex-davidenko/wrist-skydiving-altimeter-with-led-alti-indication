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
#include "jump_detect.h"
#include "velocity_window.h"
#include "flight_mode.h"
#include "zones.h"

// The mode thresholds are not in the log header, so take them from the same
// config.h the firmware compiles against — replay must not carry its own copy.
#include "config.h"

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

  // The phase machine and the aircraft latch, reconstructed. These are what
  // actually decide the colour, and jump 1 showed the zone can be perfectly
  // correct while the screen shows none of it.
  FlightModeTracker modes;
  modes.begin({MODE_FREEFALL_ENTER_MPS, MODE_FREEFALL_EXIT_MPS,
               MODE_CLIMB_ENTER_MPS, MODE_CLIMB_EXIT_MPS, MODE_DWELL_MS});
  bool inAircraft = false;
  size_t phaseMismatch = 0;

  // The proposed fix, run side by side with the real behaviour on the same
  // samples, so the comparison cannot drift.
  VelocityWindow vwin;
  vwin.begin(VELOCITY_WINDOW_MS, 96);
  FlightModeTracker modes2;
  modes2.begin({MODE_FREEFALL_ENTER_MPS, MODE_FREEFALL_EXIT_MPS,
                MODE_CLIMB_ENTER_MPS, MODE_CLIMB_EXIT_MPS, MODE_DWELL_MS});
  // Optional third argument: write the detected jump out as a JUMPnnnn.CSV in
  // the format the firmware now produces, so old flights can be replayed into
  // a logbook that did not exist when they were flown.
  FILE *out = (argc > 3) ? fopen(argv[3], "w") : nullptr;
  const unsigned outNum = (argc > 2) ? (unsigned)atoi(argv[2]) : 0;
  bool outHeader = false;
  float oExit = 0, oOpen = 0, oMaxDesc = 0;
  uint32_t oFfStart = 0, oFfEnd = 0, oBestDur = 0;
  float oPeak = -1e9f; uint32_t oPeakMs = 0;
  float cExit = 0, cMax = 0; uint32_t cStart = 0;
  bool oInFf = false;

  JumpDetector jd;
  jd.begin({JUMP_START_ALT_M, JUMP_STOP_ALT_M, JUMP_FREEFALL_MPS,
            JUMP_STILL_MPS, JUMP_SETTLE_MS, JUMP_MAX_MS});
  bool jdWas = false;
  size_t jdRows = 0, jdFiles = 0;
  uint32_t jdStartMs = 0;
  float jdStartAlt = 0;

  AircraftLatch latch2;
  latch2.begin(AIRCRAFT_LATCH_ALT_M, AIRCRAFT_CLEAR_ALT_M, AIRCRAFT_DESCENT_CONFIRM_M);
  size_t scrZone2 = 0, scrBlue2 = 0, scrLadder2 = 0;
  size_t rearm = 0, rearm2 = 0;
  // What flightPattern() would have painted, counted over the descent only.
  size_t scrZone = 0, scrBlue = 0, scrLadder = 0;
  bool descending = false;

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

    const uint8_t ph = modes.update(filt.velocity(), r.tMs);
    if (ph != r.phase) phaseMismatch++;

    // Mirrors main.cpp exactly: a single CLIMB sample above the latch altitude
    // re-arms it, and only FREEFALL or dropping below the clear altitude
    // releases it. That is what suppressed jump 4's landing ladder.
    const float alt = filt.altitude();
    if (ph == MODE_CLIMB && alt > AIRCRAFT_LATCH_ALT_M) { if (descending && !inAircraft) rearm++; inAircraft = true; }
    if (ph == MODE_FREEFALL || alt < AIRCRAFT_CLEAR_ALT_M) inAircraft = false;

    if (r.filtM >= apogee) descending = false;
    else if (r.filtM < apogee - 50.0f) descending = true;
    const float vw = vwin.update(filt.altitude(), r.tMs);
    const uint8_t ph2 = vwin.ready() ? modes2.update(vw, r.tMs) : (uint8_t)MODE_CANOPY;
    const bool inAircraft2 = latch2.update(alt, ph2);
    if (descending && !inAircraft2 && latch2.inAircraft()) rearm2++;

    const bool jrec = jd.update(alt, vw, ph2, r.tMs);
    if (out && jrec)
    {
      if (!outHeader)
      {
        fprintf(out, "# altimeter jump log\n# mode=FLIGHT sample_hz=%d osr=12 math_mode=1\n",
                h.sampleHz);
        fprintf(out, "# qnh=%.2f ground_p=%.3f\n", h.qnh, h.groundP);
        fprintf(out, "# extracted from %s by tools/replay\n", argv[1]);
        fprintf(out, "t_ms,p_hpa,temp_c,alt_raw_m,alt_filt_m,vs_mps,zone,phase,ground_p\n");
        outHeader = true;
      }
      // The LONGEST freefall episode is the jump. Not the last, which is the
      // velocity window twitching near the ground during the flare (1.4 s from
      // 423 m), and not the first, which is a ~1 s blip at exit while the
      // window still holds climb data. Only duration separates them.
      if (alt > oPeak) { oPeak = alt; oPeakMs = r.tMs; }
      if (ph2 == MODE_FREEFALL && !oInFf)
      { oInFf = true; cExit = alt; cStart = r.tMs; cMax = 0; }
      if (oInFf && -vw > cMax) cMax = -vw;
      if (ph2 != MODE_FREEFALL && oInFf)
      {
        oInFf = false;
        if (r.tMs - cStart > oBestDur)
        {
          oBestDur = r.tMs - cStart;
          oExit = cExit; oOpen = alt; oFfStart = cStart; oFfEnd = r.tMs;
          oMaxDesc = cMax;
        }
      }
      fprintf(out, "%lu,%.3f,%.2f,%.3f,%.3f,%.2f,%d,%d,%.3f\n",
              (unsigned long)r.tMs, r.pHpa, r.tempC, r.rawM, r.filtM,
              vw, (int)z, (int)ph2, r.groundP);
    }
    if (jrec) jdRows++;
    if (jrec && !jdWas) { jdStartMs = r.tMs; jdStartAlt = alt; jdFiles++; }
    if (jd.justAborted())
      printf("  discarded a recording that never saw freefall (t=%.0f..%.0f s)\n",
             jdStartMs / 1000.0, r.tMs / 1000.0);
    if (out && jd.justFinished())
    {
      // Apogee to opening. Both ends well determined, unlike the exit-side
      // mode transition, which fragments while the window still holds climb.
      const double ffS = (oFfEnd > oPeakMs) ? (oFfEnd - oPeakMs) / 1000.0 : 0.0;
      const double canS = oFfEnd ? (r.tMs - oFfEnd) / 1000.0 : 0.0;
      const double climbS = (oPeakMs > jdStartMs) ? (oPeakMs - jdStartMs) / 1000.0 : 0.0;
      fprintf(out, "# jump=%u exit_m=%.0f open_m=%.0f\n", outNum, oPeak, oOpen);
      fprintf(out, "# freefall_s=%.1f canopy_s=%.1f avg_freefall_mps=%.1f avg_climb_mps=%.1f\n",
              ffS, canS, ffS > 1.0 ? (oPeak - oOpen) / ffS : 0.0,
              climbS > 0 ? oPeak / climbS : 0.0);
      fclose(out); out = nullptr;
    }
    if (jd.justFinished())
      printf("  jump %zu: t=%.0f..%.0f s, %.0f m -> %.0f m, peak %.0f m, %.1f min\n",
             jdFiles, jdStartMs / 1000.0, r.tMs / 1000.0, jdStartAlt, alt,
             jd.peakAltM(), (r.tMs - jdStartMs) / 60000.0);
    jdWas = jrec;

    if (descending && alt > 5.0f)
    {
      // Same order of tests as led::flightPattern().
      if (ph == MODE_FREEFALL)                 scrZone++;
      else if (ph == MODE_CLIMB || inAircraft) scrBlue++;
      else                                     scrLadder++;

      if (ph2 == MODE_FREEFALL)                  scrZone2++;
      else if (ph2 == MODE_CLIMB || inAircraft2) scrBlue2++;
      else                                       scrLadder2++;
    }

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

  printf("JUMP DETECTION (files this would have produced)\n");
  printf("  %zu file(s), %zu of %zu rows recorded (%.1f%% of the log)\n\n",
         jdFiles, jdRows, rows.size(), 100.0 * jdRows / rows.size());

  printf("VERIFICATION (replay vs what the device logged)\n");
  printf("  baro maths  : max %.4f m\n", worstRaw);
  printf("  filter      : %.4f m at the 99.9th percentile (peak %.4f m)\n",
         p999, worstFilt);
  printf("  zone        : %zu / %zu samples differ\n", zoneMismatch, rows.size());
  printf("  phase       : %zu / %zu samples differ\n", phaseMismatch, rows.size());
  const bool trust = p999 < 0.05 && worstRaw < 0.10 &&
                     zoneMismatch * 200 < rows.size() &&
                     phaseMismatch * 50 < rows.size();
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

  {
    const size_t tot = scrZone + scrBlue + scrLadder;
    if (tot)
    {
      printf("WHAT THE SCREEN SHOWED, descent only (%zu samples)\n", tot);
      printf("  zone colours : %5.1f%%\n", 100.0 * scrZone / tot);
      printf("  blue (climb/in-aircraft) : %5.1f%%\n", 100.0 * scrBlue / tot);
      printf("  landing ladder : %5.1f%%\n", 100.0 * scrLadder / tot);
      printf("  --- with velocity from a %d ms window ---\n", (int)VELOCITY_WINDOW_MS);
      printf("  zone colours : %5.1f%%\n", 100.0 * scrZone2 / tot);
      printf("  blue (climb/in-aircraft) : %5.1f%%\n", 100.0 * scrBlue2 / tot);
      printf("  landing ladder : %5.1f%%\n", 100.0 * scrLadder2 / tot);
      printf("  aircraft-latch re-arms during descent: %zu (old) -> %zu (new)", rearm, rearm2);
      printf("\n\n");
    }
  }

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
