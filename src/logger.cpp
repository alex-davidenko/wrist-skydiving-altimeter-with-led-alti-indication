#include "logger.h"

#include "config.h"
#include "zones.h"

#if !LOGGER_ENABLED

namespace logger {
bool begin(float, float) { return false; }
void startTask() {}
void push(uint32_t, float, float, float, float, float, uint8_t, uint8_t, float) {}
void close() {}
bool openJump(uint32_t, float, float) { return false; }
void closeJump(const Summary &) {}
void discardJump() {}
bool recording() { return false; }
void pause() {}
void resume() {}
void setEnabled(bool) {}
bool enabled() { return false; }
bool available() { return false; }
const char *filename() { return ""; }
uint32_t rowsWritten() { return 0; }
uint32_t rowsDropped() { return 0; }
uint64_t cardSizeMb() { return 0; }
}  // namespace logger

#else

#include <MS5611.h>   // only for SENSOR_OSR's enum, recorded in the header
#include <SD_MMC.h>

#include <atomic>
#include <time.h>

namespace logger {

namespace {

// One sample. Kept binary and formatted by the writer task, so the sample loop
// never pays for snprintf.
struct Rec
{
  uint32_t tMs;
  float    pHpa;
  float    tempC;
  float    rawM;
  float    filtM;
  float    vsMps;
  uint8_t  zone;
  uint8_t  phase;
  // Carried per row, not just in the header: re-zeroing mid-log changes the
  // reference, and a replay that assumed the header value diverged from the
  // device by half a metre from that point on. Now every row is self-describing
  // and a change of this value is also how a replay knows to reset its filter,
  // which is what the device does when you zero.
  float    groundP;
};

Rec     *g_ring    = nullptr;
uint32_t g_ringLen = 0;

// Single producer (sample loop), single consumer (writer task). Release/acquire
// on the indices is all the synchronisation an SPSC ring needs — no lock, so
// push() cannot ever block the sample loop.
std::atomic<uint32_t> g_head{0};
std::atomic<uint32_t> g_tail{0};

File     g_file;
char     g_name[24]  = {0};
bool     g_ok        = false;
bool     g_enabled   = true;
uint64_t g_cardMb    = 0;
float    g_qnh       = 1013.25f;
float    g_groundP   = 0.0f;
std::atomic<uint32_t> g_rows{0};
std::atomic<uint32_t> g_drops{0};
volatile bool g_paused = false;

void writeHeader()
{
  g_file.printf("# altimeter jump log\n");
  g_file.printf("# mode=%s sample_hz=%d osr=%d math_mode=%d\n",
                BENCH_MODE ? "BENCH" : "FLIGHT",
                1000 / SAMPLE_PERIOD_MS, SENSOR_OSR, SENSOR_MATH_MODE);
  g_file.printf("# filter sigma_accel=%.3f sigma_meas=%.3f gate=%.2f inflate=%.0f\n",
                (double)FILTER_SIGMA_ACCEL, (double)FILTER_SIGMA_MEAS,
                (double)FILTER_GATE_SIGMA, (double)FILTER_MAX_INFLATE);
  g_file.printf("# zones bounds=%.2f,%.2f,%.2f,%.2f,%.2f hyst_urgent=%.2f "
                "hyst_relax=%.2f dwell_urgent=%lu dwell_relax=%lu\n",
                (double)kZoneConfig.bounds[0], (double)kZoneConfig.bounds[1],
                (double)kZoneConfig.bounds[2], (double)kZoneConfig.bounds[3],
                (double)kZoneConfig.bounds[4],
                (double)kZoneConfig.hystUrgent, (double)kZoneConfig.hystRelax,
                (unsigned long)kZoneConfig.dwellUrgentMs,
                (unsigned long)kZoneConfig.dwellRelaxMs);
  // Recorded so a replay can go all the way from pressure to AGL without
  // guessing. Captured at file creation: re-zeroing mid-log is not reflected.
  g_file.printf("# qnh=%.2f ground_p=%.3f\n", (double)g_qnh, (double)g_groundP);
  // Wall clock at file creation. More reliable than the FAT timestamp, which
  // reads 1980 whenever the device has not been told the time.
  {
    const time_t now = time(nullptr);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", localtime(&now));
    g_file.printf("# started=%s\n", ts);
  }
  g_file.printf("# version=%s build=%s %s\n", FW_VERSION, __DATE__, __TIME__);
  g_file.printf("t_ms,p_hpa,temp_c,alt_raw_m,alt_filt_m,vs_mps,zone,phase,ground_p\n");
  g_file.flush();
}

// Pick the next unused LOGnnnn.CSV so a power cycle never overwrites a jump.
bool openNewFile(uint32_t number)
{
  for (uint16_t i = 0; i < 1000; i++)
  {
    // The jump number is the name. If one already exists — a counter reset, or
    // a card moved between devices — step forward rather than overwrite it.
    snprintf(g_name, sizeof(g_name), "/JUMP%04u.CSV", (unsigned)(number + i));
    if (!SD_MMC.exists(g_name))
    {
      g_file = SD_MMC.open(g_name, FILE_WRITE);
      if (!g_file) return false;
      writeHeader();
      return true;
    }
  }
  return false;
}

void writerTask(void *)
{
  uint32_t lastFlush = millis();
  char line[128];

  for (;;)
  {
    if (g_paused) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

    uint32_t tail = g_tail.load(std::memory_order_relaxed);
    const uint32_t head = g_head.load(std::memory_order_acquire);

    while (tail != head)
    {
      const Rec &r = g_ring[tail % g_ringLen];
      const int n = snprintf(line, sizeof(line),
                             "%lu,%.3f,%.2f,%.3f,%.3f,%.2f,%u,%u,%.3f\n",
                             (unsigned long)r.tMs, (double)r.pHpa, (double)r.tempC,
                             (double)r.rawM, (double)r.filtM, (double)r.vsMps,
                             r.zone, r.phase, (double)r.groundP);
      if (n > 0) g_file.write(reinterpret_cast<const uint8_t *>(line), n);
      g_rows.fetch_add(1, std::memory_order_relaxed);
      tail++;
    }
    g_tail.store(tail, std::memory_order_release);

    // Flush on a timer rather than per row: per-row flushing is what makes SD
    // logging slow, and a crash would cost at most this much data.
    const uint32_t now = millis();
    if (static_cast<uint32_t>(now - lastFlush) >= LOG_FLUSH_MS)
    {
      g_file.flush();
      lastFlush = now;
    }

    vTaskDelay(pdMS_TO_TICKS(LOG_TICK_MS));
  }
}

}  // namespace

bool available() { return g_ok; }
bool enabled()   { return g_ok && g_enabled; }

void close()
{
  if (!g_ok) return;
  g_enabled = false;
  // Let the writer task drain whatever is still queued before we close.
  const uint32_t deadline = millis() + 500;
  while (g_head.load(std::memory_order_acquire) != g_tail.load(std::memory_order_acquire) &&
         static_cast<int32_t>(millis() - deadline) < 0)
  {
    delay(10);
  }
  g_file.flush();
  g_file.close();
  SD_MMC.end();
  g_ok = false;
  Serial.printf("logger: %s closed, %lu rows. Card is safe to remove.\n",
                g_name, (unsigned long)rowsWritten());
}
void pause()
{
  if (!g_ok) return;
  g_paused = true;
  delay(60);                 // let the writer finish whatever it was doing
  g_file.flush();
}

void resume() { g_paused = false; }

void setEnabled(bool on) { g_enabled = on; }
const char *filename() { return g_name; }
uint32_t rowsWritten() { return g_rows.load(std::memory_order_relaxed); }
uint32_t rowsDropped() { return g_drops.load(std::memory_order_relaxed); }
uint64_t cardSizeMb()  { return g_cardMb; }

bool begin(float qnhHpa, float groundPHpa)
{
  g_qnh     = qnhHpa;
  g_groundP = groundPHpa;

  SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, PIN_SD_D1, PIN_SD_D2, PIN_SD_D3);

  // 4-bit mode; the factory demo proves the wiring supports it.
  if (!SD_MMC.begin("/sdcard", false /* 1-bit */, false /* format if fail */))
  {
    Serial.println(F("logger: no SD card (or not FAT32/MBR) — logging disabled"));
    return false;
  }

  g_cardMb = SD_MMC.cardSize() / (1024ULL * 1024ULL);

  // Ring buffer in PSRAM. Internal RAM is precious and this is bulk storage.
  g_ringLen = LOG_RING_RECORDS;
  g_ring = static_cast<Rec *>(ps_malloc(sizeof(Rec) * g_ringLen));
  if (!g_ring)
  {
    g_ring = static_cast<Rec *>(malloc(sizeof(Rec) * g_ringLen));
    if (!g_ring)
    {
      Serial.println(F("logger: ring buffer allocation failed"));
      SD_MMC.end();
      return false;
    }
  }

  g_ok = true;
  g_enabled = false;                 // nothing is recorded until a jump starts
  g_name[0] = '\0';
  Serial.printf("logger: ready on %llu MB card, ring %lu rows (%lu KB)\n",
                (unsigned long long)g_cardMb,
                (unsigned long)g_ringLen,
                (unsigned long)(sizeof(Rec) * g_ringLen / 1024));
  return true;
}

bool recording() { return g_ok && g_file; }

bool openJump(uint32_t number, float qnhHpa, float groundPHpa)
{
  if (!g_ok || g_file) return false;
  g_qnh = qnhHpa;
  g_groundP = groundPHpa;
  // Start from empty: the ring may hold samples from before the jump armed.
  g_tail.store(g_head.load(std::memory_order_acquire), std::memory_order_release);
  if (!openNewFile(number)) return false;
  g_enabled = true;
  Serial.printf("logger: recording jump %lu to %s\n", (unsigned long)number, g_name);
  return true;
}

void closeJump(const Summary &s)
{
  if (!g_ok || !g_file) return;
  g_enabled = false;
  const uint32_t deadline = millis() + 500;
  while (g_head.load(std::memory_order_acquire) != g_tail.load(std::memory_order_acquire) &&
         static_cast<int32_t>(millis() - deadline) < 0)
  {
    delay(10);
  }
  // Trailing rather than leading: the file is opened before any of this is
  // known, and rewriting a header in place on FAT is not worth the risk.
  g_file.printf("# jump=%lu exit_m=%.0f open_m=%.0f\n",
                (unsigned long)s.number, s.peakAltM, s.openAltM);
  g_file.printf("# freefall_s=%.1f canopy_s=%.1f avg_freefall_mps=%.1f avg_climb_mps=%.1f\n",
                s.freefallS, s.canopyS, s.avgFreefallMps, s.avgClimbMps);
  g_file.flush();
  g_file.close();
  Serial.printf("logger: jump %lu closed — %s\n", (unsigned long)s.number, g_name);
}

void discardJump()
{
  if (!g_ok || !g_file) return;
  g_enabled = false;
  g_file.close();
  SD_MMC.remove(g_name);
  Serial.printf("logger: discarded %s (no freefall — not a jump)\n", g_name);
  g_name[0] = '\0';
}

void startTask()
{
  if (!g_ok) return;
  xTaskCreatePinnedToCore(writerTask, "logger", LOG_TASK_STACK, nullptr,
                          LOG_TASK_PRIO, nullptr, LOG_TASK_CORE);
  Serial.printf("logger: writer task on core %d\n", LOG_TASK_CORE);
}

void push(uint32_t tMs, float pressureHpa, float tempC,
          float rawAglM, float filtAglM, float vsMps,
          uint8_t zone, uint8_t phase, float groundPHpa)
{
  if (!g_ok || !g_enabled) return;

  const uint32_t head = g_head.load(std::memory_order_relaxed);
  const uint32_t tail = g_tail.load(std::memory_order_acquire);

  // Full means the card has stalled for longer than the buffer covers. Drop the
  // newest sample and count it rather than blocking the sample loop — a gap in
  // the log is recoverable, a stalled altimeter is not.
  if (head - tail >= g_ringLen)
  {
    g_drops.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  Rec &r = g_ring[head % g_ringLen];
  r.tMs   = tMs;
  r.pHpa  = pressureHpa;
  r.tempC = tempC;
  r.rawM  = rawAglM;
  r.filtM = filtAglM;
  r.vsMps = vsMps;
  r.zone  = zone;
  r.phase = phase;
  r.groundP = groundPHpa;

  g_head.store(head + 1, std::memory_order_release);
}

}  // namespace logger

#endif  // LOGGER_ENABLED
