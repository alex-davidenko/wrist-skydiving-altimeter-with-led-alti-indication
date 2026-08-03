#include "logger.h"

#include "config.h"
#include "zones.h"

#if !LOGGER_ENABLED

namespace logger {
bool begin(float, float) { return false; }
void startTask() {}
void push(uint32_t, float, float, float, float, float, uint8_t, uint8_t) {}
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
  g_file.printf("# build=%s %s\n", __DATE__, __TIME__);
  g_file.printf("t_ms,p_hpa,temp_c,alt_raw_m,alt_filt_m,vs_mps,zone,phase\n");
  g_file.flush();
}

// Pick the next unused LOGnnnn.CSV so a power cycle never overwrites a jump.
bool openNewFile()
{
  for (uint16_t i = 1; i < 10000; i++)
  {
    snprintf(g_name, sizeof(g_name), "/LOG%04u.CSV", i);
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
    uint32_t tail = g_tail.load(std::memory_order_relaxed);
    const uint32_t head = g_head.load(std::memory_order_acquire);

    while (tail != head)
    {
      const Rec &r = g_ring[tail % g_ringLen];
      const int n = snprintf(line, sizeof(line),
                             "%lu,%.3f,%.2f,%.3f,%.3f,%.2f,%u,%u\n",
                             (unsigned long)r.tMs, (double)r.pHpa, (double)r.tempC,
                             (double)r.rawM, (double)r.filtM, (double)r.vsMps,
                             r.zone, r.phase);
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

  if (!openNewFile())
  {
    Serial.println(F("logger: could not create a log file"));
    SD_MMC.end();
    return false;
  }

  g_ok = true;
  Serial.printf("logger: %s on %llu MB card, ring %lu rows (%lu KB)\n",
                g_name, (unsigned long long)g_cardMb,
                (unsigned long)g_ringLen,
                (unsigned long)(sizeof(Rec) * g_ringLen / 1024));
  return true;
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
          uint8_t zone, uint8_t phase)
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

  g_head.store(head + 1, std::memory_order_release);
}

}  // namespace logger

#endif  // LOGGER_ENABLED
