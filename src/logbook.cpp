#include "logbook.h"

#include "config.h"

#if !LOGGER_ENABLED
namespace logbook {
bool scan() { return false; }
uint16_t count() { return 0; }
static Entry g_none{};
const Entry &at(uint16_t) { return g_none; }
void filename(uint16_t, char *out, size_t n) { if (n) out[0] = '\0'; }
}  // namespace logbook
#else

#include <SD_MMC.h>

namespace logbook {
namespace {

constexpr uint16_t kMax = LOGBOOK_MAX_ENTRIES;
Entry   *g_e = nullptr;
uint16_t g_n = 0;

// Pull the summary out of the tail of one file. Returns false for anything that
// does not carry one — a jump interrupted by a flat battery, or a file still
// being written — rather than listing it with zeroes.
bool readTail(const char *path, Entry &e)
{
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return false;
  const size_t sz = f.size();
  if (sz < 32) { f.close(); return false; }
  const size_t want = sz < 256 ? sz : 256;
  f.seek(sz - want);
  char buf[257];
  const size_t got = f.read(reinterpret_cast<uint8_t *>(buf), want);
  f.close();
  buf[got] = '\0';

  const char *j = strstr(buf, "# jump=");
  const char *k = strstr(buf, "# freefall_s=");
  if (!j || !k) return false;

  unsigned num = 0;
  int ex = 0, op = 0;
  char date[16] = {0};
  if (sscanf(j, "# jump=%u date=%15s exit_m=%d open_m=%d", &num, date, &ex, &op) != 4)
    return false;
  float ff = 0, can = 0, avg = 0, cl = 0;
  if (sscanf(k, "# freefall_s=%f canopy_s=%f avg_freefall_mps=%f avg_climb_mps=%f",
             &ff, &can, &avg, &cl) != 4)
    return false;

  e.number = static_cast<uint16_t>(num);
  snprintf(e.date, sizeof(e.date), "%s", date);
  e.exitM = static_cast<int16_t>(ex);
  e.openM = static_cast<int16_t>(op);
  e.freefallS = ff; e.canopyS = can;
  e.avgFreefallMps = avg; e.avgClimbMps = cl;
  return true;
}

}  // namespace

bool scan()
{
  if (!g_e)
  {
    g_e = static_cast<Entry *>(ps_malloc(sizeof(Entry) * kMax));
    if (!g_e) g_e = static_cast<Entry *>(malloc(sizeof(Entry) * kMax));
    if (!g_e) return false;
  }
  g_n = 0;

  File root = SD_MMC.open("/");
  if (!root) return false;
  for (File f = root.openNextFile(); f && g_n < kMax; f = root.openNextFile())
  {
    const char *name = f.name();
    // f.name() is sometimes bare and sometimes rooted depending on core version.
    const char *base = strrchr(name, '/');
    base = base ? base + 1 : name;
    const bool isJump = strncmp(base, "JUMP", 4) == 0 &&
                        strstr(base, ".CSV") != nullptr && !f.isDirectory();
    char path[40];
    snprintf(path, sizeof(path), "/%s", base);
    f.close();
    if (!isJump) continue;
    if (readTail(path, g_e[g_n])) g_n++;
  }
  root.close();

  // Newest first. Insertion sort: the list is small and nearly ordered already,
  // since the card hands them back in creation order.
  for (uint16_t i = 1; i < g_n; i++)
  {
    Entry key = g_e[i];
    int16_t j = static_cast<int16_t>(i) - 1;
    while (j >= 0 && g_e[j].number < key.number) { g_e[j + 1] = g_e[j]; j--; }
    g_e[j + 1] = key;
  }
  Serial.printf("logbook: %u jump%s on the card\n", g_n, g_n == 1 ? "" : "s");
  return true;
}

uint16_t count() { return g_n; }

const Entry &at(uint16_t i)
{
  static Entry none{};
  return (g_e && i < g_n) ? g_e[i] : none;
}

void filename(uint16_t i, char *out, size_t n)
{
  if (!n) return;
  if (!g_e || i >= g_n) { out[0] = '\0'; return; }
  snprintf(out, n, "/JUMP%04u.CSV", g_e[i].number);
}

}  // namespace logbook
#endif
