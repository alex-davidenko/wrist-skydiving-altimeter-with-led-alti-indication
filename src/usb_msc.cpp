#include "usb_msc.h"

#include "config.h"
#include "display.h"

#if !LOGGER_ENABLED

namespace usbmsc {
bool bootRequested() { return false; }
void rebootIntoMode() {}

void runForever() {}
}  // namespace usbmsc

#else

#include <USB.h>
#include <USBMSC.h>
// The RTC watchdog, reached through the HAL. soc/rtc_wdt.h looks like the
// friendlier API but its RTC_WDT_STG_SEL_* constants are not defined for the
// S3 in this core, so it does not compile.
#include <Preferences.h>
#include <sys/time.h>
#include <time.h>

#include <hal/wdt_hal.h>
#include <soc/rtc_cntl_struct.h>
#include <driver/sdmmc_host.h>
#include <sdmmc_cmd.h>

namespace usbmsc {

namespace {

// Survives a software reset, not a power cycle — which is exactly the recovery
// property we want. Uninitialised on power-on, so it is guarded by a magic
// value rather than trusted to be zero.
RTC_NOINIT_ATTR uint32_t g_bootFlag;
constexpr uint32_t kMagic = 0xA5C0FFEEu;

USBMSC        g_msc;

// The clock has to be carried across both resets by hand.
//
// Entering drive mode is a software restart and leaving it is an RTC-domain
// reset, and that second one wipes RTC memory — which is where the wall clock
// lives. Without this, every visit to drive mode costs the time spent there
// plus however long since the last periodic save. main.cpp writes the clock
// just before rebooting in; this reads it back, lets it run, and writes it
// again on the way out, so the round trip costs nothing.
//
// Same namespace and key as main.cpp: "altimeter" / "clock".
Preferences   g_prefs;
constexpr uint32_t kPlausibleEpoch = 1735689600;   // 2025-01-01

void clockRestore()
{
  g_prefs.begin("altimeter", false);
  const uint32_t saved = g_prefs.isKey("clock") ? g_prefs.getULong("clock") : 0;
  if (saved >= kPlausibleEpoch)
  {
    const struct timeval tv = {static_cast<time_t>(saved), 0};
    settimeofday(&tv, nullptr);
  }
}

void clockSave()
{
  const time_t now = time(nullptr);
  if (now >= static_cast<time_t>(kPlausibleEpoch))
    g_prefs.putULong("clock", static_cast<uint32_t>(now));
}
sdmmc_card_t *g_card = nullptr;

int32_t onRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
  if (!g_card || offset != 0) return -1;
  const uint32_t blocks = bufsize / g_card->csd.sector_size;
  if (sdmmc_read_sectors(g_card, buffer, lba, blocks) != ESP_OK) return -1;
  return (int32_t)bufsize;
}

int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
  if (!g_card || offset != 0) return -1;
  const uint32_t blocks = bufsize / g_card->csd.sector_size;
  if (sdmmc_write_sectors(g_card, buffer, lba, blocks) != ESP_OK) return -1;
  return (int32_t)bufsize;
}

// The host's eject. Report the media gone so it stops issuing reads; the user
// then presses RST to get the altimeter back.
bool onStartStop(uint8_t, bool, bool load_eject)
{
  if (load_eject) g_msc.mediaPresent(false);
  return true;
}

// Bring the card up at the block level. No filesystem: MSC hands raw sectors
// to the host and the host owns the FAT.
bool cardBegin()
{
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.flags  = SDMMC_HOST_FLAG_4BIT;
  host.max_freq_khz = SDMMC_FREQ_DEFAULT;

  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = 4;
  slot.clk = (gpio_num_t)PIN_SD_CLK;
  slot.cmd = (gpio_num_t)PIN_SD_CMD;
  slot.d0  = (gpio_num_t)PIN_SD_D0;
  slot.d1  = (gpio_num_t)PIN_SD_D1;
  slot.d2  = (gpio_num_t)PIN_SD_D2;
  slot.d3  = (gpio_num_t)PIN_SD_D3;
  slot.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

  if (sdmmc_host_init() != ESP_OK) return false;
  if (sdmmc_host_init_slot(SDMMC_HOST_SLOT_1, &slot) != ESP_OK) return false;

  g_card = (sdmmc_card_t *)malloc(sizeof(sdmmc_card_t));
  if (!g_card) return false;
  if (sdmmc_card_init(&host, g_card) != ESP_OK)
  {
    free(g_card);
    g_card = nullptr;
    return false;
  }
  return true;
}

}  // namespace

bool bootRequested()
{
  const bool wanted = (g_bootFlag == kMagic);
  // Consume it immediately. If anything below crashes, the next boot is normal
  // rather than a loop back into a mode that just failed.
  g_bootFlag = 0;
  return wanted;
}

void rebootIntoMode()
{
  g_bootFlag = kMagic;
  Serial.println(F("Rebooting into USB drive mode. Press BOOT to return."));
  Serial.flush();
  delay(200);
  esp_restart();
}

// BOOT is the way out, not just RST.
//
// USB drive mode used to be a one-way door: runForever() was literally
// `while (true) delay(100)` and only a reset left it. That is fine on a bare
// board and wrong on a finished one — once the enclosure is closed the RST
// button is under the shell, and the device is stuck presenting a disk until
// you find something thin enough to poke it with. Which is exactly where Alex
// ended up, at a dropzone, with a jump he could not download.
//
// The boot flag is consumed before any of this runs, so a plain restart comes
// back as the altimeter.
static void waitForExit()
{
  pinMode(PIN_BUTTON, BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
  for (;;)
  {
    const bool down = digitalRead(PIN_BUTTON) == (BUTTON_ACTIVE_LOW ? LOW : HIGH);
    if (down)
    {
      // Eject first: leaving mid-write corrupts the FAT the host is managing.
      Serial.println(F("usb: BOOT pressed — returning to the altimeter."));
      Serial.flush();
      display::message("RETURNING", "");

      // Getting out of here without losing the USB port took three attempts,
      // so the reasoning is worth keeping.
      //
      // esp_restart() leaves the USB peripheral unenumerable once TinyUSB has
      // run (esp-idf#9826): the device boots and runs perfectly and simply
      // never appears on the host again. usb_persist_restart(RESTART_PERSIST)
      // did not help either — it is built to keep a CDC link alive across a
      // reboot, and this is a device-class change, MSC out and CDC in.
      //
      // Only RST recovered it, because on this board RST drives EN and
      // power-cycles the PHY. The RTC watchdog's RESET_RTC action is the one
      // software reset that reaches that far: it resets the main system AND the
      // RTC domain, where every softer reset leaves the RTC alone. That is
      // precisely the domain holding the USB PHY state.
      //
      // The cost is that RTC memory does not survive, which is fine now and
      // would not have been a month ago: the wall clock is mirrored to NVS, so
      // it comes back within one save interval instead of reverting to the
      // build date, and the drive-mode boot flag has already been consumed.
      // Save before the reset, not after: the reset below wipes RTC memory, so
      // this is the only copy that survives it.
      clockSave();

      g_msc.end();
      delay(400);

      // RWDT ticks on the RTC slow clock, nominally ~136 kHz, so this is
      // roughly half a second — long enough for the message above to be seen.
      wdt_hal_context_t rwdt = {};
      rwdt.inst = WDT_RWDT;
      rwdt.rwdt_dev = &RTCCNTL;
      wdt_hal_init(&rwdt, WDT_RWDT, 0, false);
      wdt_hal_write_protect_disable(&rwdt);
      wdt_hal_config_stage(&rwdt, WDT_STAGE0, 70000, WDT_STAGE_ACTION_RESET_RTC);
      wdt_hal_enable(&rwdt);
      wdt_hal_write_protect_enable(&rwdt);
      for (;;) { delay(10); }        // wait for it to fire
    }
    delay(50);
  }
}

void runForever()
{
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== USB DRIVE MODE ==="));

  // Pick the clock back up where the altimeter left it, and let it run.
  clockRestore();

  display::begin();

  if (!cardBegin())
  {
    Serial.println(F("usb: no SD card — press RST to return to the altimeter"));
    display::message("NO CARD", "BOOT to go back");
    waitForExit();
  }

  const uint32_t sectorSize  = g_card->csd.sector_size;
  const uint32_t sectorCount = g_card->csd.capacity;
  Serial.printf("usb: %llu MB, %lu sectors of %lu bytes\n",
                ((uint64_t)sectorCount * sectorSize) >> 20,
                (unsigned long)sectorCount, (unsigned long)sectorSize);

  g_msc.vendorID("Altimtr");
  g_msc.productID("Jump Logs");
  g_msc.productRevision("1.0");
  g_msc.onRead(onRead);
  g_msc.onWrite(onWrite);
  g_msc.onStartStop(onStartStop);
  g_msc.mediaPresent(true);
  g_msc.begin(sectorCount, sectorSize);
  USB.begin();

  display::message("EJECT FIRST", "then press BOOT");
  Serial.println(F("usb: mounted on the host. Eject there, then press RST."));

  // Nothing else runs in this mode — no sampling, no logging, no altimeter.
  // The card has exactly one owner.
  waitForExit();
}

}  // namespace usbmsc

#endif  // LOGGER_ENABLED
