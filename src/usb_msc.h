#pragma once
//
// Expose the microSD card to a host computer over the same USB-C, so logs can
// be read without pulling the card.
//
// This is a BOOT MODE, not a runtime toggle, and deliberately so. The firmware
// and the host cannot both own a FAT filesystem — two writers corrupt it — so
// entry means: close the log, set a flag, reboot. On the next boot the flag is
// consumed (cleared before anything else can fail) and the device comes up as
// a plain USB drive with the altimeter not running at all.
//
// The flag lives in RTC_NOINIT memory, which survives a software reset but not
// a power cycle. That gives the recovery property for free: **any reset returns
// to normal operation**. There is no state a bad USB enumeration can leave the
// device stuck in that the RST button does not clear.
//
// In this mode the card is driven at the block level through the ESP-IDF sdmmc
// API rather than through SD_MMC. MSC is a block protocol — the host supplies
// the filesystem — so mounting FAT here would be pointless and would risk two
// owners again.
//

#include <Arduino.h>

namespace usbmsc {

// True if the previous boot asked for USB drive mode. Consumes the request, so
// asking twice returns false and a crash cannot trap the device in this mode.
bool bootRequested();

// Set the flag and restart into USB drive mode. Does not return.
void rebootIntoMode();

// Run as a USB drive. Only call when bootRequested() returned true; never
// returns while the mode is active.
void runForever();

}  // namespace usbmsc
