#pragma once
//
// Reads the jump summaries off the card so they can be browsed on the device.
//
// Each JUMPnnnn.CSV carries its summary in the LAST two lines, not the first,
// because none of it is known when the file is opened. That turns out to be the
// useful arrangement anyway: listing a jump means seeking to the end and reading
// a couple of hundred bytes, so a logbook of two hundred jumps costs a few tens
// of kB of reads rather than the gigabyte the rows would.
//
#include <Arduino.h>

namespace logbook {

struct Entry
{
  uint16_t number;
  char     date[11];        // YYYY-MM-DD
  int16_t  exitM, openM;
  float    freefallS, canopyS, avgFreefallMps, avgClimbMps;
};

// Scan the card. Newest first. Safe to call repeatedly; re-reads from scratch.
bool scan();
uint16_t count();
const Entry &at(uint16_t i);
// Which file an entry came from, for replay later.
void filename(uint16_t i, char *out, size_t n);

}  // namespace logbook
