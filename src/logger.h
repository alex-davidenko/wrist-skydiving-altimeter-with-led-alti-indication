#pragma once
//
// CSV jump logger to the microSD card.
//
// Logs BOTH the raw sensor values and the derived ones. The derived columns are
// recomputable from the raw ones, so they look redundant — but they are what
// makes an offline replay trustworthy. Replay the raw trace through the same
// filter code and compare against what the device actually produced in the air:
// if they match, the tooling is sound and you can experiment freely with
// tuning; if they diverge, you have found a bug or a firmware mismatch *before*
// drawing conclusions from it. That check is impossible if only raw is stored.
//
// For the same reason the file starts with a header recording the firmware
// build and every tuning constant in play. Replaying a jump recorded before a
// tuning change, without knowing that, silently gives the wrong answer.
//
// THREADING. SD cards stall unpredictably — a card doing internal wear
// levelling can block for 100 ms or more, which would reintroduce exactly the
// loop overruns the second core just eliminated. So push() only appends to a
// lock-free ring buffer in PSRAM; a writer task drains it and does all the
// blocking I/O. At 40 Hz the buffer holds many minutes, so a pathological card
// stall costs nothing.
//

#include <Arduino.h>

namespace logger {

// Mount the card and open a new file. Safe to call with no card inserted —
// returns false and everything else then no-ops.
bool begin(float qnhHpa, float groundPHpa);

// Spawn the writer task. Call after begin().
void startTask();

// Append one sample. Called from the sample loop; never blocks on I/O.
void push(uint32_t tMs, float pressureHpa, float tempC,
          float rawAglM, float filtAglM, float vsMps,
          uint8_t zone, uint8_t phase, float groundPHpa);

// Flush, close the file and unmount the card. After this the card is safe to
// remove. Logging stays off until the next boot.
void close();

void setEnabled(bool on);
bool enabled();
bool available();

const char *filename();
uint32_t    rowsWritten();
uint32_t    rowsDropped();   // ring buffer overruns — should stay 0
uint64_t    cardSizeMb();

}  // namespace logger
