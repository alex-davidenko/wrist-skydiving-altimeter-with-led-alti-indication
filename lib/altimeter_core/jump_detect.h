#pragma once
#include <stdint.h>

// Decides when a jump is happening, so the logger records one file per jump
// instead of one per power-up.
//
// Four flights produced 147 MB across 168 files, nearly all of it a device
// sitting on a desk or riding home in a car. One file per jump is what a
// logbook needs, and it is what makes a jump addressable later.
//
// ARMING IS DELIBERATELY GENEROUS. A missed jump is unrecoverable; a spurious
// file costs a few hundred kB. So recording starts on a latched climb above
// startAltM *or* on freefall speed alone — the second is the fallback for a
// climb that was never detected, which is exactly the failure the velocity bug
// produced for a whole flight.
//
// STOPPING IS DELIBERATELY SLOW. It requires being back low AND still for
// settleMs, because a canopy ride passes through low altitudes with plenty of
// vertical speed left, and closing early truncates the landing.
class JumpDetector
{
 public:
  struct Config
  {
    float    startAltM;      // climb above this arms recording
    float    stopAltM;       // and back below this ends it
    float    freefallMps;    // descent rate that arms regardless of climb
    float    stillMps;       // vertical speed that counts as landed
    uint32_t settleMs;       // how long low and still before closing
    uint32_t maxMs;          // give up if freefall never arrives
  };

  void begin(const Config &c);
  void reset();

  // Feed every sample. Returns true while a jump should be recording.
  bool update(float altM, float vspeedMps, uint8_t mode, uint32_t nowMs);

  bool recording() const { return _rec; }

  // Manual override, for when detection is not trusted yet. forceStart() begins
  // a recording that the normal stop rule will still end; forceStop() ends one
  // immediately. Kept inside the detector rather than beside it so there is one
  // answer to "are we recording", not two that can disagree.
  void forceStart(float altM, uint32_t nowMs);
  void forceStop();
  // True for exactly one call, on the sample that closes a jump — the moment
  // to write the summary and roll to the next file.
  bool justFinished() const { return _finished; }
  // True for one call when a recording is abandoned: it never contained
  // freefall, so it was not a jump. Driving home from the DZ climbs above the
  // arming altitude and never returns below the stopping one, which on a real
  // log armed a second file that recorded for over an hour. Delete these.
  bool justAborted() const { return _aborted; }
  bool sawFreefall() const { return _sawFreefall; }
  float peakAltM() const { return _peak; }

 private:
  Config   _cfg{50.0f, 10.0f, 20.0f, 1.0f, 15000, 2700000};
  bool     _rec = false, _finished = false, _aborted = false;
  bool     _sawDescent = false, _sawFreefall = false;
  uint32_t _startedMs = 0;
  float    _peak = 0.0f;
  uint32_t _lowSince = 0;
  bool     _low = false;
};
