// alerts — non-blocking on/off pattern sequencing (#183). Moved VERBATIM
// from dual_track_control.ino [BEEPER] (one-shot) and the [ALERT] playback tail
// (repeating). Pure logic: time is a parameter, no hardware.
#pragma once

#include <stdint.h>

#include "AlertTypes.h"

// Queue a one-shot pattern (durations in ms, starting with ON).
void oneShotPatternStart(OneShotPatternState& state, const uint16_t* sequence,
                         int length, uint32_t nowMs);

// Advance the one-shot pattern; returns true while an ON phase is sounding
// (even index). Idle/finished patterns return false.
bool oneShotPatternStep(OneShotPatternState& state, uint32_t nowMs);

// (Re)start repeating playback when the selected alarm changes; keeping the
// same sequence leaves playback untouched.
void repeatingPatternSelect(RepeatingPatternState& state, const uint16_t* sequence,
                            int length, uint32_t nowMs);

// Advance the repeating pattern (loops, unlike the one-shot); returns true
// while an ON phase is sounding (even index).
bool repeatingPatternStep(RepeatingPatternState& state, uint32_t nowMs);
