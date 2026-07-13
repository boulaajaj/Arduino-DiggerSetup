// alerts — pattern player implementations (#183), verbatim from the .ino.
#include "PatternPlayer.h"

void oneShotPatternStart(OneShotPatternState& state, const uint16_t* sequence,
                         int length, uint32_t nowMs) {
  state.sequence = sequence;
  state.length = length;
  state.index = 0;
  state.phaseMs = nowMs;
}

bool oneShotPatternStep(OneShotPatternState& state, uint32_t nowMs) {
  bool patternOn = false;
  if (state.index >= 0 && state.index < state.length) {
    if (nowMs - state.phaseMs >= state.sequence[state.index]) {
      state.index++;
      state.phaseMs = nowMs;
    }
    patternOn = (state.index >= 0 && state.index < state.length) &&
                (state.index % 2 == 0);
  }
  return patternOn;
}

void repeatingPatternSelect(RepeatingPatternState& state, const uint16_t* sequence,
                            int length, uint32_t nowMs) {
  if (sequence != state.sequence) {
    state.sequence = sequence;
    state.length = length;
    state.index = 0;
    state.phaseMs = nowMs;
  }
}

bool repeatingPatternStep(RepeatingPatternState& state, uint32_t nowMs) {
  if (state.sequence == nullptr || state.length <= 0) return false;  // silent
  if (nowMs - state.phaseMs >= state.sequence[state.index]) {
    state.index = (state.index + 1) % state.length;
    state.phaseMs = nowMs;
  }
  return (state.index % 2 == 0);  // even index = ON phase
}
