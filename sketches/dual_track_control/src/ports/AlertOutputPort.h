// ports — the alert (piezo) output contract (#117 step 10 slice 2, #174).
// A link-time seam (#130): free functions, ONE linked implementation
// (infrastructure/arduino/PiezoAdapter.cpp). The horn/pattern/alarm
// priority policy stays with the caller — this port only sets the level.
#pragma once

// Configure the piezo pin as an output, silent.
void alertOutputInitialize(int pin);

// Drive the piezo: true = sounding, false = silent.
void alertOutputSet(bool on);
