// application — the ESC output stage (#189): [OUTPUT] moved verbatim from
// the .ino — the write clamp, and the fail-safe gate shim around
// domain/safety/OutputGate.
#pragma once

#include <stdint.h>

extern int outL, outR;
enum OutState { OUT_ACTIVE, OUT_HOLD, OUT_CUT };
extern OutState outState;
extern uint32_t outHoldMs;
extern int rampFromL;
extern int rampFromR;

void outputInit();
void outputWrite(int left, int right);
void outputUpdate(bool driveAllowed, int mixL, int mixR);
