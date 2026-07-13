// ports — the RC input contract (#117 step 10 slice 3, #176).
// A link-time seam (#130): free functions, ONE linked implementation
// (infrastructure/radiolink/SbusReceiverAdapter.cpp). Domain types only —
// the vendor S.BUS type never crosses this port. The caller owns the
// freshness timeout and every per-channel policy.
#pragma once

#include <stdint.h>

// One received RC frame: 16 raw channel values plus the receiver's own
// failsafe/lost-frame flags.
struct RcFrame {
  int16_t channels[16];
  bool failsafe;
  bool lostFrame;
};

// Start the receiver (UART + S.BUS parser).
void rcInputInitialize();

// Poll the receiver. Returns true and fills `out` when a complete fresh
// frame arrived this pass; false (out untouched) otherwise.
bool rcInputReadFrame(RcFrame* out);
