// domain/safety — the drive ALLOW/DENY supervisor (#117 step 8, #168).
// Extracted from the inline chain in dual_track_control.ino loop() (#88/#65/#111):
//   driveAllowed = rcValid && batteryReady && !cutoffLatched && !thermalCut
// with batteryReady = batteryConfirmed || bootGraceElapsed (the fail-open
// boot gate: a low pack cannot drive after a watchdog reset wipes the RAM
// latch, yet a dead X.BUS can never permanently disable driving).
// The early-return shape below is provably identical to the && chain for
// every input combination; the returned reason mirrors the chain's order.
#pragma once

#include "SafetyTypes.h"

SafetyDecision safetySupervisorDecide(const SafetyInputs& inputs);
