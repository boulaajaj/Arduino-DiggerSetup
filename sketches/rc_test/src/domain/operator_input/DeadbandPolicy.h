// domain/operator_input — center-snap deadband (#117 step 5, #162).
// The shared math of rc_test.ino's rcDeadband() (servo µs) and
// joyDeadband() (ADC counts) — identical integer logic, parameterized.
// Pure domain: center and width arrive as parameters ([CONFIG] owns them).
#pragma once

// Snaps value to center while within ±width of it (inclusive); passes the
// value through untouched otherwise.
int centerSnapDeadband(int value, int center, int width);
