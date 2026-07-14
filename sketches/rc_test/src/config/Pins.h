// config — every application-owned pin assignment in one file (#185).
// The S.BUS pins live with infrastructure/radiolink/SbusReceiverAdapter.cpp
// (#176) — wiring detail of that adapter; docs/WIRING-GUIDE-V8.md is the
// canonical hardware reference.
//
// A0/A1 are the core's analog-pin macros; config/ may not include hardware
// headers by name, so this file reaches them through the types.h bridge
// (an outside-src include, like the sketch itself) — interim until types.h
// dissolves in Phase E.
#pragma once

#include <stdint.h>

#include "../../types.h"

const uint8_t PIN_JOY_Y  = A0;  // Throttle
const uint8_t PIN_JOY_X  = A1;  // Steering
const uint8_t PIN_ESC_L  = 9;   // Left ESC PWM (50 Hz, 1000-2000 us)
const uint8_t PIN_ESC_R  = 10;  // Right ESC PWM
const uint8_t PIN_BEEPER = 8;   // D8 — active piezo (digital HIGH = beep)
