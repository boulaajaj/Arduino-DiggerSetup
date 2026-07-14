// config — every application-owned pin assignment in one file (#185).
// The S.BUS pins live with infrastructure/radiolink/SbusReceiverAdapter.cpp
// (#176) — wiring detail of that adapter; docs/WIRING-GUIDE-V8.md is the
// canonical hardware reference.
//
// Hardware-free by design (config/ may not depend on the core header, even
// transitively). The analog joystick pins need the core's A0/A1 macros, so
// they live with their processing home, application/OperatorInput.h (which
// already carries the interim types.h bridge); the S.BUS pins live with
// their adapter (#176).
#pragma once

#include <stdint.h>

const uint8_t PIN_ESC_L  = 9;   // Left ESC PWM (50 Hz, 1000-2000 us)
const uint8_t PIN_ESC_R  = 10;  // Right ESC PWM
const uint8_t PIN_BEEPER = 8;   // D8 — active piezo (digital HIGH = beep)
