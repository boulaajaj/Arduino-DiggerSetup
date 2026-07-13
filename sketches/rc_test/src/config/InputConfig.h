// config — operator-input tunables: RC channel map, ADC, deadbands,
// override thresholds, expo weights, joystick gains (#185).
#pragma once

#include <stdint.h>

// S.BUS channel mapping (0-indexed). Confirmed by live capture while
// the operator moved each control independently on the RC6GS V3:
// trigger → ch 1, wheel → ch 0.
const uint8_t SBUS_CH_THR   = 1;  // trigger → throttle (forward/back)
const uint8_t SBUS_CH_STEER = 0;  // wheel   → steering (left/right)
const uint8_t SBUS_CH_GEAR  = 3;  // CH4 = gear selector (3-pos switch)
const uint8_t SBUS_CH_OVR   = 4;  // CH5 = override switch (3-pos switch)
const uint8_t SBUS_CH_HORN  = 6;  // CH7 = SWD button → horn (beep at +100%)

// S.BUS value range (raw 172-1811, center ~992)
const int SBUS_MIN = 172;
const int SBUS_MAX = 1811;
const int HORN_ON_RAW = 1400;  // SWD raw above this = horn ON (toward +100% ~1811)

// ADC
const int ADC_CENTER = 8192;  // 14-bit midpoint

// Deadbands
const int RC_DEADBAND  = 50;   // RC mapped pulse (us)
const int JOY_DEADBAND = 480;  // Joystick ADC (~5.9% of travel)

// Override switch thresholds (mapped to PWM-equivalent)
const int OVR_LO = 1400;  // Below → RC only
const int OVR_HI = 1600;  // Above → 50/50 blend (RC + joystick)

// Expo curve blend weights — output = LINEAR*|x| + CUBIC*|x|^3.
// Throttle keeps the smoother (more cubic) curve so launch feel is gentle.
// Steering uses a more linear curve so partial joystick deflection
// produces real turn authority — operator feedback was that the joystick
// pivot felt underpowered before reaching full lock.
const float EXPO_THROTTLE_LINEAR = 0.4f;
const float EXPO_THROTTLE_CUBIC  = 0.6f;
const float EXPO_STEER_LINEAR    = 0.7f;
const float EXPO_STEER_CUBIC     = 0.3f;

// Joystick steering polarity: -1.0f to flip left/right (set after the
// operator-side cable was rewired and right-stick produced a left turn).
const float JOY_STEER_DIR = -1.0f;

// Joystick throttle gain (#90) — the Genie stick under-ranges: full physical
// deflection only reaches ~0.75 xSpeed, so the rider couldn't hit the per-gear
// caps below. Lift it so full travel can reach the cap. Joystick-only; RC
// unaffected. Tunable.
const float JOY_THROTTLE_GAIN = 1.40f;
