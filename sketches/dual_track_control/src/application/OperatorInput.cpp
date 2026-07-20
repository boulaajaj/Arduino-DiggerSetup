// application — operator-input implementations (#189), verbatim from the
// .ino (each section keeps its original commentary).
#include "OperatorInput.h"

#include "FirmwareState.h"
#include "RangeMath.h"
#include "../config/InputConfig.h"
#include "../config/DriveConfig.h"
#include "../config/Pins.h"
#include "../domain/drive/CurvatureDrive.h"
#include "../domain/drive/GearPolicy.h"
#include "../domain/drive/CommandMixer.h"
#include "../domain/operator_input/DeadbandPolicy.h"
#include "../domain/operator_input/ExpoCurve.h"
#include "../ports/JoystickPort.h"

// ═══════════════════════════════════════════════════════════════
// [DRIVE] — curvatureDrive: proven FRC algorithm (WPILib)
// ═══════════════════════════════════════════════════════════════

// The curvature mix is extracted to src/domain/drive/CurvatureDrive.* (#117
// step 6, #164) — the algorithm and its commentary (#72/#86/#96/#114) moved
// with it. This delegate bundles the fixed blend/taper tunables; every
// per-pass dependency, including the gear-selected pivot cap, arrives as a
// parameter (#119 — the composition root owns the gear read).
WheelSpeeds curvatureDrive(float xSpeed, float zRotation, float gearScale,
                           float pivotCap) {
  TrackCommand command = curvatureDriveStep(
      xSpeed, zRotation, gearScale,
      CurvatureParameters{pivotCap, PIVOT_THROTTLE_TAPER, PIVOT_BLEND_START,
                          PIVOT_BLEND_END, TURN_TRACK_CAP});
  return {command.left, command.right};
}

ServoOutput wheelSpeedsToServo(WheelSpeeds ws) {
  ServoOutput out;
  out.left  = SVC + (int)(ws.left  * SOFT_RANGE);
  out.right = SVC + (int)(ws.right * SOFT_RANGE);
  out.left  = clampInt(out.left,  SVMIN, SVMAX);
  out.right = clampInt(out.right, SVMIN, SVMAX);
  return out;
}


// ═══════════════════════════════════════════════════════════════
// [RC] — S.BUS input on sbusUart / SCI0 (D12 RX, NPN inverter)
// ═══════════════════════════════════════════════════════════════

RcFrame rcFrame = {};                    // latest received frame (domain type)
bool sbusValid = false;
uint32_t sbusLastFrame = 0;

int sbusToServo(int raw) {
  return mapRange(clampInt(raw, SBUS_MIN, SBUS_MAX), SBUS_MIN, SBUS_MAX, SVMIN, SVMAX);
}

// Deadband math extracted to src/domain/operator_input/DeadbandPolicy.*
// (#117 step 5, #162); this delegate keeps every call site unchanged.
int rcDeadband(int pw) {
  return centerSnapDeadband(pw, SVC, RC_DEADBAND);
}

int rcThrottle() { return sbusValid ? rcDeadband(sbusToServo(rcFrame.channels[SBUS_CH_THR]))   : SVC; }
int rcSteering() { return sbusValid ? rcDeadband(sbusToServo(rcFrame.channels[SBUS_CH_STEER])) : SVC; }
int rcOverride() { return sbusValid ? sbusToServo(rcFrame.channels[SBUS_CH_OVR]) : SVMIN; }

// RC axis command (xSpeed/zRotation), post-gain and reverse-limit — pre-mix,
// pre-curvatureDrive. The mix combines RC + joystick at this axis level (#90),
// then curvatureDrive runs once on the combined command.
DriveCommand rcCommand() {
  float xSpeed    = (float)(rcThrottle() - SVC) / SOFT_RANGE;
  float zRotation = (float)(rcSteering() - SVC) / SOFT_RANGE;
  // Apply tunable input gains, then clamp: RC keeps the gear's full forward
  // authority (1.0); reverse is the per-gear reverseCap() (55% Eco/Normal, 65%
  // Boost) (#87/#113).
  xSpeed    = clampFloat(xSpeed * RC_THROTTLE_GAIN, -outCapToX(reverseCap()), 1.0f);
  zRotation = clampFloat(zRotation * RC_STEERING_GAIN, -1.0f, 1.0f);
  return {xSpeed, zRotation};
}


// ═══════════════════════════════════════════════════════════════
// [GEAR] — RC CH4 → average-speed cap (Eco 65% / Normal 80% / Boost 100%)
// ═══════════════════════════════════════════════════════════════
//
// updateGear() is defined here (after [RC]) so it can read sbusValid /
// rcFrame directly. The gearScale and currentGear globals it writes
// are declared up in [CONFIG] so the drive functions and curvatureDrive
// can read them for the Eco-only conditional caps.

// The decision tree is extracted to src/domain/drive/GearPolicy.* (#117
// step 7, #166); this delegate owns the boundary reads — sbus CH4 and the
// Eco forces (battery Eco lock #65 latched OR thermal Eco #111
// non-latching, which clears once the motor cools below TEMP_ECO_OFF_C) —
// and mirrors the gearScale/currentGear globals every caller reads.
void updateGear() {
  // Built once — the values are [CONFIG] constants (loop() calls this every
  // pass, so no per-pass aggregate temporary).
  static const GearPolicyParameters GEAR_POLICY_PARAMETERS{
      GEAR_LOW_SCALE, GEAR_MID_SCALE, GEAR_HIGH_SCALE, OVR_LO, OVR_HI};
  GearSelection selection = gearPolicySelect(
      sbusValid ? sbusToServo(rcFrame.channels[SBUS_CH_GEAR]) : 0, sbusValid,
      ecoLockLatched || tempEcoActive, GEAR_POLICY_PARAMETERS);
  gearScale = selection.scale;
  currentGear = (Gear)selection.level;
}


// ═══════════════════════════════════════════════════════════════
// [JOYSTICK] — ADC, deadband, expo curve
// ═══════════════════════════════════════════════════════════════

// expoCurve() now lives in src/domain/operator_input/ExpoCurve.* (#117
// step 5, #162) — same name and signature, so every call site (and the
// characterization tests) resolves to the domain implementation directly.

int joyDeadband(int adc) {
  return centerSnapDeadband(adc, ADC_CENTER, JOY_DEADBAND);
}

JoystickState cachedJoy = {ADC_CENTER, ADC_CENTER, 0.0f, 0.0f};
DriveCommand  cachedJoyCmd = {0.0f, 0.0f};  // joystick axis command, post-gain/cap/rev-limit (#90)
uint32_t      lastAdcTime = 0;

void updateJoystick(uint32_t now) {
  if ((now - lastAdcTime) < ADC_INTERVAL) return;
  lastAdcTime = now;

  // The ADC conditioning sequence (discard-read, 100 µs settle, real read)
  // lives in infrastructure/arduino/AdcJoystickAdapter.cpp behind
  // ports/JoystickPort.h (#117 step 10 slice 2, #174).
  joystickReadAxes(PIN_JOY_Y, PIN_JOY_X, &cachedJoy.rawY, &cachedJoy.rawX);

  float normY = clampFloat((float)(joyDeadband(cachedJoy.rawY) - ADC_CENTER) / ADC_CENTER, -1.0f, 1.0f);
  float normX = clampFloat((float)(joyDeadband(cachedJoy.rawX) - ADC_CENTER) / ADC_CENTER, -1.0f, 1.0f);
  float signY = (normY >= 0) ? 1.0f : -1.0f;
  float signX = (normX >= 0) ? 1.0f : -1.0f;
  cachedJoy.xSpeed    = signY * expoCurve(normY, EXPO_THROTTLE_LINEAR, EXPO_THROTTLE_CUBIC);
  cachedJoy.zRotation = JOY_STEER_DIR * signX * expoCurve(normX, EXPO_STEER_LINEAR, EXPO_STEER_CUBIC);

  float xSpeed = cachedJoy.xSpeed * JOY_THROTTLE_GAIN;
  float zRotation = cachedJoy.zRotation;
  // Clamp throttle: per-gear joystick FORWARD cap, per-gear reverseCap() backward —
  // both as track-output fractions converted to the xSpeed domain (#90/#87/#113).
  // Throttle axis only; steering/pivot untouched. RC unaffected.
  float fwdCap = (currentGear == GEAR_LOW)  ? JOY_CAP_ECO
               : (currentGear == GEAR_HIGH) ? JOY_CAP_BOOST
                                            : JOY_CAP_NORMAL;
  xSpeed = clampFloat(xSpeed, -outCapToX(reverseCap()), outCapToX(fwdCap));
  cachedJoyCmd = {xSpeed, zRotation};
}


// ═══════════════════════════════════════════════════════════════
// [MIXER] — Override switch selects authority
// ═══════════════════════════════════════════════════════════════

// maxOppose() (#90) and the mode selection are extracted to
// src/domain/drive/CommandMixer.* (#117 step 7, #166) — maxOppose keeps its
// name and signature, so call sites and tests resolve to the domain
// directly. This delegate converts DriveCommand ↔ AxisCommand
// (layout-identical) and passes the [CONFIG] thresholds. curvatureDrive
// runs ONCE on the result (in loop), so a single operator always gets full
// range — the old code averaged the final wheel PWMs and halved it.
DriveCommand mixCommands(DriveCommand rc, int ovr, DriveCommand joy) {
  AxisCommand mixed = mixAxisCommands(AxisCommand{rc.xSpeed, rc.zRotation},
                                      AxisCommand{joy.xSpeed, joy.zRotation},
                                      ovr, OVR_LO, OVR_HI);
  return {mixed.xSpeed, mixed.zRotation};
}


