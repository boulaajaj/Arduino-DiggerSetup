// application — operator-input processing (#189): the [DRIVE]/[RC]/[GEAR]/
// [JOYSTICK]/[MIXER] shims moved verbatim from the .ino. Wraps the domain
// pipeline (deadband → expo → mix → curvature) around the input ports.
#pragma once

#include <stdint.h>

#include "../../types.h"
#include "../ports/RcInputPort.h"

extern RcFrame rcFrame;
extern bool sbusValid;
extern uint32_t sbusLastFrame;
const uint32_t SBUS_TIMEOUT = 100000UL;  // 100ms

extern JoystickState cachedJoy;
extern DriveCommand  cachedJoyCmd;
extern uint32_t      lastAdcTime;
const uint32_t ADC_INTERVAL = 10000UL;  // 10 ms = 100 Hz

WheelSpeeds curvatureDrive(float xSpeed, float zRotation, float gearScale);
ServoOutput wheelSpeedsToServo(WheelSpeeds ws);
int sbusToServo(int raw);
int rcDeadband(int pw);
int rcThrottle();
int rcSteering();
int rcOverride();
DriveCommand rcCommand();
void updateGear();
int joyDeadband(int adc);
void updateJoystick(uint32_t now);
DriveCommand mixCommands(DriveCommand rc, int ovr, DriveCommand joy);
