// application — THE COMPOSITION-ROOT MARKER (#189, #150). This header's
// existence activates the check_ino fitness rule: the .ino may include
// only <Arduino.h> and this file. It re-exports the whole application so
// the sketch (and the test harness, through the sketch) sees every layer.
#pragma once

#include "../config/BuildInfo.h"
#include "../config/Pins.h"
#include "../config/InputConfig.h"
#include "../config/DriveConfig.h"
#include "../config/BatteryConfig.h"
#include "../config/ThermalConfig.h"
#include "../config/AlertConfig.h"
#include "../config/TelemetryConfig.h"
#include "../config/WifiConfig.h"
#include "../config/SafetyConfig.h"
#include "../domain/battery/VoltagePlausibility.h"
#include "../domain/battery/BatteryLadder.h"
#include "../domain/thermal/ThermalHysteresis.h"
#include "../domain/thermal/ThermalDerating.h"
#include "../domain/operator_input/ExpoCurve.h"
#include "../domain/operator_input/DeadbandPolicy.h"
#include "../domain/drive/CurvatureDrive.h"
#include "../domain/drive/GearPolicy.h"
#include "../domain/drive/CommandMixer.h"
#include "../domain/safety/SafetySupervisor.h"
#include "../domain/safety/OutputGate.h"
#include "../alerts/PatternPlayer.h"
#include "../alerts/AlertPolicy.h"
#include "../telemetry/TelemetryScaling.h"
#include "../telemetry/JsonEncoder.h"
#include "../telemetry/CsvEncoder.h"
#include "../ports/EscOutputPort.h"
#include "../ports/JoystickPort.h"
#include "../ports/AlertOutputPort.h"
#include "../ports/RcInputPort.h"
#include "../ports/TelemetrySource.h"
#include "../ports/DashboardServicePort.h"
#include "../ports/TelemetryFrameSource.h"
#include "../ports/ClockPort.h"
#include "../ports/WatchdogPort.h"
#include "../ports/DebugConsolePort.h"
#include "RangeMath.h"
#include "SystemSnapshot.h"
#include "FirmwareState.h"
#include "OperatorInput.h"
#include "MotorOutput.h"
#include "SafetyControl.h"
#include "AlertControl.h"
#include "Monitoring.h"

// The control cycle (bodies verbatim from the sketch's setup()/loop()).
void firmwareSetup();
void firmwareLoop();
