// application — telemetry/beeper/alert shims (#189): [TELEMETRY]/[BEEPER]/
// [ALERT] moved verbatim from the .ino. The sketch-owned telemetry array
// lives here (every consumer reads it); policy/sequencing live in
// src/alerts/, hardware behind the alert output port.
#pragma once

#include <stdint.h>

#include "../ports/TelemetrySource.h"

const uint8_t NUM_ESCS = 2;
extern EscTelem telem[NUM_ESCS];

extern bool hornActive;
extern bool alarmOutputOn;
extern const uint16_t* beepSeq;
extern int beepLen, beepIdx;
extern uint32_t beepPhaseMs;

extern uint32_t rcOffSinceMs;
extern uint32_t lowVStartMs;
extern bool     lowVoltLatched;
extern uint32_t alertBootMs;
extern const uint16_t* alarmSeq;
extern int alarmLen, alarmIdx;
extern uint32_t alarmPhaseMs;

void telemUpdate();
void beeperInit();
void beepStart(const uint16_t *seq, int len);
void beeperUpdate();
void alertInit();
void alertUpdate(bool rcOn);
