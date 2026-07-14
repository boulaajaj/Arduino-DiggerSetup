// application — the staged battery + thermal protection shims (#189):
// [SAFETY] moved verbatim from the .ino (#65/#111). MOTOR-AFFECTING —
// the ladders' latches live in FirmwareState; the domain steps are pure.
#pragma once

#include <stdint.h>

extern uint32_t cutoffStartMs;
extern uint32_t ecoLockStartMs;
extern uint32_t tempWarnSinceMs;
extern uint32_t tempEcoSinceMs;
extern uint32_t tempCutSinceMs;

bool worstPackVoltage(float* worst);
void batteryEcoLockUpdate();
void batteryCutoffUpdate();
bool hottestMotorTemp(float* hot);
void tempStageUpdate(bool* state, uint32_t* since, float temp,
                     float onC, float offC, uint32_t nowMs);
void thermalUpdate();
