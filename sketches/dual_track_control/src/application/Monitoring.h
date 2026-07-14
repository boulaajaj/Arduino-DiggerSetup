// application — the observers (#189): the [WIFI] remnant + [DEBUG] +
// SystemSnapshot construction, moved verbatim from the .ino. Monitoring
// only — no control input over Wi-Fi, ever (permanent rule).
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "SystemSnapshot.h"

extern uint32_t wifiSeq;
extern uint32_t wifiDbgPrev;
extern uint32_t prevPrint;

int wifiMode();
void wifiInit();
void wifiDebug(uint32_t nowUs);
SystemSnapshot buildSystemSnapshot(uint32_t nowMs);
int buildTelemJson(char *body, size_t cap);
void wifiUpdate();
void debugInit();
void debugPrint(uint32_t now);
