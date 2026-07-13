// telemetry — the ONE implementation of the debug CSV line (#132). The
// telemetry columns use the shared TelemetryScaling helpers — the same
// lroundf math debugPrint() previously inlined (the #121 duplication).
#include "CsvEncoder.h"

#include <stdio.h>

#include "../application/SystemSnapshot.h"
#include "TelemetryScaling.h"

int encodeDebugCsv(char *line, size_t capacity, const SystemSnapshot &snapshot) {
  return snprintf(line, capacity,
                  "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                  snapshot.rcThrottleUs, snapshot.rcSteeringUs,
                  snapshot.rcGearUs, snapshot.rcOverrideUs,
                  snapshot.joystickRawY, snapshot.joystickRawX,
                  snapshot.outLeftUs, snapshot.outRightUs, snapshot.gear,
                  snapshot.rcFailsafe, snapshot.rcLostFrame,
                  scaleToDeciInteger(snapshot.esc[0].voltage),
                  scaleToDeciInteger(snapshot.esc[0].busCurrentA),
                  snapshot.esc[0].rpmHz,
                  roundToWholeInteger(snapshot.esc[0].escTempC),
                  roundToWholeInteger(snapshot.esc[0].motorTempC),
                  snapshot.esc[0].valid ? 1 : 0,
                  scaleToDeciInteger(snapshot.esc[1].voltage),
                  scaleToDeciInteger(snapshot.esc[1].busCurrentA),
                  snapshot.esc[1].rpmHz,
                  roundToWholeInteger(snapshot.esc[1].escTempC),
                  roundToWholeInteger(snapshot.esc[1].motorTempC),
                  snapshot.esc[1].valid ? 1 : 0);
}
