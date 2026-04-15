// S.BUS Signal Test — Arduino Nano R4
// Tests S.BUS reception from R7FG CH7 via NPN inverter on D0 (Serial1 RX).
// Prints all 6 channels + failsafe + frame-lost flags at 10Hz.
//
// Wiring:
//   R7FG CH7 signal ──[1K]──► 2N2222 BASE
//                              2N2222 EMITTER ──► GND
//   5V ──[10K]──┬──► 2N2222 COLLECTOR
//               └──► D0 (Serial1 RX)
//
// R7FG must be in Mode 2 (blue LED) — press ID SET once from Mode 1.

#include "sbus.h"

bfs::SbusRx sbus(&Serial1);

uint32_t lastPrint = 0;
uint32_t frameCount = 0;
uint32_t startTime = 0;
bool gotFirstFrame = false;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("# === S.BUS Test ===");
  Serial.println("# Waiting for S.BUS frames on D0 (Serial1 RX)...");
  Serial.println("# If no data appears, check:");
  Serial.println("#   1. R7FG in Mode 2 (blue LED)");
  Serial.println("#   2. Inverter wired correctly (1K to base, emitter to GND, collector to D0 + 10K to 5V)");
  Serial.println("#   3. CH7 signal wire connected");
  Serial.println("# CSV: CH1,CH2,CH3,CH4,CH5,CH6,Failsafe,Lost,Hz");

  sbus.Begin();
  startTime = millis();
}

void loop() {
  if (sbus.Read()) {
    frameCount++;
    bfs::SbusData data = sbus.data();

    if (!gotFirstFrame) {
      gotFirstFrame = true;
      Serial.println("# *** S.BUS SIGNAL DETECTED ***");
    }

    uint32_t now = millis();
    if (now - lastPrint >= 100) {  // 10Hz output
      lastPrint = now;

      // Calculate frame rate
      float elapsed = (now - startTime) / 1000.0f;
      float hz = (elapsed > 0) ? frameCount / elapsed : 0;

      char buf[120];
      snprintf(buf, sizeof(buf), "%d,%d,%d,%d,%d,%d,%d,%d,%.0f",
              data.ch[0], data.ch[1], data.ch[2],
              data.ch[3], data.ch[4], data.ch[5],
              data.failsafe, data.lost_frame,
              (double)hz);
      Serial.println(buf);
    }
  }

  // Timeout warning
  if (!gotFirstFrame && millis() - startTime > 5000) {
    Serial.println("# WARNING: No S.BUS frames received after 5s");
    Serial.println("# Check wiring and receiver mode");
    startTime = millis();  // Reset to warn again in 5s
  }
}
