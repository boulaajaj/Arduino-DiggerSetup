// Serial2 Hardware UART Test — Arduino Nano R4
// Tests if a second hardware UART works on A4 (TX) / A5 (RX)
// using the SCI0 channel on the RA4M1.

#include "Serial.h"

// Create second UART on A4=TX(pin18), A5=RX(pin19)
UART Serial2(18, 19);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("# === Serial2 Test ===");

  Serial2.begin(19200);

  // Check if Serial2 initialized
  if (Serial2) {
    Serial.println("# Serial2: INITIALIZED OK");
  } else {
    Serial.println("# Serial2: FAILED TO INITIALIZE");
  }

  Serial.println("# Sending poll on A4 (TX), listening on A5 (RX)...");
  Serial.println("# Run motor to see telemetry");
}

void loop() {
  static uint32_t lastPoll = 0;
  static uint32_t lastPrint = 0;
  static int totalRx = 0;

  // Poll every 50ms
  if (millis() - lastPoll >= 50) {
    lastPoll = millis();
    uint8_t poll[] = {0x9B, 0x03, 0x00, 0x00, 0x00, 0x9E};
    Serial2.write(poll, 6);
  }

  // Count received bytes
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    totalRx++;
    // Print first 32 bytes as hex
    if (totalRx <= 32) {
      char hex[4];
      snprintf(hex, sizeof(hex), "%02X ", b);
      Serial.print(hex);
      if (totalRx % 16 == 0) Serial.println();
    }
  }

  // Report every 2 seconds
  if (millis() - lastPrint >= 2000) {
    lastPrint = millis();
    Serial.print("# Total RX bytes: ");
    Serial.println(totalRx);
  }
}
