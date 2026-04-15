// X.BUS Focused Poll — Hobbywing V4 at 19200 baud
// Locked to the only combination that got a response.
// Sends HW V4 telemetry request every 200ms, dumps all response bytes.
//
// Wiring:
//   ESC X.BUS yellow ──┬──[1K]──► NPN base (RX inverter on D2)
//                      └──[1K]──► D3 (TX direct)
//   ESC X.BUS brown ──► GND
//   ESC X.BUS red ──► NOT CONNECTED

#include <SoftwareSerial.h>

const uint8_t RX_PIN = 2;
const uint8_t TX_PIN = 3;

SoftwareSerial xbus(RX_PIN, TX_PIN);

// Hobbywing V4 telemetry request (from ArduPilot source)
const uint8_t HW_V4_REQ[] = {0x9B, 0x03, 0x00, 0x00, 0x00, 0x9E};
// Hobbywing V5 telemetry request
const uint8_t HW_V5_REQ[] = {0x9B, 0x03, 0x00, 0x60, 0x00, 0xFE};

uint32_t lastPoll = 0;
uint32_t totalRx = 0;
uint32_t totalPolls = 0;
uint32_t pollInterval = 200;  // ms between polls
bool useV5 = false;  // Alternate V4/V5 requests
uint8_t rxBuf[32];
int rxIdx = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("# === X.BUS Focused Poll — 19200 baud ===");
  Serial.println("# Sending HW V4/V5 requests every 200ms");
  Serial.println("# Try running the motor while this is active!");
  Serial.println("#");

  xbus.begin(19200);
}

void loop() {
  // Collect any response bytes
  while (xbus.available()) {
    uint8_t b = xbus.read();
    if (rxIdx < 32) rxBuf[rxIdx++] = b;
    totalRx++;
  }

  uint32_t now = millis();

  // Print response if we got any, then send next poll
  if (now - lastPoll >= pollInterval) {
    // Print any received bytes from last poll
    if (rxIdx > 0) {
      Serial.print("# RX (");
      Serial.print(rxIdx);
      Serial.print("b): ");
      for (int i = 0; i < rxIdx; i++) {
        char hex[4];
        snprintf(hex, sizeof(hex), "%02X ", rxBuf[i]);
        Serial.print(hex);
      }
      Serial.println();
      rxIdx = 0;
    }

    // Send next poll
    lastPoll = now;
    totalPolls++;

    if (useV5) {
      xbus.write(HW_V5_REQ, sizeof(HW_V5_REQ));
    } else {
      xbus.write(HW_V4_REQ, sizeof(HW_V4_REQ));
    }

    // Print status every 5 seconds
    if (totalPolls % 25 == 0) {
      Serial.print("# Status: ");
      Serial.print(totalPolls);
      Serial.print(" polls, ");
      Serial.print(totalRx);
      Serial.print(" bytes received, using ");
      Serial.println(useV5 ? "V5" : "V4");
    }

    // Switch between V4 and V5 every 50 polls (10 seconds)
    if (totalPolls % 50 == 0) {
      useV5 = !useV5;
      Serial.print("# Switching to ");
      Serial.println(useV5 ? "V5 requests" : "V4 requests");
    }
  }
}
