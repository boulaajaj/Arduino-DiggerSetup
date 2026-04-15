// X.BUS Inverted UART Test — Arduino Nano R4
// Tests whether XC E10 ESC X.BUS is inverted UART (like S.BUS).
// Listens on D2 via NPN inverter using SoftwareSerial.
// S.BUS on D0 (Serial1) is NOT disturbed.
//
// Wiring:
//   ESC X.BUS yellow ──[1K]──► NPN base
//                               NPN emitter ──► GND
//   5V ──[10K]──┬──► NPN collector
//               └──► D2
//   ESC X.BUS brown ──► GND (common ground)
//   ESC X.BUS red ──► NOT CONNECTED (BEC — never connect both ESCs)
//
// Tests common baud rates used by hobby ESC telemetry protocols.
// If data appears, we've found the protocol.

#include <SoftwareSerial.h>

const uint8_t XBUS_RX_PIN = 2;  // D2 (freed by S.BUS migration)
const uint8_t XBUS_TX_PIN = 3;  // D3 (not used for TX, but SoftwareSerial requires it)

// Baud rates to scan — ordered by likelihood for ESC telemetry
const int32_t BAUDS[] = {115200, 19200, 9600, 38400, 57600, 100000, 250000};
const int BAUD_COUNT = 7;

SoftwareSerial xbus(XBUS_RX_PIN, XBUS_TX_PIN);

int currentBaud = 0;
uint32_t baudStart = 0;
uint32_t totalBytes = 0;
uint32_t lastReport = 0;
uint8_t rxBuf[64];
int rxCount = 0;

void tryBaud(int idx) {
  currentBaud = idx;
  xbus.end();
  xbus.begin(BAUDS[idx]);
  baudStart = millis();
  totalBytes = 0;
  rxCount = 0;
  Serial.print("# Trying ");
  Serial.print(BAUDS[idx]);
  Serial.println(" baud...");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("# === X.BUS Inverted UART Test ===");
  Serial.println("# Listening on D2 via NPN inverter");
  Serial.println("# Scanning baud rates for ESC telemetry data...");
  Serial.println("#");

  pinMode(XBUS_RX_PIN, INPUT);
  tryBaud(0);
}

void loop() {
  // Read any available bytes
  while (xbus.available()) {
    uint8_t b = xbus.read();
    totalBytes++;

    // Buffer first 64 bytes for hex dump
    if (rxCount < 64) {
      rxBuf[rxCount++] = b;
    }

    // Print first bytes as they arrive
    if (totalBytes <= 128) {
      char hex[4];
      snprintf(hex, sizeof(hex), "%02X ", b);
      Serial.print(hex);
      if (totalBytes % 16 == 0) Serial.println();
    }
  }

  uint32_t now = millis();

  // Report every 2 seconds (once per interval)
  if (totalBytes > 0 && (now - lastReport) >= 2000) {
    lastReport = now;
    Serial.print("# ");
    Serial.print(BAUDS[currentBaud]);
    Serial.print(" baud: ");
    Serial.print(totalBytes);
    Serial.println(" bytes received");
  }

  // Switch baud after 5 seconds if no data (or very little)
  if (now - baudStart >= 5000) {
    if (totalBytes >= 10) {
      // Got data! Lock this baud rate
      Serial.println();
      Serial.print("# *** DATA FOUND at ");
      Serial.print(BAUDS[currentBaud]);
      Serial.print(" baud! ");
      Serial.print(totalBytes);
      Serial.println(" bytes ***");

      // Print hex dump of first bytes
      Serial.print("# First bytes: ");
      for (int i = 0; i < min(rxCount, 32); i++) {
        char hex[4];
        snprintf(hex, sizeof(hex), "%02X ", rxBuf[i]);
        Serial.print(hex);
      }
      Serial.println();

      // Stay on this baud and keep printing
      baudStart = now;
      totalBytes = 0;
      rxCount = 0;
    } else {
      // No data, try next baud
      int next = (currentBaud + 1) % BAUD_COUNT;
      if (next == 0) {
        Serial.println("# --- Full scan complete, no data. Restarting... ---");
        Serial.println("# Check: inverter wired correctly? ESC powered on? Yellow wire connected?");
      }
      tryBaud(next);
    }
  }
}
