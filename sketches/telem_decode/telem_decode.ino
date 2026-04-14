// ESC X.BUS — Serial1 D0(RX)/D1(TX) at 19200 through ADUM1201
// Joystick direct drive on A0/A1. No SoftwareSerial.

#include <Servo.h>

const uint8_t PIN_ESC_L = 9, PIN_ESC_R = 10;
const uint8_t PIN_JOY_Y = A0, PIN_JOY_X = A1;

Servo escL, escR;
const int SVC = 1500, SVMIN = 1000, SVMAX = 2000;
const int ADC_CENTER = 8192, JOY_DEADBAND = 480;

const uint8_t POLL[] = {0x9B, 0x03, 0x00, 0x00, 0x00, 0x9E};

uint32_t lastPoll = 0, lastPrint = 0, totalRx = 0;
uint8_t rxBuf[64];
int rxCount = 0;
int outL = SVC, outR = SVC;

int joyToServo(int adc) {
  int centered = adc - ADC_CENTER;
  if (abs(centered) < JOY_DEADBAND) return SVC;
  float norm = constrain((float)centered / ADC_CENTER, -1.0f, 1.0f);
  return SVC + (int)(norm * 400.0f);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("# === X.BUS — Serial1 19200 + ADUM1201 ===");

  analogReadResolution(14);
  Serial1.begin(19200);

  escL.attach(PIN_ESC_L);
  escR.attach(PIN_ESC_R);
  escL.writeMicroseconds(SVC);
  escR.writeMicroseconds(SVC);
}

void loop() {
  uint32_t now = millis();

  // Joystick
  analogRead(PIN_JOY_Y); delayMicroseconds(100);
  int jy = analogRead(PIN_JOY_Y);
  analogRead(PIN_JOY_X); delayMicroseconds(100);
  int jx = analogRead(PIN_JOY_X);

  int throttle = joyToServo(jy);
  int steer = joyToServo(jx);
  int offset = steer - SVC;
  outL = constrain(throttle + offset, SVMIN, SVMAX);
  outR = constrain(throttle - offset, SVMIN, SVMAX);
  escL.writeMicroseconds(outL);
  escR.writeMicroseconds(outR);

  // Read Serial1
  while (Serial1.available() && rxCount < 64) {
    rxBuf[rxCount++] = Serial1.read();
    totalRx++;
  }

  // Poll every 50ms
  if (now - lastPoll >= 50) {
    lastPoll = now;
    Serial1.write(POLL, 6);
    Serial1.flush();
  }

  // Print every 2s
  if (now - lastPrint >= 2000) {
    lastPrint = now;
    Serial.print("# rx="); Serial.print(totalRx);
    Serial.print(" outL="); Serial.print(outL);
    Serial.print(" outR="); Serial.print(outR);
    if (rxCount > 0) {
      Serial.print(" | ");
      for (int i = 0; i < rxCount && i < 32; i++) {
        char hex[4];
        snprintf(hex, sizeof(hex), "%02X ", rxBuf[i]);
        Serial.print(hex);
      }
    }
    Serial.println();
    rxCount = 0;
  }
}
