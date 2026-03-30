// Excavator Track Controller — Wokwi Simulation Stub
// This is a simplified version for Wokwi visualization.
// The real sketch is in sketches/rc_test/rc_test.ino
//
// Board: Arduino UNO (stand-in for UNO Q in Wokwi)
//
// Pin Map:
//   D0  <- X.BUS shared bus (both ESCs via diode-OR)
//   D2  <- RC CH1 (left motor) via TXS0108E
//   D3  <- RC CH4 (ctrl mode) via TXS0108E
//   D4  <- RC CH2 (right motor) via TXS0108E
//   D7  <- RC CH5 (override) via TXS0108E
//   D8  -> Debug serial TX (SoftwareSerial)
//   D9  -> Left ESC servo PWM
//   D10 -> Right ESC servo PWM
//   A0  <- Joystick Y (throttle) via voltage divider
//   A1  <- Joystick X (steering) via voltage divider

#include <Servo.h>

Servo escLeft;
Servo escRight;

const int PIN_JOY_Y = A0;
const int PIN_JOY_X = A1;
const int PIN_ESC_L = 9;
const int PIN_ESC_R = 10;

void setup() {
  Serial.begin(115200);
  Serial.println("Excavator Controller - Wokwi Simulation");

  escLeft.attach(PIN_ESC_L);
  escRight.attach(PIN_ESC_R);

  // Center ESCs (neutral = 1500us)
  escLeft.writeMicroseconds(1500);
  escRight.writeMicroseconds(1500);
}

void loop() {
  int joyY = analogRead(PIN_JOY_Y);
  int joyX = analogRead(PIN_JOY_X);

  // Map joystick to ESC range (1000-2000us)
  int throttle = map(joyY, 0, 1023, -500, 500);
  int steering = map(joyX, 0, 1023, -500, 500);

  // Tank mix
  int leftUs  = 1500 + throttle + steering;
  int rightUs = 1500 + throttle - steering;

  leftUs  = constrain(leftUs, 1000, 2000);
  rightUs = constrain(rightUs, 1000, 2000);

  escLeft.writeMicroseconds(leftUs);
  escRight.writeMicroseconds(rightUs);

  // Debug output
  Serial.print("JoyY:");
  Serial.print(joyY);
  Serial.print(" JoyX:");
  Serial.print(joyX);
  Serial.print(" L:");
  Serial.print(leftUs);
  Serial.print(" R:");
  Serial.println(rightUs);

  delay(50);
}
