// ============================================================
// Hardware Diagnostic Sketch — Arduino Nano R4
// Version: 2.1 (2026-03-31)
//
// Uses pulseIn() for RC channels (not interrupts — D4 and D7
// don't support attachInterrupt on Nano R4 Renesas core).
//
// CSV format (~10 Hz):
//   RC1,RC2,RC4,RC5,JoyY,JoyX,OutL,OutR,XbusB
//
// Pin assignments:
//   D2  <- RC CH1 (Left motor)       [pulseIn]
//   D4  <- RC CH2 (Right motor)      [pulseIn]
//   D3  <- RC CH4 (Control mode)     [pulseIn]
//   D7  <- RC CH5 (Override switch)  [pulseIn]
//   A0  <- Joystick Y (Throttle)
//   A1  <- Joystick X (Steering)
//   D9  -> Left ESC  (Servo PWM, held neutral)
//   D10 -> Right ESC (Servo PWM, held neutral)
// ============================================================

#include <Servo.h>

// ---- Pin Definitions ----
const int PIN_RC_CH1 = 2;   // Left motor (pre-mixed by TX)
const int PIN_RC_CH4 = 3;   // Control mode selector
const int PIN_RC_CH2 = 4;   // Right motor (pre-mixed by TX)
const int PIN_RC_CH5 = 7;   // Override switch
const int PIN_JOY_Y  = A0;  // Joystick throttle
const int PIN_JOY_X  = A1;  // Joystick steering
const int PIN_ESC_L  = 9;   // Left ESC servo output
const int PIN_ESC_R  = 10;  // Right ESC servo output

// ---- ESC Outputs ----
Servo escLeft, escRight;

// ---- Timing ----
unsigned long lastSummary = 0;
const unsigned long SUMMARY_INTERVAL = 10000;
unsigned long startTime = 0;

// ---- Voltage Tracking ----
int joyYMin = 16383, joyYMax = 0;
int joyXMin = 16383, joyXMax = 0;

// ---- RC pulse timeout (25ms = slightly more than one 50Hz frame) ----
const unsigned long PULSE_TIMEOUT = 25000;  // microseconds

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  delay(500);

  Serial.println("# ==========================================");
  Serial.println("# HARDWARE DIAGNOSTIC v2.1 — Arduino Nano R4");
  Serial.println("# RC read via pulseIn (no interrupts)");
  Serial.println("# ==========================================");

  // 14-bit ADC
  analogReadResolution(14);

  // RC inputs
  pinMode(PIN_RC_CH1, INPUT);
  pinMode(PIN_RC_CH2, INPUT);
  pinMode(PIN_RC_CH4, INPUT);
  pinMode(PIN_RC_CH5, INPUT);

  // ESC outputs — hold neutral
  escLeft.attach(PIN_ESC_L);
  escRight.attach(PIN_ESC_R);
  escLeft.writeMicroseconds(1500);
  escRight.writeMicroseconds(1500);

  Serial.println("# Format: RC1,RC2,RC4,RC5,JoyY,JoyX,OutL,OutR,XbusB");
  Serial.println("# RC CH1[D2], CH2[D4], CH4[D3], CH5[D7]");
  Serial.println("# Joy Y[A0], Joy X[A1], ESC L[D9], ESC R[D10]");
  Serial.println("# ");

  startTime = millis();
}

void loop() {
  // ---- Read RC channels via pulseIn ----
  unsigned long pw1 = pulseIn(PIN_RC_CH1, HIGH, PULSE_TIMEOUT);
  unsigned long pw2 = pulseIn(PIN_RC_CH2, HIGH, PULSE_TIMEOUT);
  unsigned long pw4 = pulseIn(PIN_RC_CH4, HIGH, PULSE_TIMEOUT);
  unsigned long pw5 = pulseIn(PIN_RC_CH5, HIGH, PULSE_TIMEOUT);

  // ---- Read joystick (double-read to avoid ADC channel crosstalk) ----
  // First read after channel switch picks up residual charge from previous channel.
  // Discard it and read again for a clean sample.
  analogRead(PIN_JOY_Y); delayMicroseconds(100);
  int jy = analogRead(PIN_JOY_Y);
  analogRead(PIN_JOY_X); delayMicroseconds(100);
  int jx = analogRead(PIN_JOY_X);

  // Track min/max
  if (jy < joyYMin) joyYMin = jy;
  if (jy > joyYMax) joyYMax = jy;
  if (jx < joyXMin) joyXMin = jx;
  if (jx > joyXMax) joyXMax = jx;

  // ---- CSV output ----
  Serial.print(pw1); Serial.print(",");
  Serial.print(pw2); Serial.print(",");
  Serial.print(pw4); Serial.print(",");
  Serial.print(pw5); Serial.print(",");
  Serial.print(jy);  Serial.print(",");
  Serial.print(jx);  Serial.print(",");
  Serial.print(1500); Serial.print(",");
  Serial.print(1500); Serial.print(",");
  Serial.println(0);

  // ---- Summary every 10s ----
  unsigned long now = millis();
  if (now - lastSummary >= SUMMARY_INTERVAL) {
    lastSummary = now;
    float elapsed = (now - startTime) / 1000.0;

    Serial.print("# --- SUMMARY at ");
    Serial.print(elapsed, 0);
    Serial.println("s ---");

    printRcSummary("RC CH1 [D2] left ", pw1);
    printRcSummary("RC CH2 [D4] right", pw2);
    printRcSummary("RC CH4 [D3] mode ", pw4);
    printRcSummary("RC CH5 [D7] ovrd ", pw5);

    float minVY = (joyYMin / 16383.0) * 5.0;
    float maxVY = (joyYMax / 16383.0) * 5.0;
    float minVX = (joyXMin / 16383.0) * 5.0;
    float maxVX = (joyXMax / 16383.0) * 5.0;

    Serial.print("#   Joy Y [A0] range: ");
    Serial.print(minVY, 3); Serial.print("V - ");
    Serial.print(maxVY, 3); Serial.println("V");

    Serial.print("#   Joy X [A1] range: ");
    Serial.print(minVX, 3); Serial.print("V - ");
    Serial.print(maxVX, 3); Serial.println("V");

    Serial.println("# ---");
  }
}

void printRcSummary(const char* label, unsigned long pw) {
  Serial.print("#   ");
  Serial.print(label);
  Serial.print(": ");
  if (pw == 0) {
    Serial.println("NO SIGNAL");
  } else if (pw < 900 || pw > 2200) {
    Serial.print(pw); Serial.println("us — OUT OF RANGE");
  } else {
    Serial.print(pw); Serial.print("us — OK");
    if (pw > 1450 && pw < 1550) Serial.println(" (centered)");
    else if (pw < 1100) Serial.println(" (LOW)");
    else if (pw > 1900) Serial.println(" (HIGH)");
    else Serial.println();
  }
}
