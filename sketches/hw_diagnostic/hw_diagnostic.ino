// ============================================================
// Hardware Diagnostic Sketch — Arduino UNO Q
// Version: 1.1 (2026-03-29)
//
// Tests ALL connected hardware and reports via USB Serial Monitor.
//
// IMPORTANT: On UNO Q, Serial goes to D0/D1 (hardware UART).
// To output to USB Serial Monitor, we use Monitor (Router bridge).
//
// Current setup:
//   - One ESC X.BUS on D0 (can stay connected — we use Monitor, not Serial)
//   - RC receiver on D2, D3, D4, D7 (via TXS0108E level shifter)
//   - Joystick on A0, A1 (via voltage dividers)
//   - ESC servo outputs on D9, D10
//   - Debug output: Monitor (USB via Router bridge to Qualcomm MPU)
//
// HOW TO USE:
//   1. Upload this sketch
//   2. Open Serial Monitor at 115200 baud (Arduino IDE or CLI)
//   3. Observe test results for each subsystem
//   4. Move joystick and RC sticks to verify all inputs
// ============================================================

#include <Servo.h>
#include <Arduino_RouterBridge.h>

// Use Monitor for USB output (not Serial — that goes to D0/D1)
#define OUT Monitor

// ---- Pin Definitions ----
const int PIN_RC_CH1  = 2;   // Left motor (pre-mixed by TX)
const int PIN_RC_CH4  = 3;   // Control mode selector
const int PIN_RC_CH2  = 4;   // Right motor (pre-mixed by TX)
const int PIN_RC_CH5  = 7;   // Override switch
const int PIN_JOY_Y   = A0;  // Joystick throttle (via divider)
const int PIN_JOY_X   = A1;  // Joystick steering (via divider)
const int PIN_ESC_L   = 9;   // Left ESC servo output
const int PIN_ESC_R   = 10;  // Right ESC servo output

// ---- RC Pulse Reading (interrupt-based) ----
volatile unsigned long rc1_rise = 0, rc1_pw = 0;
volatile unsigned long rc2_rise = 0, rc2_pw = 0;
volatile unsigned long rc4_rise = 0, rc4_pw = 0;
volatile unsigned long rc5_rise = 0, rc5_pw = 0;
volatile bool rc1_new = false, rc2_new = false;
volatile bool rc4_new = false, rc5_new = false;

void isr_rc1() {
  if (digitalRead(PIN_RC_CH1) == HIGH) rc1_rise = micros();
  else { rc1_pw = micros() - rc1_rise; rc1_new = true; }
}
void isr_rc2() {
  if (digitalRead(PIN_RC_CH2) == HIGH) rc2_rise = micros();
  else { rc2_pw = micros() - rc2_rise; rc2_new = true; }
}
void isr_rc4() {
  if (digitalRead(PIN_RC_CH4) == HIGH) rc4_rise = micros();
  else { rc4_pw = micros() - rc4_rise; rc4_new = true; }
}
void isr_rc5() {
  if (digitalRead(PIN_RC_CH5) == HIGH) rc5_rise = micros();
  else { rc5_pw = micros() - rc5_rise; rc5_new = true; }
}

// ---- ESC Outputs ----
Servo escLeft, escRight;

// ---- Timing ----
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 500;  // ms
unsigned long testStart = 0;
int summaryCount = 0;

// ---- ADC Voltage Tracking ----
int joyYMin = 16383, joyYMax = 0;
int joyXMin = 16383, joyXMax = 0;

// ---- X.BUS data detection ----
int xbusBytes = 0;

void setup() {
  // Start the Router bridge (connects MCU to MPU for USB output)
  Bridge.begin();
  Monitor.begin();
  delay(2000);  // Give bridge time to initialize

  // Also open Serial (USART1 on D0) to listen for X.BUS data
  Serial.begin(115200);

  OUT.println("");
  OUT.println("========================================");
  OUT.println("  HARDWARE DIAGNOSTIC — Arduino UNO Q");
  OUT.println("  Excavator Track Controller V2.3");
  OUT.println("========================================");
  OUT.println("");

  // Configure ADC for 14-bit resolution
  analogReadResolution(14);

  // RC inputs
  pinMode(PIN_RC_CH1, INPUT);
  pinMode(PIN_RC_CH2, INPUT);
  pinMode(PIN_RC_CH4, INPUT);
  pinMode(PIN_RC_CH5, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH1), isr_rc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH2), isr_rc2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH4), isr_rc4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH5), isr_rc5, CHANGE);

  // ESC outputs — hold neutral
  escLeft.attach(PIN_ESC_L);
  escRight.attach(PIN_ESC_R);
  escLeft.writeMicroseconds(1500);
  escRight.writeMicroseconds(1500);
  OUT.println("ESCs: D9 and D10 set to NEUTRAL (1500us)");
  OUT.println("");

  // ---- Phase 1: Voltage Safety Check ----
  OUT.println("=== PHASE 1: VOLTAGE SAFETY CHECK ===");
  OUT.println("Reading joystick analog pins (100 samples)...");
  OUT.println("");

  int maxY = 0, maxX = 0;
  for (int i = 0; i < 100; i++) {
    int y = analogRead(PIN_JOY_Y);
    int x = analogRead(PIN_JOY_X);
    if (y > maxY) maxY = y;
    if (x > maxX) maxX = x;
    delay(5);
  }

  float voltY = (maxY / 16383.0) * 3.3;
  float voltX = (maxX / 16383.0) * 3.3;

  OUT.print("  A0 (Joy Y): raw=");
  OUT.print(maxY);
  OUT.print("/16383  voltage=");
  OUT.print(voltY, 3);
  OUT.println("V");

  OUT.print("  A1 (Joy X): raw=");
  OUT.print(maxX);
  OUT.print("/16383  voltage=");
  OUT.print(voltX, 3);
  OUT.println("V");

  if (voltY > 3.2 || voltX > 3.2) {
    OUT.println("");
    OUT.println("  *** WARNING: VOLTAGE NEAR 3.3V LIMIT! ***");
    OUT.println("  Check your resistor divider values.");
    if (voltY > 3.3 || voltX > 3.3) {
      OUT.println("  !!! DANGER: EXCEEDS 3.3V — DISCONNECT IMMEDIATELY !!!");
    }
  } else if (maxY < 100 && maxX < 100) {
    OUT.println("");
    OUT.println("  NOTE: Both readings near zero.");
    OUT.println("  Joystick may not be connected or powered.");
  } else {
    OUT.println("  Voltage levels: OK (within 3.3V ADC range)");
  }

  OUT.println("");
  OUT.println("=== PHASE 2: CONTINUOUS MONITORING ===");
  OUT.println("Reporting every 500ms. Move sticks to test.");
  OUT.println("");
  OUT.println("  RC1=left  RC2=right  RC4=mode  RC5=override");
  OUT.println("  JoyY=throttle  JoyX=steering");
  OUT.println("  Valid RC pulse: 900-2100us. Center ~1500us.");
  OUT.println("  '--' = no signal detected on that channel.");
  OUT.println("");

  testStart = millis();
}

void loop() {
  unsigned long now = millis();

  // Track joystick min/max
  int jy = analogRead(PIN_JOY_Y);
  int jx = analogRead(PIN_JOY_X);
  if (jy < joyYMin) joyYMin = jy;
  if (jy > joyYMax) joyYMax = jy;
  if (jx < joyXMin) joyXMin = jx;
  if (jx > joyXMax) joyXMax = jx;

  // Check for X.BUS data on Serial (D0)
  while (Serial.available()) {
    Serial.read();
    xbusBytes++;
  }

  // Print report
  if (now - lastPrint >= PRINT_INTERVAL) {
    lastPrint = now;
    float elapsed = (now - testStart) / 1000.0;

    // ---- One-line status ----
    OUT.print("[");
    OUT.print(elapsed, 1);
    OUT.print("s] ");

    // RC channels
    printRcInline("RC1:", rc1_pw);
    printRcInline("RC2:", rc2_pw);
    printRcInline("RC4:", rc4_pw);
    printRcInline("RC5:", rc5_pw);

    // Joystick
    float vy = (jy / 16383.0) * 3.3;
    float vx = (jx / 16383.0) * 3.3;
    OUT.print("| JY:");
    OUT.print(vy, 2);
    OUT.print("V JX:");
    OUT.print(vx, 2);
    OUT.print("V ");

    // Voltage max tracker
    float maxVY = (joyYMax / 16383.0) * 3.3;
    float maxVX = (joyXMax / 16383.0) * 3.3;
    OUT.print("| MAX Y:");
    OUT.print(maxVY, 2);
    OUT.print(" X:");
    OUT.print(maxVX, 2);

    if (maxVY > 3.2 || maxVX > 3.2) OUT.print(" ***HIGH***");

    // X.BUS data indicator
    OUT.print(" | XBUS:");
    if (xbusBytes > 0) {
      OUT.print(xbusBytes);
      OUT.print("B");
    } else {
      OUT.print("--");
    }

    OUT.println();

    // Reset X.BUS byte counter each print
    xbusBytes = 0;

    // Summary every 10 seconds
    summaryCount++;
    if (summaryCount >= 20) {
      summaryCount = 0;
      printSummary();
    }
  }
}

void printRcInline(const char* label, unsigned long pw) {
  OUT.print(label);
  if (pw == 0) {
    OUT.print("-- ");
  } else {
    OUT.print(pw);
    OUT.print(" ");
  }
}

void printSummary() {
  OUT.println("");
  OUT.println("--- 10s SUMMARY ---");

  OUT.print("  RC CH1 (D2, left):  "); printRcStatus(rc1_pw);
  OUT.print("  RC CH2 (D4, right): "); printRcStatus(rc2_pw);
  OUT.print("  RC CH4 (D3, mode):  "); printRcStatus(rc4_pw);
  OUT.print("  RC CH5 (D7, ovrd):  "); printRcStatus(rc5_pw);

  float minVY = (joyYMin / 16383.0) * 3.3;
  float maxVY = (joyYMax / 16383.0) * 3.3;
  float minVX = (joyXMin / 16383.0) * 3.3;
  float maxVX = (joyXMax / 16383.0) * 3.3;

  OUT.print("  Joy Y range: ");
  OUT.print(minVY, 3); OUT.print("V - "); OUT.print(maxVY, 3); OUT.println("V");
  OUT.print("  Joy X range: ");
  OUT.print(minVX, 3); OUT.print("V - "); OUT.print(maxVX, 3); OUT.println("V");

  if (maxVY > 3.2 || maxVX > 3.2) {
    OUT.println("  *** WARNING: Joystick max voltage near 3.3V limit! ***");
  }

  OUT.println("  ESC L (D9): NEUTRAL 1500us");
  OUT.println("  ESC R (D10): NEUTRAL 1500us");
  OUT.println("---");
  OUT.println("");
}

void printRcStatus(unsigned long pw) {
  if (pw == 0) {
    OUT.println("NO SIGNAL");
  } else if (pw < 900 || pw > 2200) {
    OUT.print(pw); OUT.println("us - OUT OF RANGE");
  } else {
    OUT.print(pw); OUT.print("us - OK");
    if (pw > 1450 && pw < 1550) OUT.println(" (centered)");
    else if (pw < 1100) OUT.println(" (LOW)");
    else if (pw > 1900) OUT.println(" (HIGH)");
    else OUT.println();
  }
}
