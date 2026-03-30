// =============================================================================
// Tank Mixer V2.3 — Arduino UNO Q (STM32U585 MCU)
// =============================================================================
//
// Active layers: Exponential curve + Inertia simulation
// Bypassed:      Current feedforward, RPM feedback (no data source yet)
//
// X.BUS on D0: passive listen — raw hex dump for protocol discovery.
// Output via Monitor (Router Bridge → USB) for live_plot.py.
//
// Inputs:
//   D0  <- ESC X.BUS (Serial/USART1 RX) — passive listen, raw capture
//   D2  <- RC CH1 left motor     (servo PWM, interrupt)
//   D3  <- RC CH4 control mode   (servo PWM, interrupt)
//   D4  <- RC CH2 right motor    (servo PWM, interrupt)
//   D7  <- RC CH5 override       (servo PWM, interrupt)
//   A0  <- Joystick Y throttle   (14-bit ADC, via voltage divider)
//   A1  <- Joystick X steering   (14-bit ADC, via voltage divider)
//
// Outputs:
//   D9  -> Left track ESC        (servo PWM)
//   D10 -> Right track ESC       (servo PWM)
//
// =============================================================================

#include <math.h>
#include <Servo.h>
#include <Arduino_RouterBridge.h>

// Debug output via Router Bridge → USB (not Serial, which is D0/D1)
#define OUT Monitor

// =============================================================================
// Pins
// =============================================================================
const uint8_t PIN_RC_LEFT     = 2;
const uint8_t PIN_RC_CTRLMODE = 3;
const uint8_t PIN_RC_RIGHT    = 4;
const uint8_t PIN_RC_OVERRIDE = 7;
const uint8_t PIN_JOY_Y       = A0;
const uint8_t PIN_JOY_X       = A1;
const uint8_t PIN_ESC_LEFT    = 9;
const uint8_t PIN_ESC_RIGHT   = 10;

// =============================================================================
// Constants
// =============================================================================
const int   ADC_MAX          = 16383;
const int   ADC_CENTER       = 8192;
const float VOLTAGE_WARN     = 3.2f;   // warn if joystick voltage exceeds this
const int   JOY_DEADBAND     = 480;
const float EXP_CURVE        = 2.5f;
const int   RC_DEADBAND      = 50;
const int   SERVO_CENTER     = 1500;
const int   SERVO_MIN        = 1000;
const int   SERVO_MAX        = 2000;
const int   MODE_LOW_THRESH  = 1250;
const int   MODE_HIGH_THRESH = 1750;
const int   POWER_LIMIT_PCT  = 50;
const float SOFT_LIMIT_RANGE = 500.0f * POWER_LIMIT_PCT / 100.0f;

// Inertia simulation
const float VIRTUAL_MASS     = 3.0f;
const float FRICTION_COEFF   = 8.0f;
const float RESPONSE_FORCE   = 20.0f;

// Timing
const unsigned long FAILSAFE_TIMEOUT_US = 500000UL;
const unsigned long PRINT_INTERVAL_US   = 100000UL;  // 10 Hz output

// =============================================================================
// RC Interrupts
// =============================================================================
volatile unsigned long riseTime[4]   = {0, 0, 0, 0};
volatile int           pulseWidth[4] = {1500, 1500, 1000, 1500};
volatile unsigned long pulseTime[4]  = {0, 0, 0, 0};

const uint8_t CH_LEFT = 0, CH_RIGHT = 1, CH_OVR = 2, CH_CTRL = 3;

void isrLeft() {
  unsigned long now = micros();
  if (digitalRead(PIN_RC_LEFT) == HIGH) riseTime[CH_LEFT] = now;
  else { unsigned long pw = now - riseTime[CH_LEFT];
    if (pw >= 800 && pw <= 2200) { pulseWidth[CH_LEFT] = pw; pulseTime[CH_LEFT] = now; } }
}
void isrRight() {
  unsigned long now = micros();
  if (digitalRead(PIN_RC_RIGHT) == HIGH) riseTime[CH_RIGHT] = now;
  else { unsigned long pw = now - riseTime[CH_RIGHT];
    if (pw >= 800 && pw <= 2200) { pulseWidth[CH_RIGHT] = pw; pulseTime[CH_RIGHT] = now; } }
}
void isrOverride() {
  unsigned long now = micros();
  if (digitalRead(PIN_RC_OVERRIDE) == HIGH) riseTime[CH_OVR] = now;
  else { unsigned long pw = now - riseTime[CH_OVR];
    if (pw >= 800 && pw <= 2200) { pulseWidth[CH_OVR] = pw; pulseTime[CH_OVR] = now; } }
}
void isrCtrlMode() {
  unsigned long now = micros();
  if (digitalRead(PIN_RC_CTRLMODE) == HIGH) riseTime[CH_CTRL] = now;
  else { unsigned long pw = now - riseTime[CH_CTRL];
    if (pw >= 800 && pw <= 2200) { pulseWidth[CH_CTRL] = pw; pulseTime[CH_CTRL] = now; } }
}

// =============================================================================
// X.BUS — Passive Raw Capture on D0
// =============================================================================
const int XBUS_SAMPLE_SIZE = 32;         // bytes to show per print cycle
uint8_t xbusSample[XBUS_SAMPLE_SIZE];
int     xbusSampleLen  = 0;
int     xbusBytesTotal = 0;              // total bytes since last print
bool    xbusBaudLocked = false;
int     xbusBaudIdx    = 0;
unsigned long xbusScanStart = 0;

const long XBUS_BAUDS[]  = {115200, 250000, 100000, 19200, 57600, 38400, 9600};
const int  XBUS_BAUD_CNT = 7;

void xbusStartScan() {
  xbusBaudIdx = 0;
  xbusBaudLocked = false;
  Serial.begin(XBUS_BAUDS[0]);
  xbusScanStart = millis();
  xbusBytesTotal = 0;
  xbusSampleLen = 0;
  OUT.print("XBUS: scanning ");
  OUT.print(XBUS_BAUDS[0]);
  OUT.println(" baud...");
}

// Call each loop — reads bytes, captures a sample, handles baud scanning
void xbusUpdate() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    xbusBytesTotal++;
    if (xbusSampleLen < XBUS_SAMPLE_SIZE) {
      xbusSample[xbusSampleLen++] = b;
    }
  }

  if (!xbusBaudLocked) {
    if (millis() - xbusScanStart >= 3000) {
      if (xbusBytesTotal >= 10) {
        xbusBaudLocked = true;
        OUT.print("XBUS: LOCKED ");
        OUT.print(XBUS_BAUDS[xbusBaudIdx]);
        OUT.print(" baud (");
        OUT.print(xbusBytesTotal);
        OUT.println(" bytes)");
      } else {
        xbusBaudIdx++;
        if (xbusBaudIdx >= XBUS_BAUD_CNT) xbusBaudIdx = 0;
        Serial.end();
        Serial.begin(XBUS_BAUDS[xbusBaudIdx]);
        xbusScanStart = millis();
        xbusBytesTotal = 0;
        xbusSampleLen = 0;
        OUT.print("XBUS: trying ");
        OUT.print(XBUS_BAUDS[xbusBaudIdx]);
        OUT.println(" baud...");
      }
    }
  }
}

// Print a clean hex line of the latest sample
void xbusPrintSample() {
  OUT.print("  XBUS ");
  OUT.print(xbusBytesTotal);
  OUT.print("B");
  if (xbusBaudLocked) {
    OUT.print(" @");
    OUT.print(XBUS_BAUDS[xbusBaudIdx]);
  }
  if (xbusSampleLen > 0) {
    OUT.print(" | ");
    for (int i = 0; i < xbusSampleLen && i < 24; i++) {
      if (xbusSample[i] < 0x10) OUT.print("0");
      OUT.print(xbusSample[i], HEX);
      OUT.print(" ");
    }
    if (xbusSampleLen > 24) OUT.print("...");
  } else {
    OUT.print(" | (no data)");
  }
  OUT.println();
  // Reset for next cycle
  xbusBytesTotal = 0;
  xbusSampleLen = 0;
}

// =============================================================================
// Servo Objects
// =============================================================================
Servo escLeft, escRight;

// =============================================================================
// Inertia State
// =============================================================================
float velocityL = 0, velocityR = 0;
float positionL = 0, positionR = 0;

// =============================================================================
// Timing
// =============================================================================
unsigned long prevLoopUs   = 0;
unsigned long prevPrintUs  = 0;

// =============================================================================
// Helpers
// =============================================================================

int deadbandRC(int v) { return (abs(v - SERVO_CENTER) <= RC_DEADBAND) ? SERVO_CENTER : v; }
int deadbandJoy(int v) { return (abs(v - ADC_CENTER) <= JOY_DEADBAND) ? ADC_CENTER : v; }

// Attempt to link math — Zephyr may need explicit linkage
extern "C" float powf(float base, float exp);
extern "C" float tanhf(float x);

// Fallback: manual expo if powf unavailable at link time
// x^2.5 ≈ x*x*sqrt(x)
float expoCurve(float x) {
  float ax = fabsf(x);
  // x^2.5 = x^2 * x^0.5
  float sq = ax * ax;
  // Fast sqrt via Newton's method (2 iterations, good enough)
  float s = ax;
  if (s > 0.001f) {
    s = 0.5f * (s + ax / s);
    s = 0.5f * (s + ax / s);
  }
  return sq * s;  // ax^2.5
}

// Fast tanh approximation: tanh(x) ≈ x / (1 + |x| + 0.28*x^2)
float fastTanh(float x) {
  float ax = fabsf(x);
  float t = x / (1.0f + ax + 0.28f * ax * ax);
  return constrain(t, -1.0f, 1.0f);
}

int joyToServoExpo(int adc) {
  float norm = (float)(adc - ADC_CENTER) / (float)ADC_CENTER;
  norm = constrain(norm, -1.0f, 1.0f);
  float sign = (norm >= 0) ? 1.0f : -1.0f;
  float curved = sign * expoCurve(norm);
  return SERVO_CENTER + (int)(curved * SOFT_LIMIT_RANGE);
}

int softLimit(float delta) {
  if (fabsf(delta) < 0.5f) return SERVO_CENTER;
  float s = SOFT_LIMIT_RANGE * fastTanh(delta / SOFT_LIMIT_RANGE);
  return SERVO_CENTER + (int)(s + 0.5f);
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Bridge.begin();
  Monitor.begin();
  delay(3000);

  analogReadResolution(14);

  pinMode(PIN_RC_LEFT, INPUT);
  pinMode(PIN_RC_CTRLMODE, INPUT);
  pinMode(PIN_RC_RIGHT, INPUT);
  pinMode(PIN_RC_OVERRIDE, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_LEFT),     isrLeft,     CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CTRLMODE), isrCtrlMode, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_RIGHT),    isrRight,    CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_OVERRIDE), isrOverride, CHANGE);

  escLeft.attach(PIN_ESC_LEFT);
  escRight.attach(PIN_ESC_RIGHT);
  escLeft.writeMicroseconds(SERVO_CENTER);
  escRight.writeMicroseconds(SERVO_CENTER);

  prevLoopUs = micros();

  OUT.println();
  OUT.println("=== Tank Mixer V2.3 ===");
  OUT.println("Layers: Expo + Inertia (PID bypassed)");
  OUT.println("X.BUS: passive raw capture on D0");
  OUT.println();

  xbusStartScan();
}

// =============================================================================
// Main Loop
// =============================================================================
void loop() {
  unsigned long nowUs = micros();
  unsigned long dtUs = nowUs - prevLoopUs;
  if (dtUs == 0) return;
  prevLoopUs = nowUs;
  float dt = (float)dtUs * 0.000001f;

  // --- X.BUS passive capture (always runs, never blocks) ---
  xbusUpdate();

  // --- RC snapshot ---
  noInterrupts();
  int rcL   = pulseWidth[CH_LEFT];
  int rcR   = pulseWidth[CH_RIGHT];
  int rcOvr = pulseWidth[CH_OVR];
  int rcMod = pulseWidth[CH_CTRL];
  unsigned long tL = pulseTime[CH_LEFT];
  unsigned long tR = pulseTime[CH_RIGHT];
  interrupts();

  // --- Joystick ---
  int rawJoyY = analogRead(PIN_JOY_Y);
  int rawJoyX = analogRead(PIN_JOY_X);
  float joyVoltY = (rawJoyY / (float)ADC_MAX) * 3.3f;
  float joyVoltX = (rawJoyX / (float)ADC_MAX) * 3.3f;

  // --- Failsafe ---
  bool rcLost = (nowUs - tL > FAILSAFE_TIMEOUT_US) && (nowUs - tR > FAILSAFE_TIMEOUT_US);
  int safeL = rcLost ? SERVO_CENTER : deadbandRC(rcL);
  int safeR = rcLost ? SERVO_CENTER : deadbandRC(rcR);

  // --- Joystick processing (expo + tank mix) ---
  int joyThrottle = joyToServoExpo(deadbandJoy(rawJoyY));
  int joySteering = joyToServoExpo(deadbandJoy(rawJoyX));
  int joySteerOff = joySteering - SERVO_CENTER;
  int joyL = constrain(joyThrottle + joySteerOff, SERVO_MIN, SERVO_MAX);
  int joyR = constrain(joyThrottle - joySteerOff, SERVO_MIN, SERVO_MAX);

  // --- Override mode (CH5) ---
  int left, right;
  if (rcOvr < MODE_LOW_THRESH) {
    // Mode 1: RC only
    left = safeL; right = safeR;
  } else if (rcOvr <= MODE_HIGH_THRESH) {
    // Mode 2: Joystick, RC overrides when active
    bool rcActive = (safeL != SERVO_CENTER) || (safeR != SERVO_CENTER);
    if (rcActive && !rcLost) { left = safeL; right = safeR; }
    else { left = joyL; right = joyR; }
  } else {
    // Mode 3: 50/50 blend
    left = (safeL + joyL) / 2;
    right = (safeR + joyR) / 2;
  }

  // --- Soft power limit ---
  float deltaL = (float)(left - SERVO_CENTER);
  float deltaR = (float)(right - SERVO_CENTER);
  int scaledL = softLimit(deltaL);
  int scaledR = softLimit(deltaR);

  // --- Inertia simulation ---
  float targetL = (float)(scaledL - SERVO_CENTER);
  float targetR = (float)(scaledR - SERVO_CENTER);

  float forceL = RESPONSE_FORCE * (targetL - positionL);
  float forceR = RESPONSE_FORCE * (targetR - positionR);
  float fricL  = -FRICTION_COEFF * velocityL;
  float fricR  = -FRICTION_COEFF * velocityR;

  velocityL += (forceL + fricL) / VIRTUAL_MASS * dt;
  velocityR += (forceR + fricR) / VIRTUAL_MASS * dt;
  positionL += velocityL * dt;
  positionR += velocityR * dt;

  // Snap to zero when nearly stopped
  if (fabsf(targetL) < 1 && fabsf(positionL) < 2 && fabsf(velocityL) < 5) {
    positionL = 0; velocityL = 0;
  }
  if (fabsf(targetR) < 1 && fabsf(positionR) < 2 && fabsf(velocityR) < 5) {
    positionR = 0; velocityR = 0;
  }

  int outL = constrain(SERVO_CENTER + (int)(positionL + 0.5f), SERVO_MIN, SERVO_MAX);
  int outR = constrain(SERVO_CENTER + (int)(positionR + 0.5f), SERVO_MIN, SERVO_MAX);

  // --- Write to ESCs ---
  escLeft.writeMicroseconds(outL);
  escRight.writeMicroseconds(outR);

  // --- Print telemetry at 10 Hz ---
  if (nowUs - prevPrintUs >= PRINT_INTERVAL_US) {
    prevPrintUs = nowUs;

    // Voltage warning
    bool vWarn = (joyVoltY > VOLTAGE_WARN) || (joyVoltX > VOLTAGE_WARN);

    // Line 1: inputs + outputs
    OUT.print("RC1:");  OUT.print(rcL);
    OUT.print(" RC2:"); OUT.print(rcR);
    OUT.print(" RC4:"); OUT.print(rcMod);
    OUT.print(" RC5:"); OUT.print(rcOvr);
    OUT.print(" | JY:"); OUT.print(rawJoyY);
    OUT.print("(");      OUT.print(joyVoltY, 2); OUT.print("V)");
    OUT.print(" JX:");   OUT.print(rawJoyX);
    OUT.print("(");      OUT.print(joyVoltX, 2); OUT.print("V)");
    if (vWarn) OUT.print(" **VHIGH**");
    OUT.print(" | L:"); OUT.print(outL);
    OUT.print(" R:");   OUT.print(outR);
    if (rcLost) OUT.print(" **RCLOST**");
    OUT.println();

    // Line 2: X.BUS raw data
    xbusPrintSample();
  }
}
