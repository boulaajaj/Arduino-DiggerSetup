// Digger Control V2.4 — Arduino Nano R4
// Expo + Inertia active | PID bypassed | X.BUS passive capture
// Serial = USB (debug/plot) | Serial1 = D0/D1 (X.BUS)
//
// Pin assignments:
//   D2  <- RC CH1 Left motor     [attachInterrupt]
//   D3  <- RC CH4 Control mode   [attachInterrupt]
//   D4  <- RC CH2 Right motor    [polled pulse measurement]
//   D7  <- RC CH5 Override       [polled pulse measurement]
//   A0  <- Joystick Y Throttle   [14-bit ADC, double-read]
//   A1  <- Joystick X Steering   [14-bit ADC, double-read]
//   D9  -> Left ESC              [Servo PWM]
//   D10 -> Right ESC             [Servo PWM]
//   D0  <- X.BUS telemetry       [Serial1 RX, passive capture]

#include <Servo.h>

// --- Pins ---
const uint8_t PIN_RC_CH1  = 2;   // Left motor — interrupt
const uint8_t PIN_RC_CH4  = 3;   // Control mode — interrupt
const uint8_t PIN_RC_CH2  = 4;   // Right motor — polled
const uint8_t PIN_RC_CH5  = 7;   // Override switch — polled
const uint8_t PIN_JOY_Y   = A0;
const uint8_t PIN_JOY_X   = A1;
const uint8_t PIN_ESC_L   = 9;
const uint8_t PIN_ESC_R   = 10;

// --- Constants ---
const int ADC_MAX = 16383, ADC_CENTER = 8192;
const int JOY_DEADBAND = 480, RC_DEADBAND = 50;
const int SVC = 1500, SVMIN = 1000, SVMAX = 2000;
const int MODE_LO = 1250, MODE_HI = 1750;
const float SOFT_RANGE = 250.0f;
const float VMASS = 3.0f, FRIC = 8.0f, RESP = 20.0f;
const unsigned long FAILSAFE_US = 500000UL;

Servo escL, escR;

// --- RC via interrupt (D2=CH1, D3=CH4) ---
volatile unsigned long ch1_rise = 0, ch4_rise = 0;
volatile int ch1_pw = 1500, ch4_pw = 1500;
volatile unsigned long ch1_time = 0, ch4_time = 0;

void isr_ch1() {
  unsigned long n = micros();
  if (digitalRead(PIN_RC_CH1) == HIGH) ch1_rise = n;
  else {
    unsigned long p = n - ch1_rise;
    if (p >= 800 && p <= 2200) { ch1_pw = p; ch1_time = n; }
  }
}
void isr_ch4() {
  unsigned long n = micros();
  if (digitalRead(PIN_RC_CH4) == HIGH) ch4_rise = n;
  else {
    unsigned long p = n - ch4_rise;
    if (p >= 800 && p <= 2200) { ch4_pw = p; ch4_time = n; }
  }
}

// --- RC via pulseIn (D4=CH2, D7=CH5) ---
// Polling caused CH2/CH5 value swapping due to sequential RC frame timing.
// pulseIn is blocking (~25ms per channel) but gives clean readings.
const unsigned long PULSE_TIMEOUT = 25000;  // 25ms, slightly > one 50Hz frame
int ch2_pw = 1500, ch5_pw = 1500;
unsigned long ch2_time = 0, ch5_time = 0;

void readPolledRC() {
  unsigned long p;
  p = pulseIn(PIN_RC_CH2, HIGH, PULSE_TIMEOUT);
  if (p >= 800 && p <= 2200) { ch2_pw = p; ch2_time = micros(); }
  p = pulseIn(PIN_RC_CH5, HIGH, PULSE_TIMEOUT);
  if (p >= 800 && p <= 2200) { ch5_pw = p; ch5_time = micros(); }
}

// --- X.BUS passive capture on Serial1 (D0/D1) ---
int xbtotal = 0;
bool xlocked = false;
int xbi = 0;
unsigned long xstart = 0;
const long xbauds[] = {115200, 250000, 100000, 19200, 57600, 38400, 9600};
const int XBAUD_CNT = 7;

// D0 raw activity counter (check if ANY signal present regardless of baud)
int d0_toggles = 0;
bool d0_last_state = false;

void xbusInit() {
  xbi = 0; xlocked = false;
  Serial1.begin(xbauds[0]);
  xstart = millis(); xbtotal = 0;
  d0_last_state = digitalRead(0);
}
void xbusUpdate() {
  while (Serial1.available()) { Serial1.read(); xbtotal++; }
  if (!xlocked && millis() - xstart >= 3000) {
    if (xbtotal >= 10) {
      xlocked = true;
      // Print locked baud to debug
      char msg[60];
      sprintf(msg, "# XBUS LOCKED at %ld baud (%d bytes)", xbauds[xbi], xbtotal);
      Serial.println(msg);
    } else {
      xbi++;
      if (xbi >= XBAUD_CNT) xbi = 0;
      Serial1.end();
      Serial1.begin(xbauds[xbi]);
      char msg[60];
      sprintf(msg, "# XBUS scan: trying %ld baud (prev got %d bytes)", xbauds[xbi], xbtotal);
      Serial.println(msg);
      xstart = millis();
      xbtotal = 0;
    }
  }
}

// Check D0 raw pin toggling (independent of Serial1)
void checkD0Raw() {
  // Temporarily check if D0 is changing state at all
  // This detects signal presence even if baud rate is wrong
  bool s = digitalRead(0);
  if (s != d0_last_state) { d0_toggles++; d0_last_state = s; }
}

// --- Inertia state ---
float velL = 0, velR = 0, posL = 0, posR = 0;
unsigned long prevUs = 0, prevPrint = 0;

// --- Math helpers ---
float expoCurve(float x) {
  // Attempt pow(|x|, 2.5) = x^2 * sqrt(x) via Newton's method sqrt
  float a = fabsf(x), sq = a * a, r = a;
  if (r > 0.001f) { r = 0.5f * (r + a / r); r = 0.5f * (r + a / r); }
  return sq * r;  // a^2 * sqrt(a) ≈ a^2.5
}

float fastTanh(float x) {
  float a = fabsf(x);
  return constrain(x / (1.0f + a + 0.28f * a * a), -1.0f, 1.0f);
}

int deadRC(int v) { return (abs(v - SVC) <= RC_DEADBAND) ? SVC : v; }
int deadJoy(int v) { return (abs(v - ADC_CENTER) <= JOY_DEADBAND) ? ADC_CENTER : v; }

int joyExpo(int adc) {
  float n = (float)(adc - ADC_CENTER) / (float)ADC_CENTER;
  n = constrain(n, -1.0f, 1.0f);
  float s = (n >= 0) ? 1.0f : -1.0f;
  return SVC + (int)(s * expoCurve(n) * SOFT_RANGE);
}

int softLim(float d) {
  if (fabsf(d) < 0.5f) return SVC;
  return SVC + (int)(SOFT_RANGE * fastTanh(d / SOFT_RANGE) + 0.5f);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("# === Digger Control V2.4 — Nano R4 ===");
  Serial.println("# Expo+Inertia | PID bypassed | XBUS capture");
  Serial.println("# CH1[D2] CH2[D4] CH4[D3] CH5[D7] JoyY[A0] JoyX[A1] EscL[D9] EscR[D10]");
  Serial.println("# CSV: RC1,RC2,RC4,RC5,JoyY,JoyX,OutL,OutR,XbusB");

  analogReadResolution(14);

  // RC inputs
  pinMode(PIN_RC_CH1, INPUT);
  pinMode(PIN_RC_CH4, INPUT);
  pinMode(PIN_RC_CH2, INPUT);
  pinMode(PIN_RC_CH5, INPUT);

  // Interrupts only on D2 and D3 (Nano R4 limitation)
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH1), isr_ch1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH4), isr_ch4, CHANGE);

  // D4 and D7 read via pulseIn in loop (no init needed)

  // ESC outputs — neutral
  escL.attach(PIN_ESC_L);
  escR.attach(PIN_ESC_R);
  escL.writeMicroseconds(SVC);
  escR.writeMicroseconds(SVC);

  prevUs = micros();
  xbusInit();
}

void loop() {
  unsigned long now = micros();
  unsigned long dt = now - prevUs;
  if (dt == 0) return;
  prevUs = now;
  float dts = (float)dt * 0.000001f;

  // Read D4 and D7 via pulseIn (blocking ~50ms total)
  readPolledRC();

  // X.BUS passive capture + raw D0 signal check
  xbusUpdate();
  checkD0Raw();

  // Snapshot RC values
  noInterrupts();
  int rcL = ch1_pw;   // CH1 = left motor
  int rcR = ch2_pw;   // CH2 = right motor
  int rcM = ch4_pw;   // CH4 = mode selector
  int rcO = ch5_pw;   // CH5 = override switch
  unsigned long tL = ch1_time, tR = ch2_time;
  interrupts();

  // Joystick (double-read to avoid ADC crosstalk)
  analogRead(PIN_JOY_Y); delayMicroseconds(100);
  int jy = analogRead(PIN_JOY_Y);
  analogRead(PIN_JOY_X); delayMicroseconds(100);
  int jx = analogRead(PIN_JOY_X);

  // Failsafe: no RC signal for 500ms → neutral
  bool lost = (now - tL > FAILSAFE_US) && (now - tR > FAILSAFE_US);
  int sL = lost ? SVC : deadRC(rcL);
  int sR = lost ? SVC : deadRC(rcR);

  // Joystick → expo → tank mix
  int jT = joyExpo(deadJoy(jy));
  int jS = joyExpo(deadJoy(jx));
  int jo = jS - SVC;
  int jL = constrain(jT + jo, SVMIN, SVMAX);
  int jR = constrain(jT - jo, SVMIN, SVMAX);

  // Mode selection (CH5 override switch)
  int left, right;
  if (rcO < MODE_LO) {
    // Mode 1: RC only
    left = sL; right = sR;
  } else if (rcO <= MODE_HI) {
    // Mode 2: RC overrides joystick (joystick when RC centered)
    bool rcActive = (sL != SVC) || (sR != SVC);
    if (rcActive && !lost) { left = sL; right = sR; }
    else { left = jL; right = jR; }
  } else {
    // Mode 3: 50/50 blend
    left = (sL + jL) / 2;
    right = (sR + jR) / 2;
  }

  // Soft power limit (tanh saturation)
  int scL = softLim((float)(left - SVC));
  int scR = softLim((float)(right - SVC));

  // Inertia simulation (spring-damper model)
  float tgL = (float)(scL - SVC), tgR = (float)(scR - SVC);
  velL += (RESP * (tgL - posL) - FRIC * velL) / VMASS * dts;
  velR += (RESP * (tgR - posR) - FRIC * velR) / VMASS * dts;
  posL += velL * dts;
  posR += velR * dts;

  // Settle to zero when near target and slow
  if (fabsf(tgL) < 1 && fabsf(posL) < 2 && fabsf(velL) < 5) { posL = 0; velL = 0; }
  if (fabsf(tgR) < 1 && fabsf(posR) < 2 && fabsf(velR) < 5) { posR = 0; velR = 0; }

  // Final output
  int outL = constrain(SVC + (int)(posL + 0.5f), SVMIN, SVMAX);
  int outR = constrain(SVC + (int)(posR + 0.5f), SVMIN, SVMAX);

  escL.writeMicroseconds(outL);
  escR.writeMicroseconds(outR);

  // 10 Hz CSV output via USB
  if (now - prevPrint >= 100000UL) {
    prevPrint = now;
    char buf[120];
    sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
            rcL, rcR, rcM, rcO, jy, jx, outL, outR, xbtotal);
    Serial.println(buf);
    // Print D0 raw toggle count separately as comment (doesn't break CSV)
    if (d0_toggles > 0) {
      sprintf(buf, "# D0 raw toggles: %d", d0_toggles);
      Serial.println(buf);
    }
    xbtotal = 0;
    d0_toggles = 0;
  }
}
