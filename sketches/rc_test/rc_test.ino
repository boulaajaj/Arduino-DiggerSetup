// Digger Control V2.5 — Arduino Nano R4
// Fixes: override logic, loop speed, expo curve, inertia tuning
// Serial = USB (debug/plot) | Serial1 = D0/D1 (X.BUS)
//
// Pin assignments:
//   D2  <- RC CH1 Left motor     [attachInterrupt]
//   D3  <- RC CH4 Control mode   [attachInterrupt]
//   D4  <- RC CH2 Right motor    [pulseIn, alternating]
//   D7  <- RC CH5 Override       [pulseIn, alternating]
//   A0  <- Joystick Y Throttle   [14-bit ADC, double-read]
//   A1  <- Joystick X Steering   [14-bit ADC, double-read]
//   D9  -> Left ESC              [Servo PWM]
//   D10 -> Right ESC             [Servo PWM]
//   D0  <- X.BUS telemetry       [Serial1 RX, passive capture]
//
// Override switch (CH5, 3-position):
//   LOW  (<1400): RC ONLY — Jason has full control, joystick ignored
//   MID  (1400-1600): RC PRIORITY — RC overrides when sticks moved, else joystick
//   HIGH (>1600): JOYSTICK ONLY — Malaki drives, RC ignored (except failsafe)

#include <Servo.h>

// --- Pins ---
const uint8_t PIN_RC_CH1  = 2;   // Left motor — interrupt
const uint8_t PIN_RC_CH4  = 3;   // Control mode — interrupt
const uint8_t PIN_RC_CH2  = 4;   // Right motor — pulseIn
const uint8_t PIN_RC_CH5  = 7;   // Override switch — pulseIn
const uint8_t PIN_JOY_Y   = A0;
const uint8_t PIN_JOY_X   = A1;
const uint8_t PIN_ESC_L   = 9;
const uint8_t PIN_ESC_R   = 10;

// --- Constants ---
const int ADC_MAX = 16383, ADC_CENTER = 8192;
const int JOY_DEADBAND = 480, RC_DEADBAND = 50;
const int SVC = 1500, SVMIN = 1000, SVMAX = 2000;

// Override switch thresholds (adjusted for actual switch range ~1334-1670)
const int OVR_LO = 1400;   // Below = RC only
const int OVR_HI = 1600;   // Above = Joystick only

// Expo: pow(1.8) — more responsive than 2.5, still curved
// SOFT_RANGE: 400us — allows 80% of servo range (was 250 = 50%)
const float EXPO_POWER = 1.8f;
const float SOFT_RANGE = 400.0f;

// Inertia: tuned for ~40Hz loop (one pulseIn per loop = ~25ms)
const float VMASS = 1.5f;    // Lighter = faster response (was 3.0)
const float FRIC  = 5.0f;    // Less damping = smoother (was 8.0)
const float RESP  = 30.0f;   // Faster tracking (was 20.0)

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

// --- RC via alternating pulseIn (D4=CH2, D7=CH5) ---
// Read ONE channel per loop iteration (~25ms each).
// Alternating gives ~40Hz loop rate and ~20Hz per channel.
// Avoids the swapping bug from back-to-back reads.
const unsigned long PULSE_TIMEOUT = 25000;
int ch2_pw = 1500, ch5_pw = 1500;
unsigned long ch2_time = 0, ch5_time = 0;
bool readCh2Next = true;  // Alternates between CH2 and CH5

void readOnePolledRC() {
  unsigned long p;
  if (readCh2Next) {
    p = pulseIn(PIN_RC_CH2, HIGH, PULSE_TIMEOUT);
    if (p >= 800 && p <= 2200) { ch2_pw = p; ch2_time = micros(); }
  } else {
    p = pulseIn(PIN_RC_CH5, HIGH, PULSE_TIMEOUT);
    if (p >= 800 && p <= 2200) { ch5_pw = p; ch5_time = micros(); }
  }
  readCh2Next = !readCh2Next;
}

// --- X.BUS passive capture on Serial1 (D0/D1) ---
int xbtotal = 0;
bool xlocked = false;
int xbi = 0;
unsigned long xstart = 0;
const long xbauds[] = {115200, 250000, 100000, 19200, 57600, 38400, 9600};
const int XBAUD_CNT = 7;

void xbusInit() {
  xbi = 0; xlocked = false;
  Serial1.begin(xbauds[0]);
  xstart = millis(); xbtotal = 0;
}
void xbusUpdate() {
  while (Serial1.available()) { Serial1.read(); xbtotal++; }
  if (!xlocked && millis() - xstart >= 3000) {
    if (xbtotal >= 10) {
      xlocked = true;
      char msg[60];
      sprintf(msg, "# XBUS LOCKED at %ld baud (%d bytes)", xbauds[xbi], xbtotal);
      Serial.println(msg);
    } else {
      xbi++;
      if (xbi >= XBAUD_CNT) xbi = 0;
      Serial1.end();
      Serial1.begin(xbauds[xbi]);
      xstart = millis();
      xbtotal = 0;
    }
  }
}

// --- Inertia state ---
float velL = 0, velR = 0, posL = 0, posR = 0;
unsigned long prevUs = 0, prevPrint = 0;

// --- Math helpers ---

// Attempt pow(|x|, 1.8) ≈ x * x^0.8
// x^0.8 = x / x^0.2.  Approximate x^0.2 via Newton: x^0.2 ≈ exp(0.2*ln(x))
// Simpler: use x^2 * x^(-0.2) but that's complex.
// Best approach for embedded: just use x^2 as a good-enough expo.
// pow(1.8) is between linear and squared. Use weighted blend:
//   result = 0.2*a + 0.8*a^2  (approximates pow(a, ~1.8) well)
float expoCurve(float x) {
  float a = fabsf(x);
  return 0.2f * a + 0.8f * a * a;  // ≈ pow(a, 1.8)
}

float fastTanh(float x) {
  float a = fabsf(x);
  return constrain(x / (1.0f + a + 0.28f * a * a), -1.0f, 1.0f);
}

int deadRC(int v) { return (abs(v - SVC) <= RC_DEADBAND) ? SVC : v; }
int deadJoy(int v) { return (abs(v - ADC_CENTER) <= JOY_DEADBAND) ? ADC_CENTER : v; }

int joyToServo(int adc) {
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
  Serial.println("# === Digger Control V2.5 — Nano R4 ===");
  Serial.println("# Override: CH5 LOW=RC | MID=RC priority | HIGH=Joystick");
  Serial.println("# Expo 1.8 | Inertia tuned for 40Hz | XBUS passive");
  Serial.println("# CSV: RC1,RC2,RC4,RC5,JoyY,JoyX,OutL,OutR,XbusB");

  analogReadResolution(14);

  // RC inputs
  pinMode(PIN_RC_CH1, INPUT);
  pinMode(PIN_RC_CH4, INPUT);
  pinMode(PIN_RC_CH2, INPUT);
  pinMode(PIN_RC_CH5, INPUT);

  // Interrupts only on D2 and D3
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH1), isr_ch1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH4), isr_ch4, CHANGE);

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

  // Read ONE polled channel per loop (alternating CH2/CH5, ~25ms each)
  readOnePolledRC();

  // X.BUS passive capture
  xbusUpdate();

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

  // RC stick values (with deadband)
  int sL = lost ? SVC : deadRC(rcL);
  int sR = lost ? SVC : deadRC(rcR);

  // Joystick → expo → tank mix
  int jT = joyToServo(deadJoy(jy));
  int jS = joyToServo(deadJoy(jx));
  int jo = jS - SVC;  // Steering offset
  int jL = constrain(jT + jo, SVMIN, SVMAX);
  int jR = constrain(jT - jo, SVMIN, SVMAX);

  // --- Override switch (CH5): who drives? ---
  int left, right;
  if (rcO < OVR_LO) {
    // LOW: RC ONLY — Jason has full control
    left = sL;
    right = sR;
  } else if (rcO > OVR_HI) {
    // HIGH: JOYSTICK ONLY — Malaki drives
    // (failsafe still applies: if RC lost AND joystick centered → neutral)
    left = jL;
    right = jR;
  } else {
    // MID: RC PRIORITY — RC wins when sticks moved, else joystick
    bool rcActive = (sL != SVC) || (sR != SVC);
    if (rcActive && !lost) {
      left = sL;
      right = sR;
    } else {
      left = jL;
      right = jR;
    }
  }

  // Soft power limit (tanh saturation at SOFT_RANGE)
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
    xbtotal = 0;
  }
}
