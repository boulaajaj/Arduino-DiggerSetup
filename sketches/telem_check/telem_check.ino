// ═══════════════════════════════════════════════════════════════
// X.BUS Telemetry Check — READ-ONLY bench tool (issue #36)
// ═══════════════════════════════════════════════════════════════
//
// PURPOSE: Confirm X.BUS telemetry comes through from BOTH GL10 ESCs on
// the post-migration circuit. Proven 0x50 framing (confirmed on GL10
// 2026-05-23), but throttle is HARD-WIRED to 0 — this sketch NEVER drives
// the motors. No joystick, no RPM loop, no step test. Telemetry only.
//
// Board: Arduino UNO R4 WiFi   FQBN: arduino:renesas_uno:unor4wifi   Port: COM7
// X.BUS on Serial1 (D0 RX / D1 TX), 115200 8N1, non-inverted, bus pull-up.
//   D1 (TX) --[1K]--+-- both ESC X.BUS yellow wires
//   D0 (RX) --------+
//   ESC brown -> GND (common),  ESC red (BEC) -> NOT CONNECTED
//
// NOTE: 0x50 puts the ESC in BUS_MODE. Safe here because we only ever send
// throttle = 0. The real telemetry feature in rc_test will use Read Register
// (0x10) so it can coexist with PWM driving — see issue #36.

const uint8_t  NUM_ESCS        = 2;
const uint8_t  HEADER_MASTER   = 0x0F;
const uint8_t  HEADER_SLAVE    = 0xF0;
const uint8_t  ADDR_BROADCAST  = 0xFF;
const uint8_t  FUNC_THROTTLE   = 0x50;
const uint8_t  FUNC_RESTART    = 0xA0;
const uint8_t  TELEM_DATA_LEN  = 17;   // Length field reported by ESC
const uint8_t  TELEM_FRAME_LEN = 22;   // total on-wire bytes
const uint32_t TELEM_TIMEOUT_US = 5000;
const uint32_t POLL_PERIOD_MS   = 50;  // 20 Hz poll, alternating ESCs
const uint32_t PRINT_PERIOD_MS  = 500; // 2 Hz print

struct ESCTelemetry {
  int16_t  rpmHz, busCurrent, phaseCurrent;
  uint16_t status;
  int16_t  receivedThrottle, outputThrottle, busVoltage, escTempRaw;
  bool     valid;
  uint32_t timestamp;
};
ESCTelemetry telem[NUM_ESCS];

uint8_t  txBuf[40], rxBuf[40];
int      rxLen = 0;
uint8_t  telemTarget = 0;
uint32_t lastPollMs = 0, lastPrintMs = 0;
uint32_t totalPolls = 0, goodFrames = 0, badFrames = 0;
uint32_t lastBusModeMs = 0, lastRestartMs = 0;
bool     firstGood = true;

uint8_t checksumFull(const uint8_t *f, int len) {
  uint8_t s = 0; for (int i = 1; i < len - 1; i++) s += f[i]; return s;
}
uint8_t checksumNarrow(uint8_t sadr, uint8_t extra) { return (sadr + extra) & 0xFF; }

// Send broadcast throttle (ZERO for both ESCs) + request telemetry from target
void sendZeroThrottle(uint8_t target) {
  int dataLen = NUM_ESCS * 2;
  int frameLen = 6 + dataLen;
  txBuf[0] = HEADER_MASTER;
  txBuf[1] = ADDR_BROADCAST;
  txBuf[2] = target;            // telemetry responder
  txBuf[3] = FUNC_THROTTLE;
  txBuf[4] = dataLen;
  for (uint8_t i = 0; i < NUM_ESCS; i++) { txBuf[5 + i*2] = 0x00; txBuf[6 + i*2] = 0x00; } // throttle 0
  txBuf[frameLen - 1] = checksumFull(txBuf, frameLen);
  while (Serial1.available()) Serial1.read();
  Serial1.write(txBuf, frameLen);
  Serial1.flush();
}

void sendRestart(uint8_t target) {
  txBuf[0] = HEADER_MASTER; txBuf[1] = ADDR_BROADCAST; txBuf[2] = target;
  txBuf[3] = FUNC_RESTART;   txBuf[4] = 0; txBuf[5] = checksumFull(txBuf, 6);
  Serial1.write(txBuf, 6); Serial1.flush();
}

bool receiveTelemetry(uint8_t esc) {
  uint32_t start = micros(); rxLen = 0;
  while ((micros() - start) < TELEM_TIMEOUT_US) {
    while (Serial1.available() && rxLen < (int)sizeof(rxBuf)) rxBuf[rxLen++] = Serial1.read();
    for (int i = 0; i + TELEM_FRAME_LEN <= rxLen; i++)
      if (rxBuf[i]==HEADER_SLAVE && rxBuf[i+1]==ADDR_BROADCAST && rxBuf[i+3]==FUNC_THROTTLE) { start = 0; break; }
    if (start == 0) break;
  }
  int fs = -1;
  for (int i = 0; i + TELEM_FRAME_LEN <= rxLen; i++)
    if (rxBuf[i]==HEADER_SLAVE && rxBuf[i+1]==ADDR_BROADCAST && rxBuf[i+3]==FUNC_THROTTLE) { fs = i; break; }
  if (fs < 0 || (rxLen - fs) < TELEM_FRAME_LEN) return false;
  uint8_t *f = &rxBuf[fs];
  if (f[4] != TELEM_DATA_LEN) return false;
  uint8_t sum = checksumFull(f, TELEM_FRAME_LEN);
  if (f[TELEM_FRAME_LEN-1] != sum && f[TELEM_FRAME_LEN-1] != checksumNarrow(f[1], f[2])) return false;
  const uint8_t *d = &f[5];
  ESCTelemetry *t = &telem[esc];
  t->rpmHz            = (int16_t)(d[0]  | (d[1]  << 8));
  t->busCurrent       = (int16_t)(d[2]  | (d[3]  << 8));
  t->phaseCurrent     = (int16_t)(d[4]  | (d[5]  << 8));
  t->status           = (uint16_t)(d[6] | (d[7]  << 8));
  t->receivedThrottle = (int16_t)(d[8]  | (d[9]  << 8));
  t->outputThrottle   = (int16_t)(d[10] | (d[11] << 8));
  t->busVoltage       = (int16_t)(d[12] | (d[13] << 8));
  t->escTempRaw       = (int16_t)(d[14] | (d[15] << 8));
  t->valid = true; t->timestamp = micros();
  return true;
}

void printRawRX() {
  if (rxLen == 0) { Serial.println("# RAW RX: (nothing)"); return; }
  Serial.print("# RAW RX ("); Serial.print(rxLen); Serial.print("B): ");
  for (int i = 0; i < min(rxLen, 32); i++) { char h[4]; snprintf(h, sizeof(h), "%02X ", rxBuf[i]); Serial.print(h); }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  uint32_t w = millis(); while (!Serial && (millis() - w) < 2000) {}
  Serial.println();
  Serial.println("# X.BUS Telemetry Check (READ-ONLY, throttle hard-wired 0) — issue #36");
  Serial.println("# UNO R4 WiFi, Serial1 D0/D1 @115200. Reading BOTH ESCs (0 and 1).");
  for (uint8_t i = 0; i < NUM_ESCS; i++) telem[i].valid = false;
  delay(500);
  Serial.println("# Restart broadcast (0xA0) to activate BUS_MODE...");
  while (Serial1.available()) Serial1.read();
  sendRestart(0xFF);
  delay(200);
  lastPollMs = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastPollMs >= POLL_PERIOD_MS) {
    lastPollMs = now; totalPolls++;
    sendZeroThrottle(telemTarget);
    if (receiveTelemetry(telemTarget)) {
      goodFrames++;
      if (firstGood) { Serial.println("# >>> FIRST TELEMETRY RECEIVED — link is working <<<"); firstGood = false; }
      if (telem[telemTarget].status & (1 << 3)) lastBusModeMs = now;  // BUS_MODE bit
    } else {
      badFrames++;
      if (badFrames <= 6) printRawRX();
    }
    telemTarget = (telemTarget + 1) % NUM_ESCS;
    if ((now - lastBusModeMs) > 2000 && (now - lastRestartMs) > 3000) {
      Serial.println("# BUS_MODE lost — re-broadcasting Restart...");
      sendRestart(0xFF); lastRestartMs = now;
    }
  }

  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;
    Serial.print("# Polls="); Serial.print(totalPolls);
    Serial.print(" Good="); Serial.print(goodFrames);
    Serial.print(" Bad="); Serial.println(badFrames);
    for (uint8_t i = 0; i < NUM_ESCS; i++) {
      ESCTelemetry *t = &telem[i];
      Serial.print("#  ESC"); Serial.print(i); Serial.print(": ");
      if (!t->valid) { Serial.println("no data yet"); continue; }
      Serial.print("V="); Serial.print(t->busVoltage * 0.1f, 1);
      Serial.print(" I_bus="); Serial.print(t->busCurrent * 0.1f, 1);
      Serial.print("A RPM="); Serial.print(t->rpmHz * 30);
      Serial.print(" T_esc="); Serial.print(t->escTempRaw - 40);
      Serial.print("C Thr_out="); Serial.print(t->outputThrottle * 0.1f, 1);
      Serial.print("% age="); Serial.print((micros() - t->timestamp) / 1000); Serial.print("ms");
      if (t->status & (1 << 3))  Serial.print(" BUS_MODE");
      if (t->status & (1 << 11)) Serial.print(" CAP_CHARGED");
      if (t->status & (1 << 1))  Serial.print(" UNDERVOLT");
      Serial.println();
    }
  }
}
