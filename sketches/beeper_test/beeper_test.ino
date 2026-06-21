// ═══════════════════════════════════════════════════════════════
// Beeper + SWD-horn bench test (issue #63)
// ═══════════════════════════════════════════════════════════════
//
// Purpose: confirm the installed beeper works and that the RC6GS SWD
// button drives it as a horn, before adding it to the flight firmware.
//
//  - Active piezo on D8 (the reserved beeper pin): digital HIGH = beep.
//  - RC6GS SWD button = S.BUS channel 7 → sbusData.ch[6] (0-indexed array,
//    same convention as gear=CH4=ch[3], override=CH5=ch[4]).
//    Beep ON when SWD at +100% (raw ~1811); OFF at center/0 or -100%.
//  - Prints S.BUS channels 0-9 at ~5 Hz so we can verify live WHICH
//    channel the SWD button actually moves (and tune the threshold).
//
// SAFE ON BENCH: this sketch does NOT init Servo or touch D9/D10, so the
// ESCs get no PWM and the motors are not driven. Keep wheels up anyway.
//
// This is a throwaway bench tool — the horn logic moves into rc_test.ino
// (a non-blocking [BEEPER] module) once confirmed.

#include "sbus.h"   // Bolder Flight Systems SBUS (same lib as the flight sketch)

const uint8_t PIN_BEEPER  = 8;     // D8 — active piezo, HIGH = beep (reserved pin)
const uint8_t PIN_SBUS_TX = 11;    // SCI0 TX (unused; S.BUS is RX-only)
const uint8_t PIN_SBUS_RX = 12;    // SCI0 RX — inverted S.BUS in (via NPN inverter)

const uint8_t SWD_CH      = 6;     // S.BUS array index for RC channel 7 (CH7 = ch[6])
const int     SWD_ON_RAW  = 1400;  // beep when channel raw > this (toward +100% ~1811)

UART          sbusUart(PIN_SBUS_TX, PIN_SBUS_RX);
bfs::SbusRx   sbusRx(&sbusUart);
bfs::SbusData sbusData;

uint32_t lastPrintMs = 0;
bool     beepOn      = false;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BEEPER, OUTPUT);
  digitalWrite(PIN_BEEPER, LOW);
  sbusRx.Begin();
}

void loop() {
  // Update the beeper on every fresh S.BUS frame (~7-14 ms).
  if (sbusRx.Read()) {
    sbusData = sbusRx.data();
    bool linkOk = !sbusData.failsafe;                    // horn off if RC link lost (safe)
    beepOn = linkOk && (sbusData.ch[SWD_CH] > SWD_ON_RAW);
    digitalWrite(PIN_BEEPER, beepOn ? HIGH : LOW);
  }

  // Diagnostic print ~5 Hz: lets us confirm which channel moves on SWD.
  if (millis() - lastPrintMs >= 200) {
    lastPrintMs = millis();
    Serial.print("# FS=");   Serial.print(sbusData.failsafe);
    Serial.print(" Lost=");  Serial.print(sbusData.lost_frame);
    Serial.print(" beep=");  Serial.print(beepOn);
    Serial.print(" | ch0-9: ");
    for (int i = 0; i < 10; i++) {
      Serial.print(sbusData.ch[i]);
      Serial.print(' ');
    }
    Serial.println();
  }
}
