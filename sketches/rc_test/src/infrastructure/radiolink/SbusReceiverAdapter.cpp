// infrastructure/radiolink — S.BUS receiver adapter (#117 step 10 slice 3,
// #176). The ONE implementation of ports/RcInputPort.h. Owns the second
// hardware UART and the Bolder Flight Systems S.BUS parser (moved from
// rc_test.ino) — the inverted-UART detail lives here.
//
// UNO R4 WiFi: SCI0 is on D11/D12 (the Nano R4 used A4/A5). S.BUS only
// needs RX; TX is unused. The UART is named sbusUart (not Serial2) to avoid
// the macro collision with the core's pre-declared _UART2_. S.BUS idles
// electrically inverted; the NPN inverter on D12 restores standard UART
// polarity (wiring: docs/WIRING-GUIDE-V8.md). Behavior is locked end-to-end
// by the characterization suites via the stub S.BUS frame queue.
#include "../../ports/RcInputPort.h"

#include <Arduino.h>

#include "sbus.h"

static const uint8_t PIN_SBUS_TX = 11;  // SCI0 TX (D11) — unused, RX-only
static const uint8_t PIN_SBUS_RX = 12;  // SCI0 RX (D12) — inverted S.BUS in

UART sbusUart(PIN_SBUS_TX, PIN_SBUS_RX);

// External linkage: the test harness scripts frames through the stub sbus.h
// queue; the object itself is reachable for completeness.
bfs::SbusRx sbusRx(&sbusUart);

void rcInputInitialize() {
  sbusRx.Begin();
}

// The copy is bounded by the PORT's channel count; the vendor frame must
// carry at least that many or this fails to compile (never a silent
// out-of-bounds read/write if the library's channel count ever changes).
static_assert(bfs::SbusData::NUM_CH >= RcFrame::CHANNEL_COUNT,
              "vendor S.BUS frame carries fewer channels than RcFrame");

bool rcInputReadFrame(RcFrame* out) {
  if (!sbusRx.Read()) return false;
  bfs::SbusData data = sbusRx.data();
  for (int i = 0; i < RcFrame::CHANNEL_COUNT; i++) {
    out->channels[i] = data.ch[i];
  }
  out->failsafe = data.failsafe;
  out->lostFrame = data.lost_frame;
  return true;
}
