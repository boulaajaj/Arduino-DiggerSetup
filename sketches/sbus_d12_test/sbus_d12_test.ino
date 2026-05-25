// Test: verify SCI0 UART on D12 (RX) / D11 (TX) compiles for UNO R4 WiFi
// S.BUS is receive-only, but UART needs both pins declared.

UART sbusUart(11, 12);  // TX=D11 (P411, SCI0), RX=D12 (P410, SCI0)

void setup() {
  Serial.begin(115200);
  sbusUart.begin(100000, SERIAL_8E2);  // S.BUS: 100000 baud, 8E2
}

void loop() {
  if (sbusUart.available()) {
    uint8_t b = sbusUart.read();
    Serial.println(b, HEX);
  }
}
