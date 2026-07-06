// SerialPortStub.h — HOST TEST STUB serial ports (included by stubs/Arduino.h).
// print()/println() (human debug text) are discarded; write() (binary
// protocol bytes, e.g. X.BUS requests) is captured for assertions; read() is
// fed from a per-port scripted RX queue.
#pragma once

#include <cstdint>
#include <deque>
#include <vector>

class SerialPortStubBase {
 public:
  void begin(unsigned long baud) { baudRate = baud; beginCalled = true; }
  void end() { beginCalled = false; }
  explicit operator bool() const { return true; }

  int available() { return (int)rxQueue.size(); }
  int read() {
    if (rxQueue.empty()) return -1;
    uint8_t b = rxQueue.front();
    rxQueue.pop_front();
    return b;
  }
  size_t write(const uint8_t* buffer, size_t length) {
    txCapture.insert(txCapture.end(), buffer, buffer + length);
    return length;
  }
  size_t write(uint8_t b) { txCapture.push_back(b); return 1; }

  // Debug text output — swallowed (host tests assert on state, not prints).
  template <typename T> size_t print(const T&) { return 0; }
  template <typename T> size_t println(const T&) { return 0; }
  size_t println() { return 0; }

  // Test control surface.
  void scriptRxBytes(const uint8_t* bytes, size_t length) {
    rxQueue.insert(rxQueue.end(), bytes, bytes + length);
  }
  void resetPort() {
    rxQueue.clear();
    txCapture.clear();
    beginCalled = false;
    baudRate = 0;
  }

  std::deque<uint8_t> rxQueue;
  std::vector<uint8_t> txCapture;
  bool beginCalled = false;
  unsigned long baudRate = 0;
};

class HardwareSerialStub : public SerialPortStubBase {};

// Renesas-core UART class constructed on explicit TX/RX pins (the firmware's
// sbusUart on D11/D12).
class UART : public SerialPortStubBase {
 public:
  UART(uint8_t txPin, uint8_t rxPin) : txPin(txPin), rxPin(rxPin) {}
  uint8_t txPin;
  uint8_t rxPin;
};

inline HardwareSerialStub Serial;   // USB CDC debug
inline HardwareSerialStub Serial1;  // X.BUS telemetry bus

namespace stub {
inline void resetSerialPorts() {
  Serial.resetPort();
  Serial1.resetPort();
}
}  // namespace stub
