// WiFiS3.h — HOST TEST STUB. The dashboard is monitoring-only (permanent
// rule), so characterization needs Wi-Fi merely to be inert: the AP "comes
// up", no clients ever connect, every write is swallowed. wifiUpdate() then
// runs its no-client path without touching control behavior.
#pragma once

#include <cstdint>
#include <cstring>

#include "Arduino.h"

#define WL_NO_MODULE     255
#define WL_IDLE_STATUS   0
#define WL_AP_LISTENING  7
#define MODEM_TIMEOUT    10000

namespace stub {
inline uint8_t  wifiStatusValue = WL_IDLE_STATUS;   // != WL_NO_MODULE → module present
inline uint8_t  wifiBeginApResult = WL_AP_LISTENING;
inline uint32_t modemTimeoutSetCount = 0;
inline uint32_t modemLastTimeoutMs = 0;
}  // namespace stub

class WiFiClient {
 public:
  bool connected() { return false; }
  int  available() { return 0; }
  int  read() { return -1; }
  size_t write(const uint8_t*, size_t length) { return length; }
  size_t print(const char* text) { return strlen(text); }
  void flush() {}
  void stop() {}
  explicit operator bool() const { return false; }
};

class WiFiServer {
 public:
  explicit WiFiServer(int) {}
  void begin() {}
  WiFiClient available() { return WiFiClient(); }
};

class WiFiClass {
 public:
  uint8_t status() { return stub::wifiStatusValue; }
  const char* firmwareVersion() { return "host-stub"; }
  uint8_t beginAP(const char*, const char*, uint8_t) { return stub::wifiBeginApResult; }
  IPAddress localIP() { return IPAddress(192, 168, 4, 1); }
};

inline WiFiClass WiFi;

class ModemStub {
 public:
  void timeout(uint32_t ms) {
    stub::modemTimeoutSetCount++;
    stub::modemLastTimeoutMs = ms;
  }
};

inline ModemStub modem;

namespace stub {
inline void resetWiFi() {
  wifiStatusValue = WL_IDLE_STATUS;
  wifiBeginApResult = WL_AP_LISTENING;
  modemTimeoutSetCount = 0;
  modemLastTimeoutMs = 0;
}
}  // namespace stub
