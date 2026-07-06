// WDT.h — HOST TEST STUB for the RA4M1 hardware watchdog. Counts refresh()
// calls so tests can assert the firmware's "exactly one refresh per loop
// pass, after the control path" rule.
#pragma once

#include <cstdint>

class WatchdogStub {
 public:
  bool begin(uint32_t timeoutMs) {
    if (!beginResult) return false;
    armed = true;
    timeout = timeoutMs;
    return true;
  }
  uint32_t getTimeout() const { return timeout; }
  void refresh() { refreshCount++; }

  bool     beginResult = true;   // test control: simulate arm failure
  bool     armed = false;
  uint32_t timeout = 0;
  uint32_t refreshCount = 0;
};

inline WatchdogStub WDT;

namespace stub {
inline void resetWatchdog() { WDT = WatchdogStub{}; }
}  // namespace stub
