// Servo.h — HOST TEST STUB. Captures every attach/detach/writeMicroseconds
// per pin, with the fake-clock timestamp, so tests can assert on the exact
// PWM the ESCs would have seen (including the output-gate ease-out ramp).
#pragma once

#include <cstdint>
#include <map>
#include <utility>
#include <vector>

namespace stub {

struct ServoLog {
  bool attached = false;
  int  attachCount = 0;
  int  detachCount = 0;
  int  lastMicroseconds = -1;              // -1 = never written
  int  writesWhileDetached = 0;            // firmware should never do this
  std::vector<std::pair<uint32_t, int>> writes;  // (millis, microseconds)
};

inline std::map<int, ServoLog> servoLogs;

inline ServoLog& servoOnPin(int pin) { return servoLogs[pin]; }

inline void resetServos() { servoLogs.clear(); }

}  // namespace stub

class Servo {
 public:
  uint8_t attach(int pin) {
    pin_ = pin;
    stub::ServoLog& log = stub::servoOnPin(pin);
    log.attached = true;
    log.attachCount++;
    return 1;
  }
  void detach() {
    if (pin_ < 0) return;
    stub::ServoLog& log = stub::servoOnPin(pin_);
    log.attached = false;
    log.detachCount++;
  }
  bool attached() { return pin_ >= 0 && stub::servoOnPin(pin_).attached; }
  void writeMicroseconds(int microseconds) {
    if (pin_ < 0) return;
    stub::ServoLog& log = stub::servoOnPin(pin_);
    if (!log.attached) {
      log.writesWhileDetached++;
      return;
    }
    log.lastMicroseconds = microseconds;
    log.writes.push_back({millis(), microseconds});
  }

 private:
  int pin_ = -1;
};
