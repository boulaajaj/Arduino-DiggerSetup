// sbus.h — HOST TEST STUB for the Bolder Flight Systems SBUS library
// (bfs::SbusRx / bfs::SbusData, API of v8.x). Tests queue whole frames;
// each SbusRx::Read() consumes one, exactly like a frame arriving on the
// wire. No UART decoding is simulated — S.BUS electrical decode is a
// hardware-in-the-loop concern (see docs/TESTING.md).
#pragma once

#include <cstdint>
#include <deque>

#include "Arduino.h"

namespace bfs {

struct SbusData {
  bool lost_frame = false;
  bool failsafe = false;
  bool ch17 = false;
  bool ch18 = false;
  static constexpr int8_t NUM_CH = 16;
  int16_t ch[NUM_CH] = {};
};

}  // namespace bfs

namespace stub {

const int16_t SBUS_RAW_MIN = 172;     // matches firmware SBUS_MIN
const int16_t SBUS_RAW_CENTER = 992;  // transmitter center
const int16_t SBUS_RAW_MAX = 1811;    // matches firmware SBUS_MAX

inline std::deque<bfs::SbusData> sbusFrameQueue;
inline bool sbusBeginCalled = false;

// A frame with every channel centered (no failsafe, no lost frame).
inline bfs::SbusData centeredSbusFrame() {
  bfs::SbusData frame;
  for (int i = 0; i < bfs::SbusData::NUM_CH; i++) frame.ch[i] = SBUS_RAW_CENTER;
  return frame;
}

inline void queueSbusFrame(const bfs::SbusData& frame) { sbusFrameQueue.push_back(frame); }

inline void resetSbus() {
  sbusFrameQueue.clear();
  sbusBeginCalled = false;
}

}  // namespace stub

namespace bfs {

class SbusRx {
 public:
  explicit SbusRx(UART* uart) : uart_(uart) {}
  void Begin() { stub::sbusBeginCalled = true; }
  bool Read() {
    if (stub::sbusFrameQueue.empty()) return false;
    data_ = stub::sbusFrameQueue.front();
    stub::sbusFrameQueue.pop_front();
    return true;
  }
  SbusData data() const { return data_; }

 private:
  UART* uart_;
  SbusData data_;
};

}  // namespace bfs
