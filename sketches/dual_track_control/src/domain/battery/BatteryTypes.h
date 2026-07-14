// domain/battery — shared value types (#117 step 1).
// Pure domain: no Arduino includes (host-compilable by design).
#pragma once

// One ESC's report of its pack voltage, as delivered by X.BUS telemetry.
// `valid` mirrors EscTelem.valid — the freshness watchdog's verdict, NOT a
// plausibility judgement (that is VoltagePlausibility's job).
struct BatteryReading {
  float voltage;  // volts (EMA-smoothed upstream)
  bool valid;     // telemetry fresh within the per-ESC watchdog window
};
