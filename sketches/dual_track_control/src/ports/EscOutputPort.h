// ports — the ESC output contract (#117 step 10 slice 1, #172).
// A link-time seam (#130): free functions, ONE linked implementation
// (infrastructure/arduino/PwmEscAdapter.cpp). Consumed by the application
// layer; domain/ never includes ports. The caller owns pulse clamping and
// any application-visible output state — this port only moves pulses.
#pragma once

// Attach both ESC outputs and remember the pins for later re-attach.
void escOutputInitialize(int leftPin, int rightPin);

// Re-attach after a cut (the OutputGate's resume-from-CUT edge).
void escOutputAttach();

// Stop pulsing entirely (the ESCs lose signal and beep).
void escOutputDetach();

// Emit one servo pulse pair, microseconds. Pulses arrive pre-clamped.
void escOutputWritePulses(int leftPulse, int rightPulse);
