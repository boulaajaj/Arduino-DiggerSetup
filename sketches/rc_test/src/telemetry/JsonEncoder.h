// telemetry — JSON encoder for the dashboard (#132). The exact SSE / /data
// payload moved VERBATIM from the .ino's buildTelemJson(). Compact keys and
// integer values are a deliberate bandwidth decision on the SSE frame budget
// (naming rules, exception 4); the dashboard JS divides by 10 on its side.
// Pure formatting: reads only the snapshot.
#pragma once

#include <stddef.h>

struct SystemSnapshot;

// Encode one telemetry frame into body (capacity bytes). Returns the body
// length, clamped so callers never read past the buffer. `sequence` is the
// caller-owned frame counter ([WIFI]'s wifiSeq).
int encodeTelemetryJson(char *body, size_t capacity,
                        const SystemSnapshot &snapshot, unsigned long sequence);
