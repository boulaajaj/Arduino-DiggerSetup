// telemetry — CSV encoder for the USB debug stream (#132). The exact 10 Hz
// line moved VERBATIM from the .ino's debugPrint(); the column header lives
// with [DEBUG]'s banner print. Pure formatting: reads only the snapshot.
#pragma once

#include <stddef.h>

struct SystemSnapshot;

// Encode one debug CSV line into `line` (capacity bytes). Returns the
// snprintf result (callers historically ignore it and print the buffer).
int encodeDebugCsv(char *line, size_t capacity, const SystemSnapshot &snapshot);
