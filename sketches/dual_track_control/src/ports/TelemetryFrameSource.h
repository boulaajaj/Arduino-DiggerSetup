// ports — the telemetry frame contract (#181). A REVERSE link-time seam
// (#130): declared here, IMPLEMENTED by the application (the .ino's
// buildTelemJson shim), CALLED by infrastructure/network/ whenever a
// dashboard client needs one frame. This keeps the dependency direction
// lawful — the network layer never includes application/ or telemetry/;
// it only ever handles the encoded bytes.
#pragma once

#include <stddef.h>

// Build one telemetry JSON frame into body (capacity bytes); returns its
// length (the implementation clamps so callers never read past the buffer).
int telemetryFrameBuild(char* body, size_t capacity);
