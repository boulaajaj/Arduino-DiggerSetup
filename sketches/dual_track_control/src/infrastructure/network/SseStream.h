// infrastructure/network — the Server-Sent Events stream (#181): one
// persistent connection pushes telemetry frames, instead of the browser
// opening a fresh (slow) connection every poll. This is the big update-rate
// win on WiFiS3, and EventSource auto-reconnects after a dropout.
#pragma once

#include <WiFiS3.h>

// Upgrade an accepted client to the persistent SSE stream (the /events
// route). Always frees any previous SSE socket first (#77). The parameter
// is only copied into the adapter-owned socket handle.
void sseAccept(const WiFiClient& client);

// Push at most one telemetry frame at the SSE cadence, reaping the socket
// the instant it dies (#77). Called from the idle branch of
// dashboardServiceUpdate().
void ssePush();
