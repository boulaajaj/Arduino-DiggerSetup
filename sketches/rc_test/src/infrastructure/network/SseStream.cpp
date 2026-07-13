// infrastructure/network — SSE upgrade + frame push (#181). Moved VERBATIM
// from rc_test.ino [WIFI]; the JSON body comes through the
// TelemetryFrameSource port (the application encodes, this layer ships
// bytes).
#include "SseStream.h"

#include <stdio.h>

#include <Arduino.h>

#include "../../ports/TelemetryFrameSource.h"
#include "WifiService.h"

void sseAccept(WiFiClient& client) {
  // Upgrade to a persistent Server-Sent Events stream. ALWAYS free any
  // previous SSE socket first — even a dead/half-open one — so its ESP32
  // link id is released (AT+CIPCLOSE) instead of leaking. The old code only
  // closed it when still connected(), so a client that dropped Wi-Fi without
  // a clean close leaked a link id every reconnect until the ~5-socket pool
  // was exhausted and the server could accept nothing (#77).
  sseClient.stop();
  sseClient = client;
  sseActive = true;
  sseClient.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n"
                    "Cache-Control: no-cache\r\nAccess-Control-Allow-Origin: *\r\n\r\n"));
  sseLastMs = 0;                                     // push first frame immediately
}

// Push at most one SSE telemetry frame, AND proactively reap the socket the
// instant it dies so a dropped client can't leak its ESP32 link id and
// exhaust the ~5-socket pool (#77). The push starts with a ": hb\n" comment
// (a no-op for EventSource) that exercises the TCP socket so Safari/iOS
// doesn't park the connection in a stalled state.
void ssePush() {
  if (!sseActive) return;
  if (!sseClient.connected()) {
    sseClient.stop();            // peer gone (e.g. Wi-Fi dropped) → free the link id NOW
    sseActive = false;
    return;
  }
  uint32_t now = millis();
  if (now - sseLastMs < SSE_INTERVAL_MS) return;
  sseLastMs = now;
  // Build the whole SSE frame — heartbeat comment, data line, terminator —
  // into one buffer and ship it in a SINGLE write() (one AT round-trip vs
  // three, issue #54). SSE_FRAME_CAP holds the 11-byte prefix + JSON + "\n\n".
  char frame[SSE_FRAME_CAP];
  int len = snprintf(frame, sizeof(frame), ": hb\ndata: ");
  // Guard the snprintf return before using it as an offset (consistent with
  // the page/304 header sends). The fixed 11-byte prefix can't really
  // truncate, but this keeps the frame+len / cap math provably in-bounds.
  if (len > 0 && len < (int)sizeof(frame) - 2) {
    // Reserve the last 2 bytes for the "\n\n" terminator so the JSON body
    // can never crowd it out; the frame builder clamps to the cap we pass.
    len += telemetryFrameBuild(frame + len, sizeof(frame) - len - 2);
    frame[len++] = '\n';
    frame[len++] = '\n';
    size_t w = sseClient.write(reinterpret_cast<const uint8_t *>(frame), len);
    if (w == 0) {                // 0-byte write = dead socket → reap immediately (#77)
      sseClient.stop();
      sseActive = false;
    }
  }
}
