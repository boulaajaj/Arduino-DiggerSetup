// infrastructure/network — request routing + page serving (#181). Moved
// VERBATIM from dual_track_control.ino [WIFI] wifiUpdate()/wifiSendData()/
// wifiBeginPage()/wifiSend304(); the JSON body comes through the
// TelemetryFrameSource port. The #69 non-starvation invariants live here:
// at most ONE modem write per pass, each modem call capped at
// WIFI_MODEM_TIMEOUT_MS (vs the WiFiS3 default 10 000 ms) so a single
// stalled TCP window can't freeze loop() for seconds (issue #54).
#include "DashboardServer.h"

#include <stdio.h>
#include <string.h>

#include <Arduino.h>

#include "../../ports/DashboardServicePort.h"
#include "../../ports/TelemetryFrameSource.h"
#include "SseStream.h"
#include "WifiService.h"

// One-shot /data JSON. Header + body coalesced into a single write() so the
// whole response is one modem round-trip (one bounded op this loop pass).
static void sendDataResponse(WiFiClient &client) {
  char body[400];
  int n = telemetryFrameBuild(body, sizeof(body));
  char buf[560];
  int h = snprintf(buf, sizeof(buf),
    "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
    "Access-Control-Allow-Origin: *\r\nConnection: close\r\nContent-Length: %d\r\n\r\n", n);
  if (h > 0 && (size_t)(h + n) < sizeof(buf)) {
    memcpy(buf + h, body, n);
    client.write(reinterpret_cast<const uint8_t *>(buf), h + n);
  }
}

// Serve the embedded dashboard. The page is static, so it carries an ETag and
// Cache-Control: no-cache (store-but-revalidate): the first visit downloads it
// once, then every refresh sends If-None-Match and gets a tiny 304 here instead
// of re-streaming ~33 KB through the blocking Wi-Fi modem (issue #54). Live
// values arrive separately over SSE, so the HTML itself never reloads in normal
// use.
//
// The 200 body is NOT sent here in one burst — that single blocking burst was
// the #69 runaway root cause (it froze loop() for ~1-2 s while the ESC held the
// last throttle). Instead beginPageResponse() sends only the headers (one
// coalesced write) and hands the body to the incremental sender in
// dashboardServiceUpdate(), which ships ONE WIFI_PAGE_CHUNK per loop pass so
// control + failsafe run between chunks.
static void beginPageResponse(WiFiClient &client) {
  size_t len = strlen(dashboardPageHtml);
  char hdr[160];
  int h = snprintf(hdr, sizeof(hdr),
    "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
    "Cache-Control: no-cache\r\nETag: %s\r\nConnection: close\r\nContent-Length: %u\r\n\r\n",
    pageEtag, (unsigned)len);
  // snprintf returns <0 on error or >= size if truncated; only proceed with a
  // valid, fully-formed header. Otherwise abort (don't start a body transfer
  // behind a partial/garbage header).
  if (h <= 0 || h >= (int)sizeof(hdr)) {
    client.stop();
    return;
  }
  client.write(reinterpret_cast<const uint8_t *>(hdr), h);
  pageClient = client;          // refcounted handle survives the local going out of scope
  pagePointer = dashboardPageHtml;
  pageRemaining = len;          // body streams over the next passes (branch A)
}

// 304 Not Modified for a cached dashboard — tiny, single coalesced write.
static void send304Response(WiFiClient &client) {
  char hdr[120];
  int h = snprintf(hdr, sizeof(hdr),
    "HTTP/1.1 304 Not Modified\r\nETag: %s\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n",
    pageEtag);
  // Guard the snprintf return (could be <0 or truncated) before using it as a length.
  if (h > 0 && h < (int)sizeof(hdr)) {
    client.write(reinterpret_cast<const uint8_t *>(hdr), h);
  }
}

// Non-blocking, and bounded to AT MOST ONE modem write per loop pass (#69):
// either one page chunk, OR one request/response, OR one SSE frame. The control
// path + WDT.refresh() run every pass between these, so Wi-Fi can never starve
// the loop.
void dashboardServiceUpdate() {
  if (!wifiUp) return;

  modem.timeout(WIFI_MODEM_TIMEOUT_MS);

  // (A) A page transfer is in flight → send exactly ONE chunk and yield. Control
  // + failsafe + watchdog refresh run before we get back here next pass.
  if (pageRemaining > 0) {
    if (!pageClient.connected()) {
      // client went away mid-transfer
      pageClient.stop();
      pagePointer = nullptr;
      pageRemaining = 0;
    } else {
      size_t chunk = pageRemaining > WIFI_PAGE_CHUNK ? WIFI_PAGE_CHUNK : pageRemaining;
      size_t w = pageClient.write(reinterpret_cast<const uint8_t *>(pagePointer), chunk);
      if (w == 0) {
        // write stalled (modem timeout) — abort the transfer instead of spinning
        // forever: a 0-byte write makes no progress, so without this the page
        // never completes and SSE telemetry is starved one pass at a time.
        pageClient.stop();
        pagePointer = nullptr;
        pageRemaining = 0;
      } else {
        pagePointer += w;
        pageRemaining -= w;
        if (pageRemaining == 0) {
          pageClient.flush();
          pageClient.stop();
          pagePointer = nullptr;
        }
      }
    }
    modem.timeout(MODEM_TIMEOUT);
    return;
  }

  // (B) Otherwise accept at most one new client this pass.
  WiFiClient client = wifiServer.available();
  if (client) {
    // Read the request line + headers into one bounded buffer (cap + 25 ms) so
    // we can route on the first line AND honor a conditional-GET If-None-Match
    // for the cached dashboard. 512 B comfortably holds an iOS Safari header set.
    char req[512];
    int  ri = 0;
    uint32_t t0 = millis();
    while ((millis() - t0) < 25 && ri < (int)sizeof(req) - 1) {
      while (client.available() && ri < (int)sizeof(req) - 1) req[ri++] = client.read();
      if (ri >= 4 && req[ri - 4] == '\r' && req[ri - 3] == '\n' &&
                     req[ri - 2] == '\r' && req[ri - 1] == '\n') break;  // end of headers
    }
    req[ri] = '\0';

    if (strstr(req, "/events")) {
      sseAccept(client);
    } else if (strstr(req, "/data")) {
      sendDataResponse(client); client.flush(); client.stop();
    } else {
      // Static dashboard. If the browser already holds our ETag, reply 304 and
      // skip the ~33 KB transfer entirely (issue #54); otherwise send the headers
      // now and stream the body incrementally over the next passes (branch A).
      bool cached = strstr(req, "If-None-Match") && strstr(req, pageEtag);
      if (cached) {
        send304Response(client);
        client.flush();
        client.stop();
      } else {
        beginPageResponse(client);  // body streams over the next passes (branch A)
      }
    }
    modem.timeout(MODEM_TIMEOUT);
    return;                                            // one modem op done this pass
  }

  // (C) Idle → push at most one SSE telemetry frame (reaping dead sockets, #77).
  ssePush();

  modem.timeout(MODEM_TIMEOUT);   // restore default 10 s timeout on the way out
}
