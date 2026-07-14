// infrastructure/network — AP bring-up + serving state definitions (#181).
// Moved VERBATIM from dual_track_control.ino [WIFI]. MONITORING ONLY — no control
// input over Wi-Fi, ever (permanent safety rule).
#include "WifiService.h"

#include <stdio.h>

#include <Arduino.h>

#include "../../ports/DashboardServicePort.h"

WiFiServer wifiServer(80);
bool       wifiUp  = false;
// ETag for the static dashboard. Filled at initialize from a content hash,
// so browsers can cache the HTML and a refresh returns 304 instead of
// re-downloading ~33 KB through the blocking Wi-Fi modem (issue #54).
char       pageEtag[24] = "\"d0\"";
const char *dashboardPageHtml = nullptr;

WiFiClient sseClient;
uint32_t   sseLastMs = 0;
bool       sseActive = false;

WiFiClient pageClient;
const char *pagePointer = nullptr;
size_t     pageRemaining = 0;

int dashboardServiceRadioStatus() { return WiFi.status(); }

bool dashboardServiceInitialize(const char* ssid, const char* password,
                                uint8_t apChannel, const char* pageHtml) {
  dashboardPageHtml = pageHtml;
  if (WiFi.status() == WL_NO_MODULE) {
    if (Serial) Serial.println("# WiFi: NO MODULE — ESP32-S3 radio not responding");
    return false;
  }
  if (Serial) {
    Serial.print("# WiFi fw version: ");
    Serial.println(WiFi.firmwareVersion());
  }
  uint8_t st = WiFi.beginAP(ssid, password, apChannel);
  if (Serial) {
    Serial.print("# beginAP returned status=");
    Serial.println(st);
  }
  if (st == WL_AP_LISTENING) {
    wifiUp = true;
    wifiServer.begin();
    // Build the dashboard ETag once — a CONTENT HASH (FNV-1a) of the embedded
    // page, so ANY edit invalidates stale browser caches, even one that doesn't
    // change the page length (the old "length + frozen v711 tag" ETag missed
    // same-length edits → 304 served a stale page). #109.
    uint32_t etagHash = 2166136261u;
    for (const char *p = dashboardPageHtml; *p; ++p) {
      etagHash ^= (uint8_t)*p;
      etagHash *= 16777619u;
    }
    snprintf(pageEtag, sizeof(pageEtag), "\"d%08lx\"", (unsigned long)etagHash);
    if (Serial) {
      Serial.print("# WiFi AP '");
      Serial.print(ssid);
      Serial.print("' UP — http://");
      Serial.println(WiFi.localIP());
    }
    return true;
  }
  if (Serial) {
    Serial.print("# WiFi AP '"); Serial.print(ssid);
    Serial.println("' FAILED to start");
  }
  return false;
}
