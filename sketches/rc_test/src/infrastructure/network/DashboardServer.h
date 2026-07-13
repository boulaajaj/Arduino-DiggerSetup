// infrastructure/network — the HTTP side of the dashboard service (#181):
// request read + routing, the one-shot /data JSON response, the cached-page
// ETag/304 path, and the incremental page transfer (#69). The port entry
// points (dashboardServiceInitialize/Update) are declared in
// ports/DashboardServicePort.h; initialize is implemented by WifiService,
// update by DashboardServer.
#pragma once
