// config — firmware identity (#185).
#pragma once

// Firmware version — SINGLE SOURCE OF TRUTH (#124). The debug banner prints
// it; docs and FIRMWARE-UPLOAD-LOG reference it. Bump here, nowhere else.
const char FIRMWARE_VERSION[] = "V7.35";
