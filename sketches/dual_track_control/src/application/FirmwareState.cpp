// application — state definitions (#189), verbatim from the .ino.
#include "FirmwareState.h"

// Gear state — declared here so curvatureDrive() and rcCommand() can read
// it for the Eco-only conditional caps above. updateGear() in [GEAR]
// owns the writes.
float gearScale  = GEAR_LOW_SCALE;
Gear  currentGear = GEAR_LOW;

// [SAFETY] latches — set by the [SAFETY] module (search [SAFETY]), read here by
// updateGear() and by the output gate. Declared early so both can see them.
bool batteryCutoffLatched = false;  // worst pack <= CUTOFF_THRESH_V → cut motors
bool ecoLockLatched       = false;  // worst pack <= ECO_LOCK_THRESH_V → force Eco gear
bool batteryOkConfirmed   = false;  // a valid reading has confirmed pack ABOVE cutoff (boot gate)

// [SAFETY] motor over-temp stage flags (#111) — NON-LATCHING (hysteresis). Set by
// thermalUpdate() off the hottest of all 4 sensors; read by updateGear() (Eco),
// the output gate (cut), alertUpdate() (beeps), and buildTelemJson() (dashboard).
bool tempWarnActive = false;  // hottest sensor >= TEMP_WARN_ON_C → warning trill
bool tempEcoActive  = false;  // hottest sensor >= TEMP_ECO_ON_C  → force Eco gear
bool tempCutActive  = false;  // hottest sensor >= TEMP_CUT_ON_C  → cut motors (auto-recovers)

