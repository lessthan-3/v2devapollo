/**
 * @file storage.cpp
 * @brief NVS persistence implementation for settings and hour meter
 */

#include "storage.h"
#include "dual_core_motor.h"
#include "job_timer.h"
#include <Preferences.h>

static Preferences prefs;

// ---------------------------------------------------------------------------
// Hour meter
// ---------------------------------------------------------------------------

void loadHourMeter(void) {
    if (prefs.begin(NVS_NAMESPACE_APP, true)) {
        totalRuntimeTenths = prefs.getULong("hourTenths", 0);
        prefs.end();
        Serial.printf("Hour meter loaded: %lu.%lu hours\n",
                      totalRuntimeTenths / 10, totalRuntimeTenths % 10);
    } else {
        prefs.end();
        // Namespace does not exist yet — create it with an initial value of 0
        Serial.println("Hour meter: No saved data, initialising...");
        if (prefs.begin(NVS_NAMESPACE_APP, false)) {
            prefs.putULong("hourTenths", 0);
            prefs.end();
            Serial.println("Hour meter: Initialised to 0.0 hours");
        } else {
            Serial.println("Hour meter: WARNING - NVS not available, data won't persist");
        }
        totalRuntimeTenths = 0;
    }
}

void saveHourMeter(void) {
    if (prefs.begin(NVS_NAMESPACE_APP, false)) {
        prefs.putULong("hourTenths", totalRuntimeTenths);
        prefs.end();
        Serial.printf("Hour meter saved: %lu.%lu hours\n",
                      totalRuntimeTenths / 10, totalRuntimeTenths % 10);
    } else {
        Serial.println("Hour meter: WARNING - Failed to save to NVS");
    }
}

// ---------------------------------------------------------------------------
// System timer (lifetime, non-resettable)
// ---------------------------------------------------------------------------

void loadSystemTimer(void) {
    if (prefs.begin(NVS_NAMESPACE_APP, true)) {
        totalSystemTimeTenths = prefs.getULong("sysTenths", 0);
        prefs.end();
        Serial.printf("System timer loaded: %lu.%lu hours\n",
                      totalSystemTimeTenths / 10, totalSystemTimeTenths % 10);
    } else {
        prefs.end();
        if (prefs.begin(NVS_NAMESPACE_APP, false)) {
            prefs.putULong("sysTenths", 0);
            prefs.end();
        }
        totalSystemTimeTenths = 0;
    }
}

void saveSystemTimer(void) {
    if (prefs.begin(NVS_NAMESPACE_APP, false)) {
        prefs.putULong("sysTenths", totalSystemTimeTenths);
        prefs.end();
    }
}

// ---------------------------------------------------------------------------
// Job timer
// ---------------------------------------------------------------------------

void loadJobTimer(void) {
    if (prefs.begin(NVS_NAMESPACE_APP, true)) {
        totalJobTimeTenths = prefs.getULong("jobTenths", 0);
        prefs.end();
        Serial.printf("Job timer loaded: %lu.%lu hours\n",
                      totalJobTimeTenths / 10, totalJobTimeTenths % 10);
    } else {
        prefs.end();
        Serial.println("Job timer: No saved data, initialising...");
        if (prefs.begin(NVS_NAMESPACE_APP, false)) {
            prefs.putULong("jobTenths", 0);
            prefs.end();
        }
        totalJobTimeTenths = 0;
    }
}

void saveJobTimer(void) {
    if (prefs.begin(NVS_NAMESPACE_APP, false)) {
        prefs.putULong("jobTenths", totalJobTimeTenths);
        prefs.end();
        Serial.printf("Job timer saved: %lu.%lu hours\n",
                      totalJobTimeTenths / 10, totalJobTimeTenths % 10);
    } else {
        Serial.println("Job timer: WARNING - Failed to save to NVS");
    }
}

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------

void loadSettings(void) {
    if (prefs.begin(NVS_NAMESPACE_APP, true)) {
        settingsIdleDev         = prefs.getFloat("idleDev",  IDLE_ENTRY_DEVIATION_PSI);
        powerPauseSeconds       = prefs.getUShort("idleSec", IDLE_ENTRY_SECONDS);
        powerPauseBeeperEnabled = prefs.getBool("warnBeep",  true);
        displayUnits            = (DisplayUnits)prefs.getUChar("units", UNITS_IMPERIAL);
        prefs.end();
    } else {
        prefs.end();
        settingsIdleDev         = IDLE_ENTRY_DEVIATION_PSI;
        powerPauseSeconds       = IDLE_ENTRY_SECONDS;
        powerPauseBeeperEnabled = true;
    }

    // Target pressure always boots at 0 — not persisted
    settingsStartPsi = TARGET_PSI_DEFAULT;

    // Clamp to valid ranges
    settingsIdleDev   = constrain(settingsIdleDev,   IDLE_DEV_MIN, IDLE_DEV_MAX);
    powerPauseSeconds = constrain(powerPauseSeconds, POWER_PAUSE_SEC_MIN, POWER_PAUSE_SEC_MAX);

    // Push to motor task
    setIdleEntryDeviationSafe(settingsIdleDev);
    setIdleEntrySecondsSafe(powerPauseSeconds);
}

void saveSettings(void) {
    if (prefs.begin(NVS_NAMESPACE_APP, false)) {
        prefs.putFloat("idleDev",   settingsIdleDev);
        prefs.putUShort("idleSec",  powerPauseSeconds);
        prefs.putBool("warnBeep",   powerPauseBeeperEnabled);
        prefs.putUChar("units",     (uint8_t)displayUnits);
        prefs.end();
    } else {
        prefs.end();
    }
}
