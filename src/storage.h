/**
 * @file storage.h
 * @brief NVS persistence for settings and hour meter
 */

#ifndef STORAGE_H
#define STORAGE_H

#include <Arduino.h>
#include "config.h"
#include "dual_core_motor.h"  // For DisplayUnits, IDLE_ENTRY_DEVIATION_PSI, etc.

// NVS namespace shared by all app settings
#define NVS_NAMESPACE_APP   "apollo"

// Externally owned state written by load/save functions
extern uint32_t totalRuntimeTenths;
extern uint32_t totalSystemTimeTenths;
extern float    settingsIdleDev;
extern float    settingsStartPsi;
extern uint16_t powerPauseSeconds;
extern bool     powerPauseBeeperEnabled;
extern DisplayUnits displayUnits;

/**
 * @brief Load the filter hour meter from NVS into totalRuntimeTenths.
 *        Initialises the NVS key to 0 if it does not yet exist.
 */
void loadHourMeter(void);

/**
 * @brief Persist totalRuntimeTenths to NVS.
 */
void saveHourMeter(void);

/**
 * @brief Load the system timer from NVS into totalSystemTimeTenths.
 *        Never resets — accumulates lifetime motor hours.
 */
void loadSystemTimer(void);

/**
 * @brief Persist totalSystemTimeTenths to NVS.
 */
void saveSystemTimer(void);

/**
 * @brief Load the job timer from NVS into totalJobTimeTenths.
 *        Initialises the NVS key to 0 if it does not yet exist.
 */
void loadJobTimer(void);

/**
 * @brief Persist totalJobTimeTenths to NVS.
 */
void saveJobTimer(void);

/**
 * @brief Load all user settings from NVS.
 *        Falls back to compile-time defaults when keys are absent.
 *        Also pushes loaded values to the motor task via the Safe setters.
 */
void loadSettings(void);

/**
 * @brief Persist all user settings to NVS.
 */
void saveSettings(void);

#endif // STORAGE_H
