/**
 * @file ota_update.h
 * @brief Over-the-air firmware update via GitHub Releases.
 *
 * Flow:
 *  1. otaStart() creates a background FreeRTOS task on Core 1.
 *  2. Task starts a captive-portal WiFi AP ("ApolloUpdate").
 *  3. User scans QR code → phone auto-joins AP → captive portal opens.
 *  4. User submits home WiFi credentials → ESP connects as STA.
 *  5. ESP queries GitHub Releases API for latest tag_name / firmware URL.
 *  6. If up to date → OTA_STATE_VERSION_CURRENT.
 *  7. If update available → waits for otaStatus.updateConfirmed.
 *  8. Downloads & flashes firmware via esp_https_ota (cert bundle, HTTPS).
 *  9. Sets NVS rollback flag, then restarts.
 *
 * Rollback:
 *  On the next boot, otaCheckRollback() returns ROLLBACK_PENDING.
 *  The startup code shows a 60-second popup.
 *    - User confirms → otaConfirmValid() clears the flag, boot proceeds.
 *    - Timeout / user rejects → otaRollbackNow() switches the OTA boot
 *      partition to the previously running partition and restarts.
 */

#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// OTA state machine
// ---------------------------------------------------------------------------
typedef enum {
    OTA_STATE_IDLE = 0,        ///< Not started
    OTA_STATE_STARTING_AP,     ///< Spinning up WiFi AP + captive portal
    OTA_STATE_WAITING_CREDS,   ///< AP active, awaiting credentials form submit
    OTA_STATE_CONNECTING_STA,  ///< Joining user's WiFi network
    OTA_STATE_CHECKING_VERSION,///< Querying GitHub Releases API
    OTA_STATE_VERSION_CURRENT, ///< Firmware is already up to date
    OTA_STATE_UPDATE_AVAILABLE,///< Newer version available, awaiting user confirm
    OTA_STATE_DOWNLOADING,     ///< Downloading + flashing firmware
    OTA_STATE_SUCCESS,         ///< Flash complete, reboot imminent
    OTA_STATE_FAILED,          ///< Unrecoverable error (see errorMessage)
    OTA_STATE_CANCELLED,       ///< User cancelled
} OtaState;

// ---------------------------------------------------------------------------
// Shared status struct – written by OTA task, read by display task.
// Simple volatile fields are used; no mutex needed for these scalar types on
// Xtensa (naturally atomic <= 4-byte aligned reads).
// ---------------------------------------------------------------------------
typedef struct {
    volatile OtaState state;
    char        latestVersion[32];   ///< e.g. "v2.1.0"
    char        downloadUrl[256];    ///< firmware.bin asset URL
    char        errorMessage[96];    ///< Human-readable failure description
    volatile int  downloadProgress;  ///< 0-100 during OTA_STATE_DOWNLOADING
    volatile bool updateConfirmed;   ///< Set true by UI to start download
    volatile bool cancelRequested;   ///< Set true by UI to abort
} OtaStatus;

extern OtaStatus otaStatus;

// ---------------------------------------------------------------------------
// Rollback state
// ---------------------------------------------------------------------------
typedef enum {
    ROLLBACK_NONE    = 0,  ///< Normal boot
    ROLLBACK_PENDING = 1,  ///< First boot after OTA – needs validation
} RollbackState;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

/**
 * @brief Initialise OTA subsystem (clears status struct).
 *        Call once from setup().
 */
void otaInit(void);

/**
 * @brief Start the OTA process.
 *        Creates a background FreeRTOS task; returns immediately.
 *        Monitor otaStatus.state for progress.
 *        Motor must already be disabled before calling.
 */
void otaStart(void);

/**
 * @brief Signal the OTA task to abort and clean up WiFi.
 *        Safe to call from any state; idempotent.
 */
void otaCancel(void);

/**
 * @brief Signal the OTA task that the user has confirmed the update.
 *        Only effective in OTA_STATE_UPDATE_AVAILABLE.
 */
void otaConfirmUpdate(void);

// ---------------------------------------------------------------------------
// Rollback API
// ---------------------------------------------------------------------------

/**
 * @brief Check whether the current boot is the first after an OTA flash.
 *        Reads the NVS "ota_state/pending" flag written before the OTA reboot.
 * @return ROLLBACK_PENDING if validation is required, ROLLBACK_NONE otherwise.
 */
RollbackState otaCheckRollback(void);

/**
 * @brief Accept the new firmware: clears the NVS pending flag.
 *        Call when the user (or health-check logic) approves the new build.
 */
void otaConfirmValid(void);

/**
 * @brief Reject the new firmware: clears the flag, switches the OTA boot
 *        partition back to the previously running partition, then restarts.
 */
void otaRollbackNow(void);

#endif // OTA_UPDATE_H
