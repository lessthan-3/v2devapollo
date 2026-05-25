#ifndef DISPLAY_UI_H
#define DISPLAY_UI_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "config.h"
#include "dual_core_motor.h"

extern TFT_eSPI tft;

void drawStartupScreen(void);
void drawMenuScreen(uint8_t menuIndex, bool forceRedraw = false);
void drawMenuFooter(const char* message, uint16_t color);
void drawSettingsFooter(const char* message, uint16_t color);
void drawPowerPauseSettingsScreen(uint8_t settingsIndex, uint16_t pauseSeconds, bool beeperEnabled, uint16_t warnSeconds, DisplayUnits units, bool editing, bool forceRedraw = false);
void drawPowerPauseSettingsFooter(const char* message, uint16_t color);
void drawPowerPauseSettingsRow(uint8_t settingsIndex, uint16_t pauseSeconds, bool beeperEnabled, uint16_t warnSeconds, DisplayUnits units, bool selected, bool editing);
void drawRuntimeStatic(DisplayUnits units);
void drawRuntimeTarget(float target, float current, DisplayUnits units, bool valid, bool forceRedraw = false, uint16_t motorSpeed = 0);
void drawRuntimeMotorPower(uint16_t motorSpeed, bool forceRedraw = false);
void drawRuntimeJobTime(uint32_t jobTimeSeconds, bool forceRedraw = false);
void drawRuntimeTemperature(float tempC, DisplayUnits units, bool forceRedraw = false);
void drawRuntimeSensorPressureDebug(float rawPsi, int32_t rawValue, bool valid, bool forceRedraw = false);
void drawRuntimePowerPauseOverlay(IdleState idleState, uint32_t secondsRemaining, bool forceRedraw = false);
void drawRuntimeOverTempOverlay(float tempC, bool forceRedraw = false);
void drawRuntimeFilterWarningOverlay(void);
void drawSupportMenuScreen(uint8_t menuIndex);
void drawSupportFaqScreen(void);
void drawSupportTechScreen(void);
void drawSupportContactScreen(void);
void drawTimersScreen(uint32_t totalRuntimeTenths, uint32_t totalJobTimeTenths, uint8_t selectedOption);
void drawAboutScreen(uint32_t totalSystemTimeTenths, const char* firmwareVersion, bool confirmVisible = false);
void drawAboutResetPopup(uint8_t selectedOption);  // 0 = Reset, 1 = Return

// OTA Update screen
#include "ota_update.h"
/**
 * @brief Draw the OTA status screen.
 * @param state            Current OTA state.
 * @param detail           Extra detail string (latest version, error, etc.).
 * @param progress         Download progress 0-100 (used in DOWNLOADING state).
 * @param selectedOption   0 = Install / Confirm, 1 = Cancel (UPDATE_AVAILABLE).
 * @param forceRedraw      true = repaint even if state hasn't changed.
 */
void drawOtaScreen(OtaState state, const char* detail, int progress,
                   uint8_t selectedOption = 0, bool forceRedraw = false);

/**
 * @brief Draw the first-boot rollback confirmation popup.
 * @param newVersion       Version string of the newly flashed firmware.
 * @param secondsRemaining Seconds until auto-rollback.
 * @param selectedOption   0 = Confirm, 1 = Rollback (encoder selection).
 */
void drawRollbackPopup(const char* newVersion, uint32_t secondsRemaining,
                       uint8_t selectedOption);

// Debug overlay preview carousel — compiled out unless DEBUG_OVERLAY_PREVIEW != 0
#if DEBUG_OVERLAY_PREVIEW
void drawDebugOverlayPreview(uint8_t stage);
#endif

#endif
