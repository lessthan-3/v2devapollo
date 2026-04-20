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
void drawAboutScreen(uint32_t totalSystemTimeTenths, const char* firmwareVersion);

// Debug overlay preview carousel — compiled out unless DEBUG_OVERLAY_PREVIEW != 0
#if DEBUG_OVERLAY_PREVIEW
void drawDebugOverlayPreview(uint8_t stage);
#endif

#endif
