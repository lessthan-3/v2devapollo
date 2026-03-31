#ifndef DISPLAY_UI_H
#define DISPLAY_UI_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "dual_core_motor.h"
#include "config.h"

extern TFT_eSPI tft;

// Temporary debug toggle for raw sensor pressure display
// #define DEBUG_DISPLAY_SENSOR_PRESSURE 1

void drawStaticUI();
void drawTargetPressure(bool forceRedraw = false);
void drawCurrentPressure(float psi, bool valid);
void drawTemperature(float tempC);
void drawDebugInfo();
void flashMinMax(bool isMin);
void updateMinMaxFlash();
void drawStartupScreen();
void drawMenuScreen(uint8_t menuIndex, bool forceRedraw = false);
void drawMenuFooter(const char* message, uint16_t color);
void drawSettingsScreen(uint8_t settingsIndex, float idleDev, float startPsi, bool editing, bool forceRedraw = false);
void drawSettingsFooter(const char* message, uint16_t color);
void drawSettingsRow(uint8_t settingsIndex, float idleDev, float startPsi, bool selected, bool editing);
void drawPowerPauseSettingsScreen(uint8_t settingsIndex, uint16_t pauseSeconds, bool beeperEnabled, uint16_t warnSeconds, bool editing, bool forceRedraw = false);
void drawPowerPauseSettingsFooter(const char* message, uint16_t color);
void drawPowerPauseSettingsRow(uint8_t settingsIndex, uint16_t pauseSeconds, bool beeperEnabled, uint16_t warnSeconds, bool selected, bool editing);
void drawRuntimeStatic();
void drawRuntimeTarget(float target, float current, bool valid, bool forceRedraw = false);
void drawRuntimeTemperature(float tempC, bool forceRedraw = false);
void drawRuntimePauseCountdown(uint32_t secondsRemaining, bool forceRedraw = false);
void drawRuntimeFooter();
void drawRuntimeMotorPower(uint16_t motorSpeed, bool forceRedraw = false);
void drawRuntimeSensorPressureDebug(float rawPsi, int32_t rawValue, bool valid, bool forceRedraw = false);
void drawRuntimePowerPauseOverlay(IdleState idleState, bool forceRedraw = false);

#endif
