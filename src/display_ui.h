#ifndef DISPLAY_UI_H
#define DISPLAY_UI_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "dual_core_motor.h"

extern TFT_eSPI tft;

// Display layout constants (480x320 landscape)
#define SCREEN_WIDTH        480
#define SCREEN_HEIGHT       320

#define PRESSURE_ZONE_HEIGHT 213
#define STATUS_ZONE_Y       220
#define LEFT_COLUMN_X       10
#define RIGHT_COLUMN_X      245
#define COLUMN_WIDTH        225

// Runtime screen grid (3x3)
#define GRID_COLS           3
#define GRID_ROWS           3
#define CELL_WIDTH          (SCREEN_WIDTH / GRID_COLS)
#define CELL_HEIGHT         (SCREEN_HEIGHT / GRID_ROWS)
#define RUNTIME_TOP_HEIGHT  (CELL_HEIGHT * 2)
#define RUNTIME_RIGHT_X     (CELL_WIDTH * 2)
#define RUNTIME_FOOTER_Y    (CELL_HEIGHT * 2)
#define RUNTIME_FOOTER_WIDTH (CELL_WIDTH * 2)

// Menu
#define MENU_OPTION_COUNT   4
#define MENU_OPTION_HEIGHT  40
#define MENU_TOP_Y          90

// Settings - Updated for new structure
#define SETTINGS_OPTION_COUNT 4
#define SETTINGS_OPTION_HEIGHT 40
#define SETTINGS_TOP_Y      85

#define MINMAX_FLASH_MS     500

// Temporary debug toggle for raw sensor pressure display
// #define DEBUG_DISPLAY_SENSOR_PRESSURE 1

// ===== COLOR THEME DEFINITIONS =====
// Centralized color management for easy theme adjustments
#define COLOR_BG                TFT_BLACK      // Background color
#define COLOR_TEXT_PRIMARY      TFT_WHITE      // Primary text color
#define COLOR_TEXT_SECONDARY    TFT_LIGHTGREY  // Secondary text color

// Target pressure indicator colors
#define COLOR_TARGET_ACTIVE     TFT_GREEN      // Target PSI in range
#define COLOR_TARGET_OUTRANGE   TFT_RED        // Target PSI out of range
#define COLOR_TARGET_INACTIVE   TFT_YELLOW     // Target PSI (motor off)

// Status and indicator colors
#define COLOR_CURRENT           TFT_CYAN       // Current pressure value
#define COLOR_TEMP              TFT_ORANGE     // Temperature display
#define COLOR_TEMP_WARNING      TFT_RED        // Temperature warning
#define COLOR_RUNTIME           TFT_GREEN      // Runtime/job time
#define COLOR_WARNING           TFT_ORANGE     // Warning state
#define COLOR_ERROR             TFT_RED        // Error state
#define COLOR_SUCCESS           TFT_GREEN      // Success state

// UI element colors
#define COLOR_LABEL             TFT_WHITE      // Labels
#define COLOR_MINMAX            TFT_RED        // Min/Max flash
#define COLOR_DEBUG             TFT_MAGENTA    // Debug info
#define COLOR_OVERLAY_BG        TFT_BLACK      // Overlay background
#define COLOR_OVERLAY_BORDER    TFT_WHITE      // Overlay border
#define COLOR_MENU_SELECT       TFT_DARKGREY   // Menu selection
#define COLOR_MENU_EDIT         TFT_YELLOW     // Menu editing mode

typedef enum {
	MODE_TARGET_PRESSURE = 0,
	MODE_KP,
	MODE_KI,
	MODE_KD,
	MODE_COUNT
} EncoderMode;

typedef enum {
  UNITS_IMPERIAL = 0,  // PSI, Fahrenheit
  UNITS_METRIC = 1     // mbar, Celsius
} DisplayUnits;

// Old UI functions (removed - using new design)
// void drawStaticUI();
// void drawTargetPressure(bool forceRedraw = false);
// void drawCurrentPressure(float psi, bool valid);
// void drawTemperature(float tempC);
// void drawDebugInfo();
void drawModeIndicator();
const char* getModeName(EncoderMode mode);
void drawStartupScreen();
void drawMenuScreen(uint8_t menuIndex, bool forceRedraw = false);
void drawMenuFooter(const char* message, uint16_t color);
void drawSettingsFooter(const char* message, uint16_t color);
void drawSettingsRow(uint8_t settingsIndex, float kp, float ki, float kd, float idleDev, float startPsi, bool selected, bool editing);
void drawPowerPauseSettingsScreen(uint8_t settingsIndex, uint16_t pauseSeconds, bool beeperEnabled, uint16_t warnSeconds, DisplayUnits units, bool editing, bool forceRedraw = false);
void drawPowerPauseSettingsFooter(const char* message, uint16_t color);
void drawPowerPauseSettingsRow(uint8_t settingsIndex, uint16_t pauseSeconds, bool beeperEnabled, uint16_t warnSeconds, DisplayUnits units, bool selected, bool editing);
void drawRuntimeStatic(DisplayUnits units);
void drawRuntimeTarget(float target, float current, DisplayUnits units, bool valid, bool forceRedraw = false);
void drawRuntimeJobTime(uint32_t jobTimeSeconds, bool forceRedraw = false);
void drawRuntimeTemperature(float tempC, DisplayUnits units, bool forceRedraw = false);
void drawRuntimeSensorPressureDebug(float rawPsi, int32_t rawValue, bool valid, bool forceRedraw = false);
void drawRuntimePowerPauseOverlay(IdleState idleState, uint32_t secondsRemaining, bool forceRedraw = false);
void drawRuntimeOverTempOverlay(float tempC, bool forceRedraw = false);
void drawRuntimeFilterWarningOverlay();
void drawSupportScreen();
void drawAboutScreen(uint32_t totalRuntimeTenths, const char* firmwareVersion, uint8_t selectedOption = 0);

#endif
