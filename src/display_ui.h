#ifndef DISPLAY_UI_H
#define DISPLAY_UI_H

#include <Arduino.h>
#include <TFT_eSPI.h>

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

// Menu
#define MENU_OPTION_COUNT   3
#define MENU_OPTION_HEIGHT  40
#define MENU_TOP_Y          90

#define MINMAX_FLASH_MS     500

// Colors
#define COLOR_TARGET        TFT_YELLOW
#define COLOR_CURRENT       TFT_CYAN
#define COLOR_LABEL         TFT_WHITE
#define COLOR_MINMAX        TFT_RED
#define COLOR_TEMP          TFT_ORANGE
#define COLOR_RUNTIME       TFT_GREEN
#define COLOR_DEBUG         TFT_MAGENTA
#define COLOR_BG            TFT_BLACK

typedef enum {
	MODE_TARGET_PRESSURE = 0,
	MODE_KP,
	MODE_KI,
	MODE_KD,
	MODE_COUNT
} EncoderMode;

void drawStaticUI();
void drawTargetPressure(bool forceRedraw = false);
void drawCurrentPressure(float psi, bool valid);
void drawTemperature(float tempC);
void drawDebugInfo();
void drawModeIndicator();
const char* getModeName(EncoderMode mode);
void flashMinMax(bool isMin);
void updateMinMaxFlash();
void drawStartupScreen();
void drawMenuScreen(uint8_t menuIndex, bool forceRedraw = false);
void drawMenuFooter(const char* message, uint16_t color);
void drawRuntimeStatic();
void drawRuntimeTarget(float target, float current, bool valid);
void drawRuntimeTemperature(float tempC);
void drawRuntimePauseCountdown(uint32_t secondsRemaining);
void drawRuntimeFooter();

#endif
