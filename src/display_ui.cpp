#include "display_ui.h"
#include "motor_control.h"
#include "splash_image.h"
#include <math.h>
#include <stdint.h>

extern float targetPsi;
extern unsigned long minMaxFlashStart;
extern bool showingMinMax;
extern bool isMinNotMax;
extern EncoderMode currentEncoderMode;
extern uint32_t sessionStartMillis;
extern uint32_t totalRuntimeTenths;
extern unsigned long pidLoopIntervalMs;
extern float currentTemperatureC;
extern bool overTempActive;

// Old UI functions removed - using new Runtime UI design

void drawDebugInfo() {
  static uint16_t lastMotorSpeed = 0xFFFF;
  static AcFrequency lastAcFreq = AC_FREQ_UNKNOWN;
  static unsigned long lastLoopTime = 9999;
  static float lastKp = -1, lastKi = -1, lastKd = -1;
  static uint32_t lastZcCount = 0xFFFFFFFF;

  uint16_t motorSpeed = motorState.motorSpeed;
  AcFrequency acFreq = motorState.acFrequency;
  uint32_t zeroCrossingCount = getZeroCrossingCount();
  PidController* pid = getPressurePid();
  float kp, ki, kd;
  pidGetGains(pid, &kp, &ki, &kd);

  if (motorSpeed != lastMotorSpeed) {
    lastMotorSpeed = motorSpeed;
    tft.fillRect(LEFT_COLUMN_X + 80, STATUS_ZONE_Y + 18, 50, 25, COLOR_BG);
    tft.setTextColor(COLOR_RUNTIME, COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(LEFT_COLUMN_X + 80, STATUS_ZONE_Y + 22);
    tft.printf("%3u.%1u%%", motorSpeed / 10, motorSpeed % 10);
  }

  if (acFreq != lastAcFreq) {
    lastAcFreq = acFreq;
    tft.fillRect(LEFT_COLUMN_X + 140, STATUS_ZONE_Y + 18, 50, 25, COLOR_BG);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(LEFT_COLUMN_X + 140, STATUS_ZONE_Y + 22);
    if (acFreq == AC_FREQ_UNKNOWN) {
      tft.print("--");
    } else {
      tft.printf("%2d", acFreq);
    }
  }

  if (pidLoopIntervalMs != lastLoopTime) {
    lastLoopTime = pidLoopIntervalMs;
    tft.fillRect(LEFT_COLUMN_X + 200, STATUS_ZONE_Y + 18, 60, 25, COLOR_BG);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(LEFT_COLUMN_X + 200, STATUS_ZONE_Y + 22);
    tft.printf("%3lu", pidLoopIntervalMs);
  }

  if (fabsf(kp - lastKp) > 0.01f || fabsf(ki - lastKi) > 0.01f || fabsf(kd - lastKd) > 0.01f) {
    lastKp = kp;
    lastKi = ki;
    lastKd = kd;
    tft.fillRect(LEFT_COLUMN_X + 280, STATUS_ZONE_Y + 18, 190, 25, COLOR_BG);
    tft.setTextColor(COLOR_DEBUG, COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(LEFT_COLUMN_X + 280, STATUS_ZONE_Y + 22);
    char pidStr[24];
    snprintf(pidStr, sizeof(pidStr), "%.0f/%.1f/%.1f", kp, ki, kd);
    tft.print(pidStr);
  }

  if (zeroCrossingCount != lastZcCount) {
    lastZcCount = zeroCrossingCount;
    tft.fillRect(LEFT_COLUMN_X + 50, STATUS_ZONE_Y + 32, 140, 16, COLOR_BG);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.setTextSize(1);
    tft.setCursor(LEFT_COLUMN_X + 50, STATUS_ZONE_Y - 28);
    tft.printf("%lu", (unsigned long)zeroCrossingCount);
  }
}

const char* getModeName(EncoderMode mode) {
  switch (mode) {
    case MODE_TARGET_PRESSURE: return "TARGET";
    case MODE_KP: return "Kp";
    case MODE_KI: return "Ki";
    case MODE_KD: return "Kd";
    default: return "???";
  }
}

void drawModeIndicator() {
  static EncoderMode lastMode = MODE_COUNT;

  if (lastMode == currentEncoderMode) {
    unsigned long sessionMs = millis() - sessionStartMillis;
    unsigned long sessionMin = sessionMs / 60000;
    unsigned long sessionHr = sessionMin / 60;
    sessionMin %= 60;

    tft.fillRect(SCREEN_WIDTH - 100, STATUS_ZONE_Y + 55, 100, 30, COLOR_BG);
    tft.setTextColor(COLOR_RUNTIME, COLOR_BG);
    tft.setTextSize(1);
    tft.setCursor(SCREEN_WIDTH - 100, STATUS_ZONE_Y + 55);
    tft.printf("Sess: %lu:%02lu", sessionHr, sessionMin);
    tft.setCursor(SCREEN_WIDTH - 100, STATUS_ZONE_Y + 68);
    tft.printf("Total: %lu.%lu hr", totalRuntimeTenths / 10, totalRuntimeTenths % 10);
    return;
  }
  lastMode = currentEncoderMode;

  tft.fillRect(0, STATUS_ZONE_Y + 50, SCREEN_WIDTH, 30, COLOR_BG);

  tft.setTextSize(2);
  tft.setCursor(LEFT_COLUMN_X, STATUS_ZONE_Y + 55);

  tft.setTextColor(TFT_BLACK, COLOR_MENU_EDIT);
  tft.printf(" MODE: %s ", getModeName(currentEncoderMode));

  tft.setCursor(LEFT_COLUMN_X + 150, STATUS_ZONE_Y + 55);
  for (int i = 0; i < MODE_COUNT; i++) {
    if (i == currentEncoderMode) {
      tft.setTextColor(TFT_BLACK, TFT_GREEN);
    } else {
      tft.setTextColor(TFT_DARKGREY, COLOR_BG);
    }
    tft.printf("[%d]", i);
  }

  unsigned long sessionMs = millis() - sessionStartMillis;
  unsigned long sessionMin = sessionMs / 60000;
  unsigned long sessionHr = sessionMin / 60;
  sessionMin %= 60;

  tft.setTextColor(COLOR_RUNTIME, COLOR_BG);
  tft.setTextSize(1);
  tft.setCursor(SCREEN_WIDTH - 100, STATUS_ZONE_Y + 55);
  tft.printf("Sess: %lu:%02lu", sessionHr, sessionMin);
  tft.setCursor(SCREEN_WIDTH - 100, STATUS_ZONE_Y + 68);
  tft.printf("Total: %lu.%lu hr", totalRuntimeTenths / 10, totalRuntimeTenths % 10);
}

// flashMinMax and updateMinMaxFlash removed - no longer used in new UI

void drawStartupScreen() {
  tft.fillScreen(COLOR_BG);
  int imageX = (SCREEN_WIDTH - SPLASH_IMAGE_WIDTH) / 2;
  int imageY = (SCREEN_HEIGHT - SPLASH_IMAGE_HEIGHT) / 2;
  tft.pushImage(imageX, imageY, SPLASH_IMAGE_WIDTH, SPLASH_IMAGE_HEIGHT, splashImage);

}

void drawMenuFooter(const char* message, uint16_t color) {
  tft.fillRect(0, SCREEN_HEIGHT - 40, SCREEN_WIDTH, 40, COLOR_BG);
  if (message != NULL && message[0] != '\0') {
    tft.setTextColor(color, COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(20, SCREEN_HEIGHT - 30);
    tft.print(message);
  }
}

void drawSettingsFooter(const char* message, uint16_t color) {
  tft.fillRect(0, SCREEN_HEIGHT - 40, SCREEN_WIDTH, 40, COLOR_BG);
  if (message != NULL && message[0] != '\0') {
    tft.setTextColor(color, COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(20, SCREEN_HEIGHT - 30);
    tft.print(message);
  }
}

void drawMenuScreen(uint8_t menuIndex, bool forceRedraw) {
  static uint8_t lastMenuIndex = 255;
  if (!forceRedraw && lastMenuIndex == menuIndex) {
    return;
  }
  lastMenuIndex = menuIndex;

  tft.fillScreen(COLOR_BG);
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(185, 30);
  tft.print("MENU");

  const char* options[MENU_OPTION_COUNT] = {"Start Motor", "Settings", "SUPPORT", "About"};
  for (uint8_t i = 0; i < MENU_OPTION_COUNT; i++) {
    int y = MENU_TOP_Y + (i * MENU_OPTION_HEIGHT);
    if (i == menuIndex) {
      tft.fillRect(40, y - 5, SCREEN_WIDTH - 80, MENU_OPTION_HEIGHT, COLOR_MENU_SELECT);
      tft.setTextColor(TFT_BLACK, COLOR_MENU_SELECT);
    } else {
      tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
    }
    tft.setTextSize(2);
    tft.setCursor(60, y);
    tft.print(options[i]);
  }

  drawMenuFooter("Press to select", COLOR_SUCCESS);
}

void drawSettingsScreen(uint8_t settingsIndex, float kp, float ki, float kd, float idleDev, float startPsi, bool editing, bool forceRedraw) {
  static uint8_t lastIndex = 255;
  static float lastKp = -1.0f;
  static float lastKi = -1.0f;
  static float lastKd = -1.0f;
  static float lastIdleDev = -1.0f;
  static float lastStartPsi = -1.0f;
  static bool lastEditing = false;

  if (!forceRedraw && lastIndex == settingsIndex && lastEditing == editing &&
      fabsf(kp - lastKp) < 0.01f && fabsf(ki - lastKi) < 0.01f && fabsf(kd - lastKd) < 0.01f &&
      fabsf(idleDev - lastIdleDev) < 0.01f && fabsf(startPsi - lastStartPsi) < 0.01f) {
    return;
  }

  lastIndex = settingsIndex;
  lastEditing = editing;
  lastKp = kp;
  lastKi = ki;
  lastKd = kd;
  lastIdleDev = idleDev;
  lastStartPsi = startPsi;

  tft.fillScreen(COLOR_BG);
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(150, 20);
  tft.print("SETTINGS");

  for (uint8_t i = 0; i < SETTINGS_OPTION_COUNT; i++) {
    bool selected = (i == settingsIndex);
    drawSettingsRow(i, kp, ki, kd, idleDev, startPsi, selected, editing);
  }

  if (editing) {
    drawSettingsFooter("Rotate to adjust, press to exit", COLOR_MENU_EDIT);
  } else {
    drawSettingsFooter("Press to edit / select", COLOR_SUCCESS);
  }
}

void drawSettingsRow(uint8_t settingsIndex, float kp, float ki, float kd, float idleDev, float startPsi, bool selected, bool editing) {
  const char* options[SETTINGS_OPTION_COUNT] = {
    "Power Pause Timeout",
    "Pause Warning Sound", 
    "Units",
    "Exit Settings"
  };
  
  int y = SETTINGS_TOP_Y + (settingsIndex * SETTINGS_OPTION_HEIGHT);
  uint16_t bg = COLOR_BG;
  uint16_t fg = COLOR_TEXT_PRIMARY;
  if (selected) {
    bg = editing ? COLOR_MENU_EDIT : COLOR_MENU_SELECT;
    fg = TFT_BLACK;
  }

  tft.fillRect(40, y - 5, SCREEN_WIDTH - 80, SETTINGS_OPTION_HEIGHT, bg);
  tft.setTextColor(fg, bg);
  tft.setTextSize(2);
  tft.setCursor(60, y);
  tft.print(options[settingsIndex]);

  // Note: Values will be managed by main.cpp state
  // Placeholder for now - actual implementation in main.cpp
}

void drawPowerPauseSettingsFooter(const char* message, uint16_t color) {
  tft.fillRect(0, SCREEN_HEIGHT - 40, SCREEN_WIDTH, 40, COLOR_BG);
  if (message != NULL && message[0] != '\0') {
    tft.setTextColor(color, COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(20, SCREEN_HEIGHT - 30);
    tft.print(message);
  }
}

void drawPowerPauseSettingsScreen(uint8_t settingsIndex, uint16_t pauseSeconds, bool beeperEnabled, uint16_t warnSeconds, DisplayUnits units, bool editing, bool forceRedraw) {
  static uint8_t lastIndex = 255;
  static uint16_t lastPauseSeconds = 0xFFFF;
  static bool lastBeeperEnabled = false;
  static uint16_t lastWarnSeconds = 0xFFFF;
  static DisplayUnits lastUnits = UNITS_IMPERIAL;
  static bool lastEditing = false;

  if (!forceRedraw && lastIndex == settingsIndex && lastEditing == editing &&
      lastPauseSeconds == pauseSeconds && lastBeeperEnabled == beeperEnabled && 
      lastWarnSeconds == warnSeconds && lastUnits == units) {
    return;
  }

  lastIndex = settingsIndex;
  lastEditing = editing;
  lastPauseSeconds = pauseSeconds;
  lastBeeperEnabled = beeperEnabled;
  lastWarnSeconds = warnSeconds;
  lastUnits = units;

  tft.fillScreen(COLOR_BG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(110, 20);
  tft.print("POWER PAUSE SETTINGS");

  for (uint8_t i = 0; i < 4; i++) {
    bool selected = (i == settingsIndex);
    drawPowerPauseSettingsRow(i, pauseSeconds, beeperEnabled, warnSeconds, units, selected, editing);
  }

  if (editing) {
    drawPowerPauseSettingsFooter("Rotate to adjust, press to exit", COLOR_MENU_EDIT);
  } else {
    drawPowerPauseSettingsFooter("Press to edit / select", COLOR_SUCCESS);
  }
}

void drawPowerPauseSettingsRow(uint8_t settingsIndex, uint16_t pauseSeconds, bool beeperEnabled, uint16_t warnSeconds, DisplayUnits units, bool selected, bool editing) {
  const char* options[4] = {"Pause Timeout", "Warning Beeper", "Units", "Exit"};
  int y = SETTINGS_TOP_Y + (settingsIndex * SETTINGS_OPTION_HEIGHT);
  uint16_t bg = COLOR_BG;
  uint16_t fg = COLOR_TEXT_PRIMARY;
  if (selected) {
    bg = editing ? COLOR_MENU_EDIT : COLOR_MENU_SELECT;
    fg = TFT_BLACK;
  }

  tft.fillRect(40, y - 5, SCREEN_WIDTH - 80, SETTINGS_OPTION_HEIGHT, bg);
  tft.setTextColor(fg, bg);
  tft.setTextSize(2);
  tft.setCursor(60, y);
  tft.print(options[settingsIndex]);

  if (settingsIndex == 0) {
    tft.setCursor(SCREEN_WIDTH - 120, y);
    tft.printf("%4u s", pauseSeconds);
  } else if (settingsIndex == 1) {
    tft.setCursor(SCREEN_WIDTH - 100, y);
    tft.print(beeperEnabled ? "ON" : "OFF");
  } else if (settingsIndex == 2) {
    tft.setCursor(SCREEN_WIDTH - 150, y);
    tft.print(units == UNITS_IMPERIAL ? "IMPERIAL" : "  METRIC");
  }
}

// Old drawSettingsRow removed - using drawPowerPauseSettingsRow for Settings screen

void drawRuntimeStatic(DisplayUnits units) {
  tft.fillScreen(COLOR_BG);

  // Header at top
  tft.setTextColor(COLOR_LABEL, COLOR_BG);
  tft.setTextSize(2);
  const char* headerText = (units == UNITS_IMPERIAL) ? "Set Target Pressure" : "Set Target Pressure";
  int headerWidth = strlen(headerText) * 12;  // Approximate width
  tft.setCursor((SCREEN_WIDTH - headerWidth) / 2, 10);
  tft.print(headerText);

  // Draw horizontal line separating top 2/3 from bottom 1/3
  int separatorY = (SCREEN_HEIGHT * 2) / 3;  // 213 pixels
  tft.drawFastHLine(0, separatorY, SCREEN_WIDTH, TFT_DARKGREY);
  
  // Draw vertical line in bottom section (split job time and temp)
  tft.drawFastVLine(SCREEN_WIDTH / 2, separatorY, SCREEN_HEIGHT - separatorY, TFT_DARKGREY);

  // Labels for bottom section
  tft.setTextColor(COLOR_LABEL, COLOR_BG);
  tft.setTextSize(2);
  
  // Job Time label (left side)
  tft.setCursor(10, separatorY + 10);
  tft.print("JOB TIME");
  
  // Temperature label (right side)
  tft.setCursor(SCREEN_WIDTH / 2 + 10, separatorY + 10);
  tft.print("TEMP");
}

void drawRuntimeTarget(float target, float current, DisplayUnits units, bool valid, bool forceRedraw) {
  static float lastTarget = -999.0f;
  static float lastCurrent = -999.0f;
  static bool lastValid = false;
  static uint16_t lastColor = 0;
  static DisplayUnits lastUnits = UNITS_IMPERIAL;

  // Convert to display units
  float displayTarget = target;
  float displayCurrent = current;
  if (units == UNITS_METRIC) {
    // Custom scaling: 3 PSI = 200 mbar, 9 PSI = 625 mbar
    // Linear mapping: mbar = 200 + (psi - 3) * (625 - 200) / (9 - 3)
    // Simplified: mbar = 200 + (psi - 3) * 70.833
    displayTarget = 200.0f + (target - 3.0f) * 70.833f;
    displayCurrent = 200.0f + (current - 3.0f) * 70.833f;
    
    // Ensure minimum of 0 mbar (no negative values)
    if (displayTarget < 0.0f) displayTarget = 0.0f;
    if (displayCurrent < 0.0f) displayCurrent = 0.0f;
  }

  // Determine color based on how close current is to target
  uint16_t color = COLOR_TARGET_INACTIVE;
  if (target < 0.1f) {
    // Motor effectively off
    color = COLOR_TARGET_INACTIVE;
  } else if (!valid) {
    color = TFT_DARKGREY;
  } else {
    float diff = fabsf(current - target);
    if (diff <= 0.5f) {
      color = COLOR_TARGET_ACTIVE;    // Green - within 0.5 PSI
    } else {
      color = COLOR_TARGET_OUTRANGE;  // Red - out of range
    }
  }

  if (!forceRedraw && fabsf(target - lastTarget) < 0.05f && 
      fabsf(current - lastCurrent) < 0.05f && 
      valid == lastValid && color == lastColor && units == lastUnits) {
    return;
  }

  lastTarget = target;
  lastCurrent = current;
  lastValid = valid;
  lastColor = color;
  lastUnits = units;

  int topSectionHeight = (SCREEN_HEIGHT * 2) / 3;
  tft.fillRect(0, 30, SCREEN_WIDTH, topSectionHeight - 30, COLOR_BG);  // Don't clear header

  // Display large target pressure using custom font
  tft.setTextColor(color, COLOR_BG);
  
  char psiStr[10];
  if (units == UNITS_IMPERIAL) {
    snprintf(psiStr, sizeof(psiStr), "%4.1f", displayTarget);
  } else {
    // For mbar, show as integer (no decimal)
    snprintf(psiStr, sizeof(psiStr), "%4.0f", displayTarget);
  }
  
  // Unit label first - positioned near bottom of top section
  tft.setTextFont(1);  // Use built-in font
  tft.setTextSize(3);
  const char* unitLabel = (units == UNITS_IMPERIAL) ? "PSI" : "mbar";
  int unitWidth = strlen(unitLabel) * 6 * 3;  // chars * 6 pixels * textSize 3
  int unitY = topSectionHeight - 35;
  tft.setCursor((SCREEN_WIDTH - unitWidth) / 2, unitY);
  tft.print(unitLabel);
  
  // Use FreeSansBold24pt font - much larger and cleaner than built-in
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextSize(4);  // Scale the 24pt font 4x for massive display
  
  // Get text dimensions using TFT_eSPI methods
  int16_t w = tft.textWidth(psiStr);
  int16_t h = tft.fontHeight();
  
  // Center horizontally and position so bottom of text touches top of unit label
  int x = (SCREEN_WIDTH - w) / 2 - 15;  // Shift left 15px for better visual balance with unit label
  int y = unitY - 10;  // Position baseline just above unit label (10px gap)
  
  tft.setCursor(x, y);
  tft.print(psiStr);
  
  // Reset to built-in font so other text isn't affected
  tft.setTextFont(1);
  tft.setTextSize(1);
}

void drawRuntimeJobTime(uint32_t jobTimeSeconds, bool forceRedraw) {
  static uint32_t lastJobTime = 0xFFFFFFFF;
  if (!forceRedraw && jobTimeSeconds == lastJobTime) {
    return;
  }
  lastJobTime = jobTimeSeconds;

  int separatorY = (SCREEN_HEIGHT * 2) / 3;
  int sectionHeight = SCREEN_HEIGHT - separatorY;
  
  // Clear job time area
  tft.fillRect(2, separatorY + 35, (SCREEN_WIDTH / 2) - 4, sectionHeight - 37, COLOR_BG);

  tft.setTextColor(COLOR_RUNTIME, COLOR_BG);
  tft.setTextSize(4);
  
  // Format as HH:MM:SS
  uint32_t hours = jobTimeSeconds / 3600;
  uint32_t minutes = (jobTimeSeconds % 3600) / 60;
  uint32_t seconds = jobTimeSeconds % 60;
  
  char timeStr[12];
  snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu:%02lu", 
           (unsigned long)hours, (unsigned long)minutes, (unsigned long)seconds);
  
  tft.setCursor(10, separatorY + 45);
  tft.print(timeStr);
}

void drawRuntimeTemperature(float tempC, DisplayUnits units, bool forceRedraw) {
  static float lastTemp = -999.0f;
  static DisplayUnits lastUnits = UNITS_IMPERIAL;
  if (!forceRedraw && fabsf(tempC - lastTemp) < 0.5f && lastTemp != -999.0f && units == lastUnits) {
    return;
  }
  lastTemp = tempC;
  lastUnits = units;

  int separatorY = (SCREEN_HEIGHT * 2) / 3;
  int sectionHeight = SCREEN_HEIGHT - separatorY;
  
  // Clear temperature area
  tft.fillRect((SCREEN_WIDTH / 2) + 2, separatorY + 35, (SCREEN_WIDTH / 2) - 4, sectionHeight - 37, COLOR_BG);

  tft.setTextColor(overTempActive ? COLOR_TEMP_WARNING : COLOR_TEMP, COLOR_BG);
  tft.setTextSize(4);
  tft.setCursor((SCREEN_WIDTH / 2) + 10, separatorY + 45);

  if (overTempActive) {
    tft.print("OVERHEAT");
  } else if (tempC < -100.0f) {
    tft.print("--");
  } else {
    char tempStr[12];
    if (units == UNITS_IMPERIAL) {
      float tempF = (tempC * 9.0f / 5.0f) + 32.0f;
      snprintf(tempStr, sizeof(tempStr), "%3.0f", tempF);
      tft.print(tempStr);
      tft.setTextSize(3);
      tft.print("F");
    } else {
      snprintf(tempStr, sizeof(tempStr), "%3.0f", tempC);
      tft.print(tempStr);
      tft.setTextSize(3);
      tft.print("C");
    }
  }
}

void drawRuntimePauseCountdown(uint32_t secondsRemaining, bool forceRedraw) {
  static uint32_t lastSeconds = 0xFFFFFFFF;
  if (!forceRedraw && secondsRemaining == lastSeconds) {
    return;
  }
  lastSeconds = secondsRemaining;

  tft.fillRect(RUNTIME_RIGHT_X + 2, CELL_HEIGHT + 30, CELL_WIDTH - 4, CELL_HEIGHT - 32, COLOR_BG);
  tft.setTextColor(TFT_CYAN, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(RUNTIME_RIGHT_X + 20, CELL_HEIGHT + 50);

  if (secondsRemaining == UINT32_MAX) {
    tft.print("--");
  } else {
    char secStr[8];
    snprintf(secStr, sizeof(secStr), "%lu", (unsigned long)secondsRemaining);
    tft.print(secStr);
  }
  tft.setTextSize(2);
  tft.print("s");
}

void drawRuntimeFooter() {
  tft.fillRect(0, RUNTIME_FOOTER_Y + 2, RUNTIME_FOOTER_WIDTH, CELL_HEIGHT - 4, COLOR_BG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(10, RUNTIME_FOOTER_Y + 35);
  tft.print("PRESS TO PAUSE / MENU");
}

void drawRuntimeMotorPower(uint16_t motorSpeed, bool forceRedraw) {
  static uint16_t lastMotorSpeed = 0xFFFF;
  if (!forceRedraw && motorSpeed == lastMotorSpeed) {
    return;
  }
  lastMotorSpeed = motorSpeed;

  tft.fillRect(RUNTIME_RIGHT_X + 2, RUNTIME_FOOTER_Y + 30, CELL_WIDTH - 4, CELL_HEIGHT - 32, COLOR_BG);
  tft.setTextColor(COLOR_RUNTIME, COLOR_BG);
  tft.setTextSize(4);
  tft.setCursor(RUNTIME_RIGHT_X + 20, RUNTIME_FOOTER_Y + 45);
  tft.printf("%3u.%1u", motorSpeed / 10, motorSpeed % 10);
  tft.setTextSize(2);
  tft.print("%");
}

void drawRuntimeSensorPressureDebug(float rawPsi, int32_t rawValue, bool valid, bool forceRedraw) {
  static float lastRawPsi = -999.0f;
  static bool lastValid = true;
  static int32_t lastRawValue = 0;

  if (!forceRedraw && valid == lastValid && fabsf(rawPsi - lastRawPsi) < 0.05f && rawValue == lastRawValue) {
    return;
  }

  lastRawPsi = rawPsi;
  lastValid = valid;
  lastRawValue = rawValue;

  tft.fillRect(20, 180, 220, 24, COLOR_BG);
  tft.setTextColor(COLOR_DEBUG, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(20, 182);
  if (!valid) {
    tft.print("RAW --.- ------");
  } else {
    char rawStr[24];
    snprintf(rawStr, sizeof(rawStr), "RAW %4.1f %ld", rawPsi, (long)rawValue);
    tft.print(rawStr);
  }
}

void drawRuntimePowerPauseOverlay(IdleState idleState, uint32_t secondsRemaining, bool forceRedraw) {
  static IdleState lastState = IDLE_STATE_OFF;
  static uint32_t lastSecondsRemaining = 0xFFFFFFFF;
  
  if (!forceRedraw && idleState == lastState && secondsRemaining == lastSecondsRemaining) {
    return;
  }
  lastState = idleState;
  lastSecondsRemaining = secondsRemaining;

  if (idleState == IDLE_STATE_OFF) {
    return;
  }

  const int overlayX = 40;
  const int overlayY = 60;
  const int overlayW = SCREEN_WIDTH - (overlayX * 2);
  const int overlayH = 200;

  // Draw overlay with border
  tft.fillRect(overlayX, overlayY, overlayW, overlayH, COLOR_OVERLAY_BG);
  tft.drawRect(overlayX, overlayY, overlayW, overlayH, COLOR_OVERLAY_BORDER);
  tft.drawRect(overlayX + 1, overlayY + 1, overlayW - 2, overlayH - 2, COLOR_OVERLAY_BORDER);
  
  tft.setTextColor(COLOR_WARNING, COLOR_OVERLAY_BG);
  tft.setTextSize(3);
  tft.setCursor(overlayX + 60, overlayY + 30);
  tft.print("POWER PAUSE");

  if (idleState == IDLE_STATE_PID_RAMP) {
    tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_OVERLAY_BG);
    tft.setTextSize(2);
    tft.setCursor(overlayX + 50, overlayY + 80);
    tft.print("Ramping to idle...");
  } else if (idleState == IDLE_STATE_HOLD) {
    tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_OVERLAY_BG);
    tft.setTextSize(2);
    tft.setCursor(overlayX + 20, overlayY + 80);
    tft.print("Press trigger or turn dial");
    tft.setCursor(overlayX + 70, overlayY + 110);
    tft.print("to resume motor");
    
    // Display countdown timer
    if (secondsRemaining != UINT32_MAX) {
      tft.setTextColor(COLOR_WARNING, COLOR_OVERLAY_BG);
      tft.setTextSize(3);
      tft.setCursor(overlayX + 80, overlayY + 150);
      tft.print("Timeout: ");
      tft.setTextSize(4);
      
      uint32_t minutes = secondsRemaining / 60;
      uint32_t seconds = secondsRemaining % 60;
      char timeStr[10];
      snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu", (unsigned long)minutes, (unsigned long)seconds);
      tft.print(timeStr);
    }
  }
}

void drawRuntimeOverTempOverlay(float tempC, bool forceRedraw) {
  static bool lastShown = false;
  static float lastTemp = -999.0f;
  
  if (!forceRedraw && lastShown && fabsf(tempC - lastTemp) < 0.5f) {
    return;
  }
  lastShown = true;
  lastTemp = tempC;

  const int overlayX = 40;
  const int overlayY = 60;
  const int overlayW = SCREEN_WIDTH - (overlayX * 2);
  const int overlayH = 200;

  // Draw overlay with border
  tft.fillRect(overlayX, overlayY, overlayW, overlayH, COLOR_OVERLAY_BG);
  tft.drawRect(overlayX, overlayY, overlayW, overlayH, COLOR_ERROR);
  tft.drawRect(overlayX + 1, overlayY + 1, overlayW - 2, overlayH - 2, COLOR_ERROR);
  
  tft.setTextColor(COLOR_ERROR, COLOR_OVERLAY_BG);
  tft.setTextSize(3);
  tft.setCursor(overlayX + 80, overlayY + 30);
  tft.print("OVERHEATING");

  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_OVERLAY_BG);
  tft.setTextSize(2);
  tft.setCursor(overlayX + 40, overlayY + 80);
  tft.print("Motor is too hot!");
  
  tft.setCursor(overlayX + 60, overlayY + 110);
  tft.print("Check filter");

  // Display current temperature
  if (tempC > -100.0f) {
    float tempF = (tempC * 9.0f / 5.0f) + 32.0f;
    tft.setTextColor(COLOR_ERROR, COLOR_OVERLAY_BG);
    tft.setTextSize(4);
    tft.setCursor(overlayX + 120, overlayY + 150);
    char tempStr[12];
    snprintf(tempStr, sizeof(tempStr), "%3.0f", tempF);
    tft.print(tempStr);
    tft.setTextSize(3);
    tft.print("F");
  }
}

void drawRuntimeFilterWarningOverlay() {
  const int overlayX = 20;
  const int overlayY = 50;
  const int overlayW = SCREEN_WIDTH - (overlayX * 2);
  const int overlayH = 220;

  // Draw overlay with border
  tft.fillRect(overlayX, overlayY, overlayW, overlayH, COLOR_OVERLAY_BG);
  tft.drawRect(overlayX, overlayY, overlayW, overlayH, COLOR_WARNING);
  tft.drawRect(overlayX + 1, overlayY + 1, overlayW - 2, overlayH - 2, COLOR_WARNING);
  
  // Title
  tft.setTextColor(COLOR_WARNING, COLOR_OVERLAY_BG);
  tft.setTextSize(3);
  tft.setCursor(overlayX + 30, overlayY + 20);
  tft.print("Filter Maintenance");
  tft.setCursor(overlayX + 100, overlayY + 55);
  tft.print("Required");

  // Message
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_OVERLAY_BG);
  tft.setTextSize(2);
  tft.setCursor(overlayX + 30, overlayY + 100);
  tft.print("Clean filters to protect");
  tft.setCursor(overlayX + 30, overlayY + 125);
  tft.print("motor.");

  tft.setCursor(overlayX + 30, overlayY + 160);
  tft.print("Reset Filter Timer in");
  tft.setCursor(overlayX + 30, overlayY + 185);
  tft.print("About Settings to clear.");
}

void drawSupportScreen() {
  tft.fillScreen(COLOR_BG);
  
  // Title
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(160, 20);
  tft.print("SUPPORT");
  
  // Support information box
  const int boxX = 20;
  const int boxY = 70;
  const int boxW = SCREEN_WIDTH - 40;
  const int boxH = 180;
  
  tft.drawRect(boxX, boxY, boxW, boxH, COLOR_TEXT_PRIMARY);
  
  // Placeholder text - user will fill this in later
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(boxX + 10, boxY + 20);
  tft.print("Support information");
  tft.setCursor(boxX + 10, boxY + 50);
  tft.print("will be added here.");
  tft.setCursor(boxX + 10, boxY + 90);
  tft.print("Contact: TBD");
  tft.setCursor(boxX + 10, boxY + 120);
  tft.print("Email: TBD");
  tft.setCursor(boxX + 10, boxY + 150);
  tft.print("Web: TBD");
  
  // Footer
  tft.setTextColor(COLOR_SUCCESS, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(100, SCREEN_HEIGHT - 40);
  tft.print("Press to return to menu");
}

void drawAboutScreen(uint32_t totalRuntimeTenths, const char* firmwareVersion, uint8_t selectedOption) {
  tft.fillScreen(COLOR_BG);
  
  // Title
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(180, 20);
  tft.print("ABOUT");
  
  // Product name
  tft.setTextSize(4);
  tft.setCursor(80, 70);
  tft.print("APOLLO HVLP");
  
  // Firmware version
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(120, 130);
  tft.print("Firmware: ");
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.print(firmwareVersion);
  
  // Filter timer (formerly Total motor runtime)
  uint32_t totalHours = totalRuntimeTenths / 10;
  uint8_t tenths = totalRuntimeTenths % 10;
  
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(50, 170);
  tft.print("Time since last filter clean:");
  
  // Color code: green if < 10 hours, red if >= 10 hours
  uint16_t timeColor = (totalRuntimeTenths < 100) ? COLOR_SUCCESS : COLOR_ERROR;
  tft.setTextColor(timeColor, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(140, 200);
  char hoursStr[16];
  snprintf(hoursStr, sizeof(hoursStr), "%lu.%u hrs", (unsigned long)totalHours, tenths);
  tft.print(hoursStr);
  
  // Footer with two options
  const int optionY = SCREEN_HEIGHT - 50;
  const int option1X = 40;
  const int option2X = 280;
  
  // Option 1: Reset Filter Timer
  if (selectedOption == 0) {
    tft.fillRect(option1X - 5, optionY - 5, 200, 35, COLOR_MENU_SELECT);
    tft.setTextColor(TFT_BLACK, COLOR_MENU_SELECT);
  } else {
    tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  }
  tft.setTextSize(2);
  tft.setCursor(option1X, optionY);
  tft.print("Reset Timer");
  
  // Option 2: Exit
  if (selectedOption == 1) {
    tft.fillRect(option2X - 5, optionY - 5, 100, 35, COLOR_MENU_SELECT);
    tft.setTextColor(TFT_BLACK, COLOR_MENU_SELECT);
  } else {
    tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  }
  tft.setTextSize(2);
  tft.setCursor(option2X, optionY);
  tft.print("Exit");
}
