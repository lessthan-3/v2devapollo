#include "display_ui.h"
#include "motor_control.h"
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

void drawStaticUI() {
  tft.drawFastHLine(0, STATUS_ZONE_Y - 5, SCREEN_WIDTH, TFT_DARKGREY);
  tft.drawFastVLine(SCREEN_WIDTH / 2, 0, STATUS_ZONE_Y - 5, TFT_DARKGREY);

  tft.setTextColor(COLOR_LABEL, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(LEFT_COLUMN_X + 50, 15);
  tft.print("TARGET");

  tft.setCursor(RIGHT_COLUMN_X + 40, 15);
  tft.print("CURRENT");

  tft.setTextSize(1);
  tft.setCursor(LEFT_COLUMN_X, STATUS_ZONE_Y + 5);
  tft.print("TEMP");

  tft.setCursor(LEFT_COLUMN_X + 80, STATUS_ZONE_Y + 5);
  tft.print("MOTOR");

  tft.setCursor(LEFT_COLUMN_X + 140, STATUS_ZONE_Y + 5);
  tft.print("AC Hz");

  tft.setCursor(LEFT_COLUMN_X + 200, STATUS_ZONE_Y + 5);
  tft.print("LOOP ms");

  tft.setCursor(LEFT_COLUMN_X, STATUS_ZONE_Y - 20);
  tft.print("ZC cnt");

  tft.setCursor(LEFT_COLUMN_X + 280, STATUS_ZONE_Y + 5);
  tft.print("PID: Kp / Ki / Kd");
}

void drawTargetPressure(bool forceRedraw) {
  static float lastDisplayedTarget = -1.0f;

  if (!forceRedraw && targetPsi == lastDisplayedTarget && !showingMinMax) {
    return;
  }

  lastDisplayedTarget = targetPsi;

  tft.fillRect(LEFT_COLUMN_X, 50, COLUMN_WIDTH, 150, COLOR_BG);

  if (showingMinMax) {
    tft.setTextColor(COLOR_MINMAX, COLOR_BG);
    tft.setTextSize(5);
    tft.setCursor(LEFT_COLUMN_X + 40, 90);
    tft.print(isMinNotMax ? "MIN" : "MAX");
  } else {
    tft.setTextColor(COLOR_TARGET, COLOR_BG);
    tft.setTextSize(6);
    tft.setCursor(LEFT_COLUMN_X + 20, 70);

    char psiStr[10];
    snprintf(psiStr, sizeof(psiStr), "%4.1f", targetPsi);
    tft.print(psiStr);

    tft.setTextSize(3);
    tft.setCursor(LEFT_COLUMN_X + 60, 140);
    tft.print("PSI");
  }
}

void drawCurrentPressure(float psi, bool valid) {
  static float lastDisplayedCurrent = -999.0f;
  static bool lastValid = true;
  static bool labelDrawn = false;

  if (valid == lastValid && valid && fabsf(psi - lastDisplayedCurrent) < 0.1f) {
    return;
  }

  bool validityChanged = (valid != lastValid);
  lastDisplayedCurrent = psi;
  lastValid = valid;

  if (!valid) {
    tft.fillRect(RIGHT_COLUMN_X, 50, COLUMN_WIDTH, 150, COLOR_BG);
    tft.setTextColor(TFT_RED, COLOR_BG);
    tft.setTextSize(4);
    tft.setCursor(RIGHT_COLUMN_X + 30, 90);
    tft.print("ERROR");
    labelDrawn = false;
  } else {
    tft.fillRect(RIGHT_COLUMN_X + 5, 65, 210, 70, COLOR_BG);

    tft.setTextColor(COLOR_CURRENT, COLOR_BG);
    tft.setTextSize(6);
    tft.setCursor(RIGHT_COLUMN_X + 10, 70);

    char psiStr[10];
    snprintf(psiStr, sizeof(psiStr), "%5.1f", psi);
    tft.print(psiStr);

    if (!labelDrawn || validityChanged) {
      tft.fillRect(RIGHT_COLUMN_X + 45, 135, 60, 30, COLOR_BG);
      tft.setTextSize(3);
      tft.setCursor(RIGHT_COLUMN_X + 50, 140);
      tft.print("PSI");
      labelDrawn = true;
    }
  }
}

void drawTemperature(float tempC) {
  static float lastTemp = -999.0f;

  if (fabsf(tempC - lastTemp) < 0.5f && lastTemp != -999.0f) {
    return;
  }
  lastTemp = tempC;
  currentTemperatureC = tempC;

  tft.fillRect(LEFT_COLUMN_X, STATUS_ZONE_Y + 18, 65, 30, COLOR_BG);

  tft.setTextColor(COLOR_TEMP, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(LEFT_COLUMN_X, STATUS_ZONE_Y + 22);

  if (tempC < -100.0f) {
    tft.print("--");
  } else {
    float tempF = (tempC * 9.0f / 5.0f) + 32.0f;
    char tempStr[10];
    snprintf(tempStr, sizeof(tempStr), "%3.0f", tempF);
    tft.print(tempStr);
  }
  tft.setTextSize(1);
  tft.print("F");
}

void drawDebugInfo() {
  static uint8_t lastMotorSpeed = 255;
  static AcFrequency lastAcFreq = AC_FREQ_UNKNOWN;
  static unsigned long lastLoopTime = 9999;
  static float lastKp = -1, lastKi = -1, lastKd = -1;
  static uint32_t lastZcCount = 0xFFFFFFFF;

  uint8_t motorSpeed = motorState.motorSpeed;
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
    tft.printf("%3d%%", motorSpeed);
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

  tft.setTextColor(TFT_BLACK, TFT_YELLOW);
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

void flashMinMax(bool isMin) {
  showingMinMax = true;
  isMinNotMax = isMin;
  minMaxFlashStart = millis();
  drawTargetPressure(true);
}

void updateMinMaxFlash() {
  if (showingMinMax && (millis() - minMaxFlashStart >= MINMAX_FLASH_MS)) {
    showingMinMax = false;
    drawTargetPressure(true);
  }
}

void drawStartupScreen() {
  tft.fillScreen(COLOR_BG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(120, 80);
  tft.println("APOLLO");
  tft.setCursor(100, 120);
  tft.println("SPRAYERS");
  tft.setTextSize(2);
  tft.setCursor(160, 170);
  tft.println("HVLP");
  tft.setTextSize(1);
  tft.setCursor(175, 220);
  tft.println("Starting...");
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

void drawMenuScreen(uint8_t menuIndex, bool forceRedraw) {
  static uint8_t lastMenuIndex = 255;
  if (!forceRedraw && lastMenuIndex == menuIndex) {
    return;
  }
  lastMenuIndex = menuIndex;

  tft.fillScreen(COLOR_BG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(185, 30);
  tft.print("MENU");

  const char* options[MENU_OPTION_COUNT] = {"Start motor", "Settings", "Firmware version"};
  for (uint8_t i = 0; i < MENU_OPTION_COUNT; i++) {
    int y = MENU_TOP_Y + (i * MENU_OPTION_HEIGHT);
    if (i == menuIndex) {
      tft.fillRect(40, y - 5, SCREEN_WIDTH - 80, MENU_OPTION_HEIGHT, TFT_DARKGREY);
      tft.setTextColor(TFT_BLACK, TFT_DARKGREY);
    } else {
      tft.setTextColor(TFT_WHITE, COLOR_BG);
    }
    tft.setTextSize(2);
    tft.setCursor(60, y);
    tft.print(options[i]);
  }

  drawMenuFooter("Press to select", TFT_GREEN);
}

void drawRuntimeStatic() {
  tft.fillScreen(COLOR_BG);

  tft.drawFastVLine(CELL_WIDTH, 0, SCREEN_HEIGHT, TFT_DARKGREY);
  tft.drawFastVLine(RUNTIME_RIGHT_X, 0, SCREEN_HEIGHT, TFT_DARKGREY);
  tft.drawFastHLine(0, CELL_HEIGHT, SCREEN_WIDTH, TFT_DARKGREY);
  tft.drawFastHLine(0, RUNTIME_FOOTER_Y, SCREEN_WIDTH, TFT_DARKGREY);

  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("TARGET PSI");

  tft.setTextSize(2);
  tft.setCursor(RUNTIME_RIGHT_X + 10, 10);
  tft.print("TEMP");

  tft.setCursor(RUNTIME_RIGHT_X + 10, CELL_HEIGHT + 10);
  tft.print("PAUSE IN");
}

void drawRuntimeTarget(float target, float current, bool valid) {
  static float lastTarget = -999.0f;
  static float lastCurrent = -999.0f;
  static bool lastValid = false;
  static uint16_t lastColor = 0;

  uint16_t color = TFT_YELLOW;
  if (!valid) {
    color = TFT_DARKGREY;
  } else {
    float diff = fabsf(current - target);
    if (diff > 1.0f) {
      color = TFT_RED;
    } else if (diff > 0.5f) {
      color = TFT_ORANGE;
    } else if (diff <= 0.1f) {
      color = TFT_GREEN;
    }
  }

  if (fabsf(target - lastTarget) < 0.05f && fabsf(current - lastCurrent) < 0.05f && valid == lastValid && color == lastColor) {
    return;
  }

  lastTarget = target;
  lastCurrent = current;
  lastValid = valid;
  lastColor = color;

  tft.fillRect(0, 40, RUNTIME_RIGHT_X, RUNTIME_TOP_HEIGHT - 40, COLOR_BG);

  tft.setTextColor(color, COLOR_BG);
  tft.setTextSize(6);
  tft.setCursor(20, 80);
  if (!valid) {
    tft.print("--.-");
  } else {
    char psiStr[10];
    snprintf(psiStr, sizeof(psiStr), "%4.1f", target);
    tft.print(psiStr);
  }

  tft.setTextSize(3);
  tft.setCursor(100, 150);
  tft.print("PSI");
}

void drawRuntimeTemperature(float tempC) {
  static float lastTemp = -999.0f;
  if (fabsf(tempC - lastTemp) < 0.5f && lastTemp != -999.0f) {
    return;
  }
  lastTemp = tempC;

  tft.fillRect(RUNTIME_RIGHT_X + 2, 30, CELL_WIDTH - 4, CELL_HEIGHT - 32, COLOR_BG);

  tft.setTextColor(COLOR_TEMP, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(RUNTIME_RIGHT_X + 20, 50);

  if (tempC < -100.0f) {
    tft.print("--");
  } else {
    float tempF = (tempC * 9.0f / 5.0f) + 32.0f;
    char tempStr[8];
    snprintf(tempStr, sizeof(tempStr), "%3.0f", tempF);
    tft.print(tempStr);
  }
  tft.setTextSize(2);
  tft.print("F");
}

void drawRuntimePauseCountdown(uint32_t secondsRemaining) {
  static uint32_t lastSeconds = 0xFFFFFFFF;
  if (secondsRemaining == lastSeconds) {
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
  tft.fillRect(0, RUNTIME_FOOTER_Y + 2, SCREEN_WIDTH, CELL_HEIGHT - 4, COLOR_BG);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(20, RUNTIME_FOOTER_Y + 35);
  tft.print("PRESS TO PAUSE AND SWAP TO MENU");
}
