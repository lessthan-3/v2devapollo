#include "display_ui.h"
#include "motor_control.h"
#include "splash_image.h"
#include "qr_images.h"
#include "config.h"
#include <math.h>
#include <stdint.h>

extern bool overTempActive;
extern bool overTempWarning;
extern bool overTempShutdown;

// Old UI functions removed - using new Runtime UI design

void drawStartupScreen() {
  tft.fillScreen(COLOR_BG);

  // Scale the splash image to fill the full screen height (320px), maintaining aspect ratio.
  // SPLASH_IMAGE is square so scaled size = SCREEN_HEIGHT x SCREEN_HEIGHT, centred horizontally.
  const int destH = SCREEN_HEIGHT;
  const int destW = SCREEN_HEIGHT;  // square source → square dest
  const int destX = (SCREEN_WIDTH - destW) / 2;

  // The image is strictly two colours: logo pixels (any non-zero alpha) and background.
  // Hardcode the logo colour as pure TFT_RED to guarantee correct output regardless
  // of any encoding artefacts in the stored pixel data.
  const uint16_t LOGO_COLOR = TFT_RED;

  tft.startWrite();
  tft.setWindow(destX, 0, destX + destW - 1, destH - 1);
  for (int dy = 0; dy < destH; dy++) {
    int sy = (dy * SPLASH_IMAGE_HEIGHT) / destH;
    for (int dx = 0; dx < destW; dx++) {
      int sx = (dx * SPLASH_IMAGE_WIDTH) / destW;
      int idx = sy * SPLASH_IMAGE_WIDTH + sx;
      uint8_t a = pgm_read_byte(&splashAlpha[idx]);
      tft.pushColor(a ? LOGO_COLOR : COLOR_BG);
    }
  }
  tft.endWrite();
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

// ---------------------------------------------------------------------------
// Menu icon helpers
// Each draws a ~14px-radius icon centred at (cx, cy) in the given colour.
// ---------------------------------------------------------------------------

// ⚙  Cog / gear icon – ring with 8 evenly spaced rectangular teeth
static void drawIconCog(int cx, int cy, int r, uint16_t color) {
  // Outer ring (teeth approximated as short filled rects around the circle)
  tft.drawCircle(cx, cy, r - 4, color);
  tft.drawCircle(cx, cy, r - 5, color);
  // 8 teeth: short rectangles projected outward from the ring
  const float PI2 = 6.28318f;
  for (uint8_t t = 0; t < 8; t++) {
    float angle = (PI2 / 8.0f) * t;
    float ca = cosf(angle), sa = sinf(angle);
    int x0 = cx + (int)roundf(ca * (r - 6));
    int y0 = cy + (int)roundf(sa * (r - 6));
    int x1 = cx + (int)roundf(ca * r);
    int y1 = cy + (int)roundf(sa * r);
    tft.drawLine(x0, y0, x1, y1, color);
    // Widen the tooth by one pixel each side
    float px = -sa, py = ca;
    tft.drawLine(x0 + (int)roundf(px), y0 + (int)roundf(py),
                 x1 + (int)roundf(px), y1 + (int)roundf(py), color);
  }
  // Small centre hole
  tft.fillCircle(cx, cy, r / 4, color);
  tft.fillCircle(cx, cy, r / 4 - 2, COLOR_BG);
}

// ?  Question-mark-in-circle icon
static void drawIconQuestion(int cx, int cy, int r, uint16_t color) {
  tft.drawCircle(cx, cy, r,     color);
  tft.drawCircle(cx, cy, r - 1, color);
  // "?" stem and dot drawn with primitives
  // Curve of the ?  – small arc at top
  tft.drawCircle(cx, cy - r / 4, r / 3, color);
  // Erase the bottom half of that arc so it looks like a hook
  tft.fillRect(cx - r / 3 - 1, cy - r / 4, (r / 3) * 2 + 2, r / 3 + 2, COLOR_BG);
  // Re-draw outer ring over the fill
  tft.drawCircle(cx, cy, r,     color);
  tft.drawCircle(cx, cy, r - 1, color);
  // Vertical stem drop
  tft.fillRect(cx - 1, cy - r / 4 + r / 3 - 2, 3, r / 4, color);
  // Dot
  tft.fillRect(cx - 1, cy + r / 4 + 1, 3, 3, color);
}

// i  Info-in-circle icon
static void drawIconInfo(int cx, int cy, int r, uint16_t color) {
  tft.drawCircle(cx, cy, r,     color);
  tft.drawCircle(cx, cy, r - 1, color);
  // Top dot of the "i"
  tft.fillRect(cx - 1, cy - r / 2 - 1, 3, 3, color);
  // Vertical bar of the "i"
  tft.fillRect(cx - 1, cy - r / 4, 3, r / 2 + 1, color);
  // Small serif base
  tft.fillRect(cx - 3, cy + r / 4 + 1, 7, 2, color);
}

// ▶  Play / triangle icon (filled)
static void drawIconPlay(int cx, int cy, int r, uint16_t color) {
  // Vertices: tip at right, flat vertical edge at left
  int tipX  = cx + r;
  int leftX = cx - r / 2;
  int topY  = cy - r;
  int botY  = cy + r;
  // Scanline fill: for each row interpolate the right edge between
  // (leftX, topY) -> (tipX, cy)  for the upper half, and
  // (tipX, cy)    -> (leftX, botY) for the lower half.
  for (int row = topY; row <= botY; row++) {
    int rx;
    if (row <= cy) {
      float t = (cy - topY > 0) ? (float)(row - topY) / (float)(cy - topY) : 1.0f;
      rx = leftX + (int)roundf(t * (tipX - leftX));
    } else {
      float t = (botY - cy > 0) ? (float)(row - cy)  / (float)(botY - cy)  : 1.0f;
      rx = tipX + (int)roundf(t * (leftX - tipX));
    }
    tft.drawFastHLine(leftX, row, rx - leftX + 1, color);
  }
}

// 🕐  Clock icon – circle with hour/minute hands
static void drawIconClock(int cx, int cy, int r, uint16_t color) {
  tft.drawCircle(cx, cy, r,     color);
  tft.drawCircle(cx, cy, r - 1, color);
  // Minute hand: pointing to 12 (straight up)
  tft.drawLine(cx, cy, cx, cy - (r - 3), color);
  // Hour hand: pointing to ~3 (straight right, shorter)
  tft.drawLine(cx, cy, cx + (r - 5), cy, color);
  // Centre dot
  tft.fillCircle(cx, cy, 2, color);
}

// Draw the appropriate icon for a given menu index
static void drawMenuIcon(uint8_t menuIdx, int cx, int cy, uint16_t color) {
  switch (menuIdx) {
    case 0: drawIconPlay(cx, cy, 11, color);     break;  // Start Motor
    case 1: drawIconCog(cx, cy, 13, color);      break;  // Settings
    case 2: drawIconClock(cx, cy, 13, color);    break;  // Timers
    case 3: drawIconQuestion(cx, cy, 13, color); break;  // Support
    case 4: drawIconInfo(cx, cy, 13, color);     break;  // About
    default: break;
  }
}

// ---------------------------------------------------------------------------

void drawMenuScreen(uint8_t menuIndex, bool forceRedraw) {
  static uint8_t lastMenuIndex = 255;
  if (!forceRedraw && lastMenuIndex == menuIndex) {
    return;
  }
  lastMenuIndex = menuIndex;

  tft.fillScreen(COLOR_BG);

  // Title — built-in font, size 3
  tft.setFreeFont(nullptr);
  tft.setTextSize(3);
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  int16_t titleW = 9 * 18;  // "MAIN MENU" 9 chars * 18px per char at size 3
  tft.setCursor((SCREEN_WIDTH - titleW) / 2, 14);
  tft.print("MAIN MENU");

  // Icon gutter width and text area bounds
  // Icons are centred at x=20 in a 40px gutter; text area starts at x=50
  const int iconGutter = 50;
  const int textAreaX  = iconGutter;
  const int textAreaW  = SCREEN_WIDTH - iconGutter - 40;  // 40px right margin

  const char* options[MENU_OPTION_COUNT] = {"Start Motor", "Settings", "Timers", "Support", "About"};

  for (uint8_t i = 0; i < MENU_OPTION_COUNT; i++) {
    int y    = MENU_TOP_Y + (i * MENU_OPTION_HEIGHT);
    int rowCy = y + (MENU_OPTION_HEIGHT / 2) - 5;  // icon vertical centre
    bool selected = (i == menuIndex);

    // Row background
    if (selected) {
      tft.fillRect(textAreaX, y - 5, SCREEN_WIDTH - textAreaX, MENU_OPTION_HEIGHT, COLOR_MENU_SELECT);
      tft.fillRect(0, y - 5, textAreaX, MENU_OPTION_HEIGHT, COLOR_BG);
    } else {
      tft.fillRect(0, y - 5, SCREEN_WIDTH, MENU_OPTION_HEIGHT, COLOR_BG);
    }

    // Icon — always on black background
    uint16_t iconColor = selected ? COLOR_TEXT_PRIMARY : COLOR_TEXT_SECONDARY;
    drawMenuIcon(i, 20, rowCy, iconColor);

    // Text — built-in font size 2, colour changes for selected row
    uint16_t fg = selected ? TFT_BLACK : COLOR_TEXT_PRIMARY;
    uint16_t bg = selected ? COLOR_MENU_SELECT : COLOR_BG;

    tft.setFreeFont(nullptr);
    tft.setTextSize(2);
    tft.setTextColor(fg, bg);

    int16_t tw = strlen(options[i]) * 12;  // 12px per char at size 2
    int tx = textAreaX + (textAreaW - tw) / 2;
    int ty = y - 5 + (MENU_OPTION_HEIGHT - 16) / 2;  // 16px tall at size 2
    tft.setCursor(tx, ty);
    tft.print(options[i]);
  }

  tft.setTextSize(1);
  drawMenuFooter("Press to select", COLOR_SUCCESS);
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
  tft.print("SETTINGS");

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
  const char* options[4] = {"PowerPause Timeout", "Audible Beeper", "Units", "Exit"};
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
    uint16_t pct = (uint16_t)((uint32_t)pauseSeconds * 100u / POWER_PAUSE_PCT_BASE);
    tft.setCursor(SCREEN_WIDTH - 110, y);
    tft.printf("%3u%%", pct);
  } else if (settingsIndex == 1) {
    tft.setCursor(SCREEN_WIDTH - 100, y);
    tft.print(beeperEnabled ? "ON" : "OFF");
  } else if (settingsIndex == 2) {
    tft.setCursor(SCREEN_WIDTH - 150, y);
    tft.print(units == UNITS_IMPERIAL ? "IMPERIAL" : "  METRIC");
  }
}

// Old drawSettingsRow removed - using drawPowerPauseSettingsRow for Settings screen

// Runtime screen layout constants
// Screen: 480 x 320 landscape
// Top zone    (1/2) : y=0   .. THIRD_1_Y   (pressure)   160 px
// Middle zone (3/8) : y=THIRD_1_Y .. THIRD_2_Y  (motor power)  120 px
// Bottom zone (1/8) : y=THIRD_2_Y .. 320  (info bar)     40 px
#define THIRD_1_Y  160
#define THIRD_2_Y  280

void drawRuntimeStatic(DisplayUnits units) {
  tft.fillScreen(COLOR_BG);

  // Divider between top third and middle third
  tft.drawFastHLine(0, THIRD_1_Y, SCREEN_WIDTH, TFT_DARKGREY);

  // Divider between middle third and bottom third
  tft.drawFastHLine(0, THIRD_2_Y, SCREEN_WIDTH, TFT_DARKGREY);

  // ---- Top zone label: SYSTEM PRESSURE (centred just above divider) ----
  tft.setTextFont(1);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_LABEL, COLOR_BG);
  const char* topLabel = "SYSTEM PRESSURE";
  int topLabelW = strlen(topLabel) * 12;
  tft.setCursor((SCREEN_WIDTH - topLabelW) / 2, THIRD_1_Y - 22);
  tft.print(topLabel);

  // ---- Middle zone label: TURBINE OUTPUT % (centred just above divider) ----
  tft.setTextSize(2);
  tft.setTextColor(COLOR_LABEL, COLOR_BG);
  const char* midLabel = "TURBINE OUTPUT %";
  int midLabelW = strlen(midLabel) * 12;
  tft.setCursor((SCREEN_WIDTH - midLabelW) / 2, THIRD_2_Y - 22);
  tft.print(midLabel);

  // ---- Bottom zone: instruction line (top of zone) ----
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  const char* instrText = "  TURN TO SET PRESSURE, PRESS FOR MENU";
  int instrW = strlen(instrText) * 6;
  tft.setCursor(5, THIRD_2_Y + 4);
  tft.print(instrText);
}

void drawRuntimeTarget(float target, float current, DisplayUnits units, bool valid, bool forceRedraw, uint16_t motorSpeed) {
  static float lastTarget = -999.0f;
  static float lastCurrent = -999.0f;
  static bool lastValid = false;
  static uint16_t lastColor = 0;
  static DisplayUnits lastUnits = UNITS_IMPERIAL;
  static bool lastShowActual = false;
  static uint32_t targetChangedAt = 0;

  // Track when the set pressure changes — show set pressure in green for 1 second
  if (fabsf(target - lastTarget) >= 0.05f && lastTarget != -999.0f) {
    targetChangedAt = millis();
  }
  bool showingSetPreview = (targetChangedAt != 0) && (millis() - targetChangedAt < 1000UL);

  // Determine if we should show actual pressure in orange:
  // motor at 100% power and set pressure is more than 0.5 PSI away from actual,
  // but not during the 1-second preview window after the user changes the set pressure
  bool showActual = !showingSetPreview && (motorSpeed >= 100) && valid && (target > 0.05f) &&
                    (fabsf(target - current) > 0.5f);

  // Convert to display units
  float displayTarget = target;
  float displayCurrent = current;
  if (units == UNITS_METRIC) {
    displayTarget  = 200.0f + (target  - 3.0f) * 70.833f;
    displayCurrent = 200.0f + (current - 3.0f) * 70.833f;
    if (displayTarget  < 0.0f) displayTarget  = 0.0f;
    if (displayCurrent < 0.0f) displayCurrent = 0.0f;
  }

  // Color: orange when at max power and off-target, otherwise green (or grey when inactive)
  uint16_t color = showActual ? COLOR_WARNING : TFT_GREEN;
  if (target <= 0.05f) color = TFT_DARKGREY;          // motor off / dialled to zero
  else if (!valid) color = TFT_DARKGREY;

  static bool lastShowingSetPreview = false;

  if (!forceRedraw && fabsf(target - lastTarget) < 0.05f &&
      fabsf(current - lastCurrent) < 0.05f &&
      valid == lastValid && color == lastColor && units == lastUnits &&
      showActual == lastShowActual && showingSetPreview == lastShowingSetPreview) {
    return;
  }

  lastTarget            = target;
  lastCurrent           = current;
  lastValid             = valid;
  lastColor             = color;
  lastUnits             = units;
  lastShowActual        = showActual;
  lastShowingSetPreview = showingSetPreview;

  // Clear the top zone value area (above the static label row)
  // Label row starts at THIRD_1_Y-22; clear from y=0 to just above it
  tft.fillRect(0, 0, SCREEN_WIDTH, THIRD_1_Y - 24, COLOR_BG);

  // Usable drawing height (above the label strip)
  int zoneH = THIRD_1_Y - 24;

  if (target >= MAX_PSI_THRESHOLD) {
    // MAX mode: single centred "MAX" label
    tft.setFreeFont(&FreeSansBold24pt7b);
    tft.setTextSize(2);
    tft.setTextColor(color, COLOR_BG);
    int16_t w = tft.textWidth("MAX");
    int16_t h = tft.fontHeight();
    tft.setCursor((SCREEN_WIDTH - w) / 2, (zoneH + h) / 2 - 15);
    tft.print("MAX");
  } else {
    // Build numeric string: show actual pressure in orange when at max power and off-target,
    // otherwise show set pressure in green
    float displayValue = showActual ? displayCurrent : displayTarget;
    char numStr[8];
    char unitStr[8];
    if (units == UNITS_IMPERIAL) {
      // Clamp to 00.0 – 99.9
      float v = displayValue;
      if (v < 0.0f)  v = 0.0f;
      if (v > 99.9f) v = 99.9f;
      snprintf(numStr,  sizeof(numStr),  "%04.1f", v);  // e.g. "00.0"
      snprintf(unitStr, sizeof(unitStr), "PSI");
    } else {
      float v = displayValue;
      if (v < 0.0f)   v = 0.0f;
      if (v > 9999.0f) v = 9999.0f;
      snprintf(numStr,  sizeof(numStr),  "%.0f", v);
      snprintf(unitStr, sizeof(unitStr), "mbar");
    }

    // --- Measure the large number ---
    tft.setFreeFont(&FreeSansBold24pt7b);
    tft.setTextSize(2);
    int16_t numW = tft.textWidth(numStr);
    int16_t numH = tft.fontHeight();

    // --- Measure the small unit label (half the text size) ---
    tft.setTextSize(1);
    int16_t unitW = tft.textWidth(unitStr);
    int16_t unitH = tft.fontHeight();

    // Total combined width, with a small gap between number and unit
    const int gap = 110;
    int16_t totalW = numW + gap + unitW;

    // Horizontal starting position so the pair is centred
    int startX = (SCREEN_WIDTH - totalW) / 2;

    // Vertical: baseline centred in the usable zone
    int baselineY = (zoneH + numH) / 2 - 10;

    // Draw the large number
    tft.setFreeFont(&FreeSansBold24pt7b);
    tft.setTextSize(3);
    tft.setTextColor(color, COLOR_BG);
    tft.setCursor(startX + 40, baselineY);
    tft.print(numStr);

    // Draw the small unit label aligned to the bottom of the large number
    tft.setTextSize(1);
    tft.setTextColor(color, COLOR_BG);
    // Align unit baseline to same baseline as the number
    tft.setCursor(startX + numW + gap + 20, baselineY);
    tft.print(unitStr);
  }

  // Reset font
  tft.setTextFont(1);
  tft.setTextSize(1);
}

void drawRuntimeJobTime(uint32_t jobTimeSeconds, bool forceRedraw) {
  static uint32_t lastJobTime = 0xFFFFFFFF;
  if (!forceRedraw && jobTimeSeconds == lastJobTime) {
    return;
  }
  lastJobTime = jobTimeSeconds;

  // Bottom zone — right half: hours
  const int infoLineY = THIRD_2_Y + 22;
  tft.fillRect(SCREEN_WIDTH / 2, infoLineY, SCREEN_WIDTH / 2, 16, COLOR_BG);

  uint32_t hours   = jobTimeSeconds / 3600;
  uint32_t minutes = (jobTimeSeconds % 3600) / 60;

  char buf[24];
  snprintf(buf, sizeof(buf), "HOURS  %03lu:%02lu",
           (unsigned long)hours, (unsigned long)minutes);

  tft.setTextFont(1);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  // Left edge sits ~1/8 screen (60px) right of centre
  tft.setCursor(SCREEN_WIDTH / 2 + 60, infoLineY);
  tft.print(buf);
  tft.setTextSize(1);
}

void drawRuntimeTemperature(float tempC, DisplayUnits units, bool forceRedraw) {
  static float lastTemp = -999.0f;
  static DisplayUnits lastUnits = UNITS_IMPERIAL;
  static bool lastWarning  = false;
  static bool lastShutdown = false;
  if (!forceRedraw && fabsf(tempC - lastTemp) < 0.5f && lastTemp != -999.0f && units == lastUnits
      && lastWarning == overTempWarning && lastShutdown == overTempShutdown) {
    return;
  }
  lastTemp     = tempC;
  lastUnits    = units;
  lastWarning  = overTempWarning;
  lastShutdown = overTempShutdown;

  // Bottom zone — left half: temperature
  const int infoLineY = THIRD_2_Y + 22;
  tft.fillRect(0, infoLineY, SCREEN_WIDTH / 2, 16, COLOR_BG);

  char buf[32];
  if (overTempShutdown) {
    snprintf(buf, sizeof(buf), "TEMP  SHUTDOWN");
  } else if (overTempWarning) {
    if (tempC < -100.0f) {
      snprintf(buf, sizeof(buf), "TEMP  WARN");
    } else {
      if (units == UNITS_IMPERIAL) {
        float tempF = (tempC * 9.0f / 5.0f) + 32.0f;
        snprintf(buf, sizeof(buf), "TEMP  %3.0f F!", tempF);
      } else {
        snprintf(buf, sizeof(buf), "TEMP  %3.0f C!", tempC);
      }
    }
  } else if (tempC < -100.0f) {
    snprintf(buf, sizeof(buf), "TEMP  ---");
  } else {
    if (units == UNITS_IMPERIAL) {
      float tempF = (tempC * 9.0f / 5.0f) + 32.0f;
      snprintf(buf, sizeof(buf), "TEMP  %3.0f F", tempF);
    } else {
      snprintf(buf, sizeof(buf), "TEMP  %3.0f C", tempC);
    }
  }

  tft.setTextFont(1);
  tft.setTextSize(2);
  uint16_t tempTextColor = COLOR_TEXT_SECONDARY;
  if (overTempShutdown)     tempTextColor = COLOR_TEMP_WARNING;
  else if (overTempWarning) tempTextColor = COLOR_WARNING;
  tft.setTextColor(tempTextColor, COLOR_BG);
  // Right-align so the text ends ~1/8 screen (60px) from the centre divider
  {
    int16_t tw = strlen(buf) * 12;  // ~12px per char at size 2
    int16_t tx = (SCREEN_WIDTH / 2) - 60 - tw;
    if (tx < 0) tx = 0;
    tft.setCursor(tx, infoLineY);
  }
  tft.print(buf);
  tft.setTextSize(1);
}

void drawRuntimeMotorPower(uint16_t motorSpeed, bool forceRedraw) {
  // motorSpeed is 0-1000 (matches motor_control scaling); convert to 0-100%
  static uint16_t lastSpeed = 0xFFFF;
  if (!forceRedraw && motorSpeed == lastSpeed) {
    return;
  }
  lastSpeed = motorSpeed;

  // Clear middle third value area (above the static label row)
  tft.fillRect(0, THIRD_1_Y + 1, SCREEN_WIDTH, (THIRD_2_Y - 24) - THIRD_1_Y - 1, COLOR_BG);

  // Clamp to 0-100 for display
  uint16_t pct = (motorSpeed > 1000) ? 100 : (motorSpeed / 10);

  char buf[8];
  snprintf(buf, sizeof(buf), "%3u%%", pct);

  tft.setFreeFont(&FreeSansBold18pt7b);
  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN, COLOR_BG);

  int16_t w = tft.textWidth(buf);
  int16_t h = tft.fontHeight();

  int zoneTop = THIRD_1_Y + 1;
  int zoneBot = THIRD_2_Y - 24;  // leave room for static label
  int zoneH   = zoneBot - zoneTop;

  int x = (SCREEN_WIDTH - w) / 2;
  int y = zoneTop + (zoneH + h) / 2 - 15;  // vertically centre baseline
  tft.setCursor(x, y);
  tft.print(buf);

  tft.setTextFont(1);
  tft.setTextSize(1);
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

// ---------------------------------------------------------------------------
// Helper: draw a filled warning triangle "!" centred at (cx, cy)
// ---------------------------------------------------------------------------
static void drawWarningTriangle(int cx, int cy, int size, uint16_t color) {
  for (int row = 0; row < size; row++) {
    int halfW = (row * size / 2) / size + 1;
    tft.drawFastHLine(cx - halfW, cy - size / 2 + row, halfW * 2 + 1, color);
  }
  int excX    = cx;
  int excTopY = cy - size / 2 + size / 5;
  int excBotY = cy + size / 2 - size / 5;
  tft.fillRect(excX - 1, excTopY,    3, (excBotY - excTopY) * 2 / 3, TFT_BLACK);
  tft.fillRect(excX - 1, excBotY - 2, 3, 3,                           TFT_BLACK);
}

void drawRuntimePowerPauseOverlay(IdleState idleState, uint32_t secondsRemaining, bool forceRedraw) {
  static IdleState lastState            = IDLE_STATE_OFF;
  static uint32_t  lastSecondsRemaining = 0xFFFFFFFF;

  if (!forceRedraw && idleState == lastState && secondsRemaining == lastSecondsRemaining) {
    return;
  }

  const bool stateChanged = forceRedraw || (idleState != lastState);
  lastState            = idleState;
  lastSecondsRemaining = secondsRemaining;

  if (idleState == IDLE_STATE_OFF) {
    return;
  }

  // ---- Overlay geometry — full-width, centred vertically ----
  const int OX = 10;
  const int OY = 40;
  const int OW = SCREEN_WIDTH - (OX * 2);   // 460
  const int OH = SCREEN_HEIGHT - (OY * 2);  // 240

  const int bannerH = OH / 2;
  const int botY    = OY + bannerH;
  const int botH    = OH - bannerH;

  // ---- Static chrome: only redrawn when the idle state changes ----
  if (stateChanged) {
    // Outer border
    tft.fillRect(OX, OY, OW, OH, TFT_BLACK);
    for (int t = 0; t < 3; t++) {
      tft.drawRect(OX + t, OY + t, OW - t * 2, OH - t * 2, TFT_RED);
    }

    // Top red banner
    tft.fillRect(OX + 3, OY + 3, OW - 6, bannerH - 3, TFT_RED);

    // Warning triangles
    int triY  = OY + 3 + (bannerH - 3) / 2;
    int triSz = 22;
    drawWarningTriangle(OX + 28,      triY, triSz, TFT_YELLOW);
    drawWarningTriangle(OX + OW - 28, triY, triSz, TFT_YELLOW);

    // "ATTENTION" title
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setFreeFont(nullptr);
    tft.setTextSize(3);
    {
      const char* title = "ATTENTION";
      int tw = strlen(title) * 18;
      tft.setCursor(OX + (OW - tw) / 2, OY + 10);
      tft.print(title);
    }

    // Sub-title line
    tft.setTextSize(2);
    {
      const char* sub = "PowerPause Active";
      int tw = strlen(sub) * 12;
      tft.setCursor(OX + (OW - tw) / 2, OY + 48);
      tft.print(sub);
    }

    // Bottom white section
    tft.fillRect(OX + 3, botY, OW - 6, botH - 3, TFT_WHITE);

    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.setFreeFont(nullptr);

    if (idleState == IDLE_STATE_PID_RAMP) {
      tft.setTextSize(2);
      const char* l1 = "Motor ramping to idle speed";
      tft.setCursor(OX + (OW - (int)strlen(l1) * 12) / 2, botY + (botH / 2) - 8);
      tft.print(l1);
    } else if (idleState == IDLE_STATE_HOLD) {
      const char* l1 = "Turn dial or press button";
      const char* l2 = "to resume motor";
      tft.setTextSize(2);
      int lineH = 24;
      if (secondsRemaining != UINT32_MAX) {
        int totalH = lineH + lineH + 38;
        int startY = botY + (botH - totalH) / 2;
        tft.setCursor(OX + (OW - (int)strlen(l1) * 12) / 2, startY);
        tft.print(l1);
        tft.setCursor(OX + (OW - (int)strlen(l2) * 12) / 2, startY + lineH);
        tft.print(l2);
      } else {
        int startY = botY + (botH / 2) - lineH;
        tft.setCursor(OX + (OW - (int)strlen(l1) * 12) / 2, startY);
        tft.print(l1);
        tft.setCursor(OX + (OW - (int)strlen(l2) * 12) / 2, startY + lineH);
        tft.print(l2);
      }
    }
  }

  // ---- Countdown clock: redrawn every tick, only when a timer is active ----
  if (idleState == IDLE_STATE_HOLD && secondsRemaining != UINT32_MAX) {
    int lineH  = 24;
    int totalH = lineH + lineH + 38;
    int startY = botY + (botH - totalH) / 2;
    int clockY = startY + lineH * 2 + 6;

    // Erase only the clock row
    tft.fillRect(OX + 3, clockY - 2, OW - 6, 36, TFT_WHITE);

    uint32_t minutes = secondsRemaining / 60;
    uint32_t secs    = secondsRemaining % 60;
    char timeStr[10];
    snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu", (unsigned long)minutes, (unsigned long)secs);
    tft.setTextColor(TFT_RED, TFT_WHITE);
    tft.setFreeFont(nullptr);
    tft.setTextSize(3);
    int tw = strlen(timeStr) * 18;
    tft.setCursor(OX + (OW - tw) / 2, clockY);
    tft.print(timeStr);
  }

  tft.setTextFont(1);
  tft.setTextSize(1);
}

extern bool overTempWarningAcknowledged;

// ---------------------------------------------------------------------------
// Over-temperature WARNING overlay (>= 230 F): motor keeps running
// Matches the reference design: red banner top, white message bottom
// ---------------------------------------------------------------------------
void drawRuntimeOverTempOverlay(float tempC, bool forceRedraw) {
  static bool  lastShown      = false;
  static float lastTemp       = -999.0f;
  static bool  lastShutdown   = false;

  bool isShutdown = overTempShutdown;

  if (!forceRedraw && lastShown && fabsf(tempC - lastTemp) < 0.5f && lastShutdown == isShutdown) {
    return;
  }
  lastShown    = true;
  lastTemp     = tempC;
  lastShutdown = isShutdown;

  const int OX = 10;
  const int OY = 40;
  const int OW = SCREEN_WIDTH  - (OX * 2);   // 460
  const int OH = SCREEN_HEIGHT - (OY * 2);   // 240

  // ---- Outer border (double-line red) ----
  tft.fillRect(OX, OY, OW, OH, TFT_BLACK);
  for (int t = 0; t < 3; t++) {
    tft.drawRect(OX + t, OY + t, OW - t * 2, OH - t * 2, TFT_RED);
  }

  // ---- Top red banner ----
  const int bannerH = OH / 2;
  tft.fillRect(OX + 3, OY + 3, OW - 6, bannerH - 3, TFT_RED);

  // Warning triangles — left and right of the title text
  int triY  = OY + 3 + (bannerH - 3) / 2;
  int triSz = 22;
  drawWarningTriangle(OX + 28,      triY, triSz, TFT_YELLOW);
  drawWarningTriangle(OX + OW - 28, triY, triSz, TFT_YELLOW);

  // "WARNING" title
  tft.setTextColor(TFT_WHITE, TFT_RED);
  tft.setFreeFont(nullptr);
  tft.setTextSize(3);
  {
    const char* title = "WARNING";
    int tw = strlen(title) * 18;
    tft.setCursor(OX + (OW - tw) / 2, OY + 10);
    tft.print(title);
  }

  // "HIGH MOTOR TEMPERATURE"
  tft.setTextSize(2);
  {
    const char* line2 = "HIGH MOTOR TEMPERATURE";
    int tw = strlen(line2) * 12;
    tft.setCursor(OX + (OW - tw) / 2, OY + 46);
    tft.print(line2);
  }

  // Temperature value in large text
  if (tempC > -100.0f) {
    float tempF = (tempC * 9.0f / 5.0f) + 32.0f;
    char tempStr[12];
    snprintf(tempStr, sizeof(tempStr), "%.0f F", tempF);
    tft.setTextSize(3);
    int tw = strlen(tempStr) * 18;
    tft.setCursor(OX + (OW - tw) / 2, OY + 76);
    tft.print(tempStr);
  }

  // ---- Bottom white section ----
  int botY = OY + bannerH;
  int botH = OH - bannerH;
  tft.fillRect(OX + 3, botY, OW - 6, botH - 3, TFT_WHITE);

  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(2);

  if (isShutdown) {
    // Shutdown message — 3 lines, no interaction possible
    const char* l1 = "MOTOR SHUTDOWN";
    const char* l2 = "Restart unit to continue";
    const char* l3 = "Check Filters before restart";
    const int lineH = 26;
    const int textBlockH = lineH * 3;
    int startY = botY + (botH - textBlockH) / 2;
    tft.setTextSize(2);
    tft.setCursor(OX + (OW - (int)strlen(l1) * 12) / 2, startY);
    tft.print(l1);
    tft.setCursor(OX + (OW - (int)strlen(l2) * 12) / 2, startY + lineH);
    tft.print(l2);
    tft.setCursor(OX + (OW - (int)strlen(l3) * 12) / 2, startY + lineH * 2);
    tft.print(l3);
  } else {
    // Warning-only message — motor still running; show "Press to continue" prompt
    const char* l1 = "Check Filters for obstructions";
    const char* l2 = "Cleaning may be required";
    const char* l3 = "Press to continue";
    const int lineH = 24;
    const int textBlockH = lineH * 3 + 4;  // small extra gap before prompt
    int startY = botY + (botH - textBlockH) / 2;
    tft.setTextSize(2);
    tft.setCursor(OX + (OW - (int)strlen(l1) * 12) / 2, startY);
    tft.print(l1);
    tft.setCursor(OX + (OW - (int)strlen(l2) * 12) / 2, startY + lineH);
    tft.print(l2);
    // "Press to continue" drawn smaller and in green to distinguish it as an action
    tft.setTextSize(2);
    tft.setTextColor(TFT_DARKGREEN, TFT_WHITE);
    tft.setCursor(OX + (OW - (int)strlen(l3) * 12) / 2, startY + lineH * 2 + 4);
    tft.print(l3);
  }

  tft.setTextFont(1);
  tft.setTextSize(1);
}

void drawRuntimeFilterWarningOverlay() {
  // ---- Overlay geometry ----
  const int OX = 10;
  const int OY = 40;
  const int OW = SCREEN_WIDTH - (OX * 2);   // 460
  const int OH = SCREEN_HEIGHT - (OY * 2);  // 240

  // Outer border (double-line yellow/orange)
  tft.fillRect(OX, OY, OW, OH, TFT_BLACK);
  for (int t = 0; t < 3; t++) {
    tft.drawRect(OX + t, OY + t, OW - t * 2, OH - t * 2, TFT_RED);
  }

  // ---- Top red banner ----
  const int bannerH = OH / 2;
  tft.fillRect(OX + 3, OY + 3, OW - 6, bannerH - 3, TFT_RED);

  // Warning triangles
  int triY  = OY + 3 + (bannerH - 3) / 2;
  int triSz = 22;
  drawWarningTriangle(OX + 28,      triY, triSz, TFT_YELLOW);
  drawWarningTriangle(OX + OW - 28, triY, triSz, TFT_YELLOW);

  // "ATTENTION" title
  tft.setTextColor(TFT_WHITE, TFT_RED);
  tft.setFreeFont(nullptr);
  tft.setTextSize(3);
  {
    const char* title = "ATTENTION";
    int tw = strlen(title) * 18;
    tft.setCursor(OX + (OW - tw) / 2, OY + 10);
    tft.print(title);
  }

  // Sub-title line
  tft.setTextSize(2);
  {
    const char* sub = "Filter Maintenance Required";
    int tw = strlen(sub) * 12;
    tft.setCursor(OX + (OW - tw) / 2, OY + 48);
    tft.print(sub);
  }

  // ---- Bottom white section ----
  int botY = OY + bannerH;
  int botH = OH - bannerH;
  tft.fillRect(OX + 3, botY, OW - 6, botH - 3, TFT_WHITE);

  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setFreeFont(nullptr);
  tft.setTextSize(2);

  const char* l1 = "Clean Filters to protect Motor";
  const char* l2 = "life and restore performance.";
  const char* l3 = "Reset Filter Timer to clear warning";
  const int lineH = 26;
  const int textBlockH = lineH * 3;
  int startY = botY + (botH - textBlockH) / 2;

  tft.setCursor(OX + (OW - (int)strlen(l1) * 12) / 2, startY);
  tft.print(l1);
  tft.setCursor(OX + (OW - (int)strlen(l2) * 12) / 2, startY + lineH);
  tft.print(l2);
  tft.setCursor(OX + (OW - (int)strlen(l3) * 12) / 2, startY + lineH * 2);
  tft.print(l3);

  tft.setTextFont(1);
  tft.setTextSize(1);
}

// ---------------------------------------------------------------------------
// Helper: draw a QR-code placeholder frame with centred URL text inside
// Replace qrBitmap pointer + w/h args with a real pushImage call when ready.
// ---------------------------------------------------------------------------
static void drawQrPlaceholder(int cx, int cy, int size, const char* url) {
  int x = cx - size / 2;
  int y = cy - size / 2;
  // Outer border
  tft.drawRect(x,     y,     size,     size,     COLOR_TEXT_PRIMARY);
  tft.drawRect(x + 1, y + 1, size - 2, size - 2, COLOR_TEXT_PRIMARY);
  // Corner finder squares (top-left, top-right, bottom-left)
  const int fp = 7 * (size / 100 > 0 ? size / 100 : 1) + 14;  // finder size ~21px for size=140
  // top-left
  tft.drawRect(x + 4,          y + 4,          fp, fp, COLOR_TEXT_PRIMARY);
  tft.drawRect(x + 4 + 2,      y + 4 + 2,      fp - 4, fp - 4, COLOR_TEXT_PRIMARY);
  tft.fillRect(x + 4 + 5,      y + 4 + 5,      fp - 10, fp - 10, COLOR_TEXT_PRIMARY);
  // top-right
  tft.drawRect(x + size - 4 - fp, y + 4,          fp, fp, COLOR_TEXT_PRIMARY);
  tft.drawRect(x + size - 4 - fp + 2, y + 4 + 2,  fp - 4, fp - 4, COLOR_TEXT_PRIMARY);
  tft.fillRect(x + size - 4 - fp + 5, y + 4 + 5,  fp - 10, fp - 10, COLOR_TEXT_PRIMARY);
  // bottom-left
  tft.drawRect(x + 4,          y + size - 4 - fp, fp, fp, COLOR_TEXT_PRIMARY);
  tft.drawRect(x + 4 + 2,      y + size - 4 - fp + 2, fp - 4, fp - 4, COLOR_TEXT_PRIMARY);
  tft.fillRect(x + 4 + 5,      y + size - 4 - fp + 5, fp - 10, fp - 10, COLOR_TEXT_PRIMARY);
  // URL centred inside (small font)
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  tft.setTextSize(1);
  int textW = strlen(url) * 6;
  tft.setCursor(cx - textW / 2, cy - 4);
  tft.print(url);
}

void drawSupportMenuScreen(uint8_t menuIndex) {
  tft.fillScreen(COLOR_BG);

  // Title
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  int titleW = 7 * 18;  // "SUPPORT" 7 chars * ~18px
  tft.setCursor((SCREEN_WIDTH - titleW) / 2, 14);
  tft.print("SUPPORT");

  // Divider
  tft.drawFastHLine(0, 50, SCREEN_WIDTH, COLOR_TEXT_PRIMARY);

  const char* options[4] = {"Frequently Asked Questions", "Technical Information", "Contact Us", "Return To Main Menu"};
  // Spread 4 items evenly between y=70 and y=270
  const int startY  = 80;
  const int stepY   = 52;

  for (uint8_t i = 0; i < 4; i++) {
    int y = startY + i * stepY;
    if (i == menuIndex) {
      tft.fillRect(20, y - 8, SCREEN_WIDTH - 40, 32, COLOR_MENU_SELECT);
      tft.setTextColor(TFT_BLACK, COLOR_MENU_SELECT);
    } else {
      tft.fillRect(20, y - 8, SCREEN_WIDTH - 40, 32, COLOR_BG);
      tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
    }
    tft.setTextSize(2);
    int textW = strlen(options[i]) * 12;
    tft.setCursor((SCREEN_WIDTH - textW) / 2, y);
    tft.print(options[i]);
  }
}

void drawSupportFaqScreen(void) {
  tft.fillScreen(COLOR_BG);

  // Title
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(2);
  int titleW = 26 * 12;
  tft.setCursor((SCREEN_WIDTH - titleW) / 2, 12);
  tft.print("Frequently Asked Questions");

  // QR code centred on screen
  int qrX = (SCREEN_WIDTH - 200) / 2;
  int qrY = 50;
  tft.pushImage(qrX, qrY, 200, 200, qrFaq);

  // Footer
  tft.setTextColor(COLOR_SUCCESS, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(100, SCREEN_HEIGHT - 28);
  tft.print("Press to return to menu");
}

void drawSupportTechScreen(void) {
  tft.fillScreen(COLOR_BG);

  // Title lines
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor((SCREEN_WIDTH - 12 * 12) / 2, 8);
  tft.print("USERS MANUAL");
  tft.setTextSize(2);
  int t2W = 20 * 12;
  tft.setCursor((SCREEN_WIDTH - t2W) / 2, 30);
  tft.print("Technical information");

  // QR code centred
  int qrX = (SCREEN_WIDTH - 200) / 2;
  int qrY = 55;
  tft.pushImage(qrX, qrY, 200, 200, qrTech);

  // Footer
  tft.setTextColor(COLOR_SUCCESS, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(100, SCREEN_HEIGHT - 28);
  tft.print("Press to return to menu");
}

void drawSupportContactScreen(void) {
  tft.fillScreen(COLOR_BG);

  // Title
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  int titleW = 10 * 18;
  tft.setCursor((SCREEN_WIDTH - titleW) / 2, 10);
  tft.print("CONTACT US");
  tft.drawFastHLine(0, 48, SCREEN_WIDTH, COLOR_TEXT_PRIMARY);

  // Contact details
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(2);
  const int lx = SCREEN_WIDTH / 2;  // centre-aligned
  struct { const char* text; int y; } lines[] = {
    { "Apollo Sprayers International, Inc.", 62 },
    { "1030 Joshua Way",                    88 },
    { "Vista, CA. 92081",                   114 },
    { "Toll Free: (888) 900-HVLP (4857)",   140 },
    { "E-Mail:  info@hvlp.com",             166 },
    { "www.HVLP.com",                       192 },
  };
  for (auto& l : lines) {
    int w = strlen(l.text) * 12;
    tft.setCursor((SCREEN_WIDTH - w) / 2, l.y);
    tft.print(l.text);
  }

  // Footer
  tft.setTextColor(COLOR_SUCCESS, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(100, SCREEN_HEIGHT - 28);
  tft.print("Press to return to menu");
}

void drawTimersScreen(uint32_t totalRuntimeTenths, uint32_t totalJobTimeTenths, uint8_t selectedOption) {
  tft.fillScreen(COLOR_BG);

  // Title with clock icon
  drawIconClock(28, 26, 16, COLOR_TEXT_PRIMARY);
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(55, 14);
  tft.print("TIMERS");

  // Divider
  tft.drawFastHLine(0, 52, SCREEN_WIDTH, COLOR_TEXT_SECONDARY);

  // --- Job Timer block ---
  const int block1Y = 65;
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(20, block1Y);
  tft.print("JOB TIMER");

  {
    uint32_t jHours  = totalJobTimeTenths / 10;
    uint8_t  jTenths = totalJobTimeTenths % 10;
    char buf[12];
    snprintf(buf, sizeof(buf), "%02lu:%02u", (unsigned long)(jHours % 100), jTenths * 6);
    tft.setFreeFont(&FreeSansBold18pt7b);
    tft.setTextColor(COLOR_RUNTIME, COLOR_BG);  // always green
    tft.setCursor(100, block1Y + 70);
    tft.print(buf);
    tft.setFreeFont(nullptr);  // back to GLCD
  }

  // "PUSH TO RESET" label under job timer
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(100, block1Y + 57);
  //tft.print("PUSH TO RESET");

  // Divider between blocks
  tft.drawFastHLine(0, 160, SCREEN_WIDTH, COLOR_TEXT_SECONDARY);

  // --- Filter Maintenance Timer block ---
  const int block2Y = 173;
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(20, block2Y);
  tft.print("FILTER MAINTENANCE TIMER");

  {
    uint32_t fHours  = totalRuntimeTenths / 10;
    uint8_t  fTenths = totalRuntimeTenths % 10;
    // Cyan/blue while healthy, red when >= 10 hours (100 tenths)
    uint16_t fColor  = (totalRuntimeTenths < 100) ? COLOR_CURRENT : COLOR_ERROR;
    char buf[12];
    snprintf(buf, sizeof(buf), "%02lu:%02u", (unsigned long)(fHours % 100), fTenths * 6);
    tft.setFreeFont(&FreeSansBold18pt7b);
    tft.setTextColor(fColor, COLOR_BG);
    tft.setCursor(100, block2Y + 70);
    tft.print(buf);
    tft.setFreeFont(nullptr);
  }

  // "PUSH TO RESET" label under filter timer
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(20, block2Y + 57);
  //tft.print("PUSH TO RESET");

  // Divider above footer
  tft.drawFastHLine(0, 278, SCREEN_WIDTH, COLOR_TEXT_SECONDARY);

  // --- Footer: three options ---
  // Option 0: Reset Job Timer  |  Option 1: Reset Filter Timer  |  Option 2: Return
  const int optionY = SCREEN_HEIGHT - 36;
  const int opt0X   = 10;
  const int opt1X   = 175;
  const int opt2X   = 360;
  const int opt0W   = 158;
  const int opt1W   = 178;
  const int opt2W   = 110;
  const int optH    = 32;

  auto drawOpt = [&](int ox, int ow, uint8_t idx, const char* label) {
    if (selectedOption == idx) {
      tft.fillRect(ox, optionY, ow, optH, COLOR_MENU_SELECT);
      tft.setTextColor(TFT_BLACK, COLOR_MENU_SELECT);
    } else {
      tft.fillRect(ox, optionY, ow, optH, COLOR_BG);
      tft.drawRect(ox, optionY, ow, optH, COLOR_TEXT_SECONDARY);
      tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
    }
    tft.setTextSize(2);
    // Centre text horizontally in the box
    int textW = strlen(label) * 12;  // ~12px per char at size 2
    int textX = ox + (ow - textW) / 2;
    tft.setCursor(textX, optionY + 8);
    tft.print(label);
  };

  drawOpt(opt0X, opt0W, 0, "Reset Job");
  drawOpt(opt1X, opt1W, 1, "Reset Filter");
  drawOpt(opt2X, opt2W, 2, "Return");
}

void drawAboutScreen(uint32_t totalSystemTimeTenths, const char* firmwareVersion) {
  tft.fillScreen(COLOR_BG);

  // Title
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(185, 14);
  tft.print("ABOUT");

  // Divider
  tft.drawFastHLine(0, 50, SCREEN_WIDTH, COLOR_TEXT_SECONDARY);

  // Product name (centred)
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(95, 68);
  tft.print("PRECISION 6 PRO ELITE");

  // Software version
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(95, 100);
  tft.print("Software Version V");
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.print(firmwareVersion);

  // System hours
  {
    uint32_t sHours  = totalSystemTimeTenths / 10;
    uint8_t  sTenths = totalSystemTimeTenths % 10;
    char buf[24];
    snprintf(buf, sizeof(buf), "System Hours %02lu:%02u",
             (unsigned long)sHours, sTenths * 6);
    tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(95, 132);
    tft.print(buf);
  }

  // Assembled in USA
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(130, 164);
  tft.print("Assembled in USA");

  // Footer — plain green text, same style as Support screen
  tft.setTextColor(COLOR_SUCCESS, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(100, SCREEN_HEIGHT - 40);
  tft.print("Press to return to menu");
}
