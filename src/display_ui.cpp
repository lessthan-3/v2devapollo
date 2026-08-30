#include "display_ui.h"
#include "motor_control.h"
#include "splash_image.h"
#include "qr_images.h"
#include "config.h"
#include "ota_update.h"
#include <math.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// Run-time theme support
// Redefine the colour macros so that all drawing code automatically uses the
// correct colour for the active theme without touching every call-site.
// ---------------------------------------------------------------------------
extern bool lightThemeEnabled;
#undef  COLOR_BG
#define COLOR_BG           (lightThemeEnabled ? (uint16_t)TFT_WHITE  : (uint16_t)TFT_BLACK)
#undef  COLOR_TEXT_PRIMARY
#define COLOR_TEXT_PRIMARY (lightThemeEnabled ? (uint16_t)TFT_BLACK  : (uint16_t)TFT_WHITE)
#undef  COLOR_LABEL
#define COLOR_LABEL        (lightThemeEnabled ? (uint16_t)TFT_BLACK      : (uint16_t)TFT_WHITE)
#undef  COLOR_TEXT_SECONDARY
#define COLOR_TEXT_SECONDARY (lightThemeEnabled ? (uint16_t)TFT_DARKGREY : (uint16_t)TFT_LIGHTGREY)
#undef  COLOR_TEXT_SECRET
#define COLOR_TEXT_SECRET    (lightThemeEnabled ? (uint16_t)TFT_DARKGREEN : (uint16_t)TFT_YELLOW)
// Green reads poorly on a white background — use dark green in light mode
#undef  COLOR_SUCCESS
#define COLOR_SUCCESS        (lightThemeEnabled ? (uint16_t)TFT_DARKGREEN : (uint16_t)TFT_GREEN)
#undef  COLOR_RUNTIME
#define COLOR_RUNTIME        (lightThemeEnabled ? (uint16_t)TFT_DARKGREEN : (uint16_t)TFT_GREEN)

// ---------------------------------------------------------------------------
// Pressure-zone sprite — renders the large number off-screen then blits
// atomically, eliminating the visible erase-then-draw flicker.
// The sprite covers the top zone that drawRuntimeTarget clears each update.
// On ESP32-S3 N8R2 with PSRAM enabled, malloc routes large allocations to the
// 2 MB PSRAM so the ~128 KB buffer does not consume internal SRAM.
// ---------------------------------------------------------------------------
static TFT_eSprite pressSprite(&tft);
static bool        pressSpritReady = false;

static TFT_eSprite motorSprite(&tft);
static bool        motorSpriteReady = false;

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

  // Use white background to match the white backlight that is already on at power-up,
  // eliminating the visible contrast flash before the display initialises.
  const uint16_t SPLASH_BG    = TFT_WHITE;
  const uint16_t LOGO_COLOR   = TFT_RED;

  // Fill the entire screen first so the edges outside the centred image are white too
  tft.fillScreen(SPLASH_BG);

  tft.startWrite();
  tft.setWindow(destX, 0, destX + destW - 1, destH - 1);
  for (int dy = 0; dy < destH; dy++) {
    int sy = (dy * SPLASH_IMAGE_HEIGHT) / destH;
    for (int dx = 0; dx < destW; dx++) {
      int sx = (dx * SPLASH_IMAGE_WIDTH) / destW;
      int idx = sy * SPLASH_IMAGE_WIDTH + sx;
      uint8_t a = pgm_read_byte(&splashAlpha[idx]);
      tft.pushColor(a ? LOGO_COLOR : SPLASH_BG);
    }
  }
  tft.endWrite();
}

void drawMenuFooter(const char* message, uint16_t color) {
  tft.fillRect(0, SCREEN_HEIGHT - 40, SCREEN_WIDTH, 40, COLOR_BG);
  if (message != NULL && message[0] != '\0') {
    tft.setTextColor(color, COLOR_BG);
    int len = (int)strlen(message);
    if (len * 18 <= SCREEN_WIDTH - 20) {
      tft.setTextSize(3);
      tft.setCursor((SCREEN_WIDTH - len * 18) / 2, SCREEN_HEIGHT - 32);
    } else {
      tft.setTextSize(2);
      tft.setCursor((SCREEN_WIDTH - len * 12) / 2, SCREEN_HEIGHT - 30);
    }
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

// ⬇  Download arrow icon – downward arrow with a horizontal base line
static void drawIconDownload(int cx, int cy, int r, uint16_t color) {
  // Vertical shaft
  tft.fillRect(cx - 2, cy - r, 5, r, color);
  // Arrowhead (filled triangle pointing down)
  for (int row = 0; row <= r / 2; row++) {
    int hw = row;
    tft.drawFastHLine(cx - hw, cy + row - 2, hw * 2 + 1, color);
  }
  // Base line
  tft.fillRect(cx - r + 2, cy + r / 2 + 2, (r - 2) * 2, 3, color);
}

// Draw the appropriate icon for a given menu index
static void drawMenuIcon(uint8_t menuIdx, int cx, int cy, uint16_t color) {
  switch (menuIdx) {
    case 0: drawIconPlay(cx, cy, 11, color);     break;  // Start Motor
    case 1: drawIconCog(cx, cy, 13, color);      break;  // Settings
    case 2: drawIconClock(cx, cy, 13, color);    break;  // Timers
    case 3: drawIconQuestion(cx, cy, 13, color); break;  // Support
    case 4: drawIconInfo(cx, cy, 13, color);     break;  // About
    case 5: drawIconDownload(cx, cy, 13, color); break;  // FW Update
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

  const char* options[MENU_OPTION_COUNT] = {
    "START MOTOR", "SETTINGS", "TIMERS", "SUPPORT", "ABOUT", "FW UPDATE"
  };

  for (uint8_t i = 0; i < MENU_OPTION_COUNT; i++) {
    int y     = MENU_TOP_Y + (i * MENU_OPTION_HEIGHT);
    int rowCy = y + (MENU_OPTION_HEIGHT / 2) - 4;  // icon vertical centre
    bool selected = (i == menuIndex);

    // Row background
    if (selected) {
      tft.fillRect(textAreaX, y - 4, SCREEN_WIDTH - textAreaX, MENU_OPTION_HEIGHT, COLOR_MENU_SELECT);
      tft.fillRect(0, y - 4, textAreaX, MENU_OPTION_HEIGHT, COLOR_BG);
    } else {
      tft.fillRect(0, y - 4, SCREEN_WIDTH, MENU_OPTION_HEIGHT, COLOR_BG);
    }

    // Icon — always on background colour
    uint16_t iconColor = selected ? COLOR_TEXT_PRIMARY : COLOR_TEXT_SECONDARY;
    drawMenuIcon(i, 20, rowCy, iconColor);

    // Text — built-in font size 3, colour changes for selected row
    uint16_t fg = selected ? TFT_BLACK : COLOR_TEXT_PRIMARY;
    uint16_t bg = selected ? COLOR_MENU_SELECT : COLOR_BG;

    tft.setFreeFont(nullptr);
    tft.setTextSize(3);
    tft.setTextColor(fg, bg);

    int16_t tw = strlen(options[i]) * 18;  // 18px per char at size 3
    int tx = textAreaX + (textAreaW - tw) / 2;
    int ty = y - 4 + (MENU_OPTION_HEIGHT - 24) / 2;  // 24px tall at size 3
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
    int len = (int)strlen(message);
    if (len * 18 <= SCREEN_WIDTH - 20) {
      tft.setTextSize(3);
      tft.setCursor((SCREEN_WIDTH - len * 18) / 2, SCREEN_HEIGHT - 32);
    } else {
      tft.setTextSize(2);
      tft.setCursor((SCREEN_WIDTH - len * 12) / 2, SCREEN_HEIGHT - 30);
    }
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
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  {
    const char* title = "SETTINGS";
    int tw = strlen(title) * 18;
    tft.setCursor((SCREEN_WIDTH - tw) / 2, 14);
    tft.print(title);
  }

  for (uint8_t i = 0; i < SETTINGS_OPTION_COUNT; i++) {
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
  const char* options[5] = {"PowerPause Timeout", "Audible Beeper", "Units", "Theme", "Exit"};
  int rowTop = SETTINGS_TOP_Y + (settingsIndex * SETTINGS_OPTION_HEIGHT);
  // Vertically centre size-3 text (24 px tall) within the row
  int textY  = rowTop + (SETTINGS_OPTION_HEIGHT - 24) / 2;
  uint16_t bg = COLOR_BG;
  uint16_t fg = COLOR_TEXT_PRIMARY;
  if (selected) {
    bg = editing ? COLOR_MENU_EDIT : COLOR_MENU_SELECT;
    fg = TFT_BLACK;
  }

  // Full-width fill gives a consistent rectangular highlight
  tft.fillRect(0, rowTop, SCREEN_WIDTH, SETTINGS_OPTION_HEIGHT, bg);
  tft.setTextColor(fg, bg);
  tft.setTextSize(3);
  tft.setCursor(16, textY);
  tft.print(options[settingsIndex]);

  if (settingsIndex == 0) {
    // "%3u%%" max 4 chars * 18px = 72px; right-align with 8px margin
    uint16_t pct = (uint16_t)((uint32_t)pauseSeconds * 100u / POWER_PAUSE_PCT_BASE);
    tft.setCursor(SCREEN_WIDTH - 72 - 8, textY);
    tft.printf("%3u%%", pct);
  } else if (settingsIndex == 1) {
    // "OFF" max 3 chars * 18px = 54px
    tft.setCursor(SCREEN_WIDTH - 54 - 8, textY);
    tft.print(beeperEnabled ? " ON" : "OFF");
  } else if (settingsIndex == 2) {
    // "IMPERIAL" 8 chars * 18px = 144px
    tft.setCursor(SCREEN_WIDTH - 144 - 8, textY);
    tft.print(units == UNITS_IMPERIAL ? "IMPERIAL" : "  METRIC");
  } else if (settingsIndex == 3) {
    // " LIGHT" 6 chars * 18px = 108px
    tft.setCursor(SCREEN_WIDTH - 108 - 8, textY);
    tft.print(lightThemeEnabled ? " LIGHT" : "  DARK ");
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

  // ---- Create pressure-zone sprite for flicker-free target updates ----
  // The sprite covers exactly the zone that drawRuntimeTarget clears each tick.
  pressSprite.deleteSprite();
  pressSprite.setColorDepth(16);
  pressSpritReady = (pressSprite.createSprite(SCREEN_WIDTH, THIRD_1_Y - 24) != nullptr);
  if (!pressSpritReady) {
    Serial.println("[WARN] pressSprite alloc failed – fallback to direct draw");
  }

  // ---- Create motor-power sprite for flicker-free turbine output updates ----
  const int motorZoneH = (THIRD_2_Y - 24) - (THIRD_1_Y + 1);
  motorSprite.deleteSprite();
  motorSprite.setColorDepth(16);
  motorSpriteReady = (motorSprite.createSprite(SCREEN_WIDTH, motorZoneH) != nullptr);
  if (!motorSpriteReady) {
    Serial.println("[WARN] motorSprite alloc failed – fallback to direct draw");
  }
}

void drawRuntimeTarget(float target, float current, DisplayUnits units, bool valid, bool forceRedraw, uint16_t motorSpeed) {
  (void)motorSpeed;  // reserved for future use
  static float lastTarget = -999.0f;
  static float lastCurrent = -999.0f;
  static bool lastValid = false;
  static uint16_t lastColor = 0;
  static DisplayUnits lastUnits = UNITS_IMPERIAL;

#ifdef SHOW_SET_PREVIEW
  // SHOW_SET_PREVIEW mode: when the user turns the encoder, briefly show the
  // set-pressure value in the display zone for 1 second, then revert to actual.
  static uint32_t targetChangedAt = 0;
  if (fabsf(target - lastTarget) >= 0.05f && lastTarget != -999.0f) {
    targetChangedAt = millis();
  }
  bool showingSetPreview = (targetChangedAt != 0) && (millis() - targetChangedAt < 1000UL);
  static bool lastShowingSetPreview = false;
#else
  // Default mode: always display the actual measured pressure.
  constexpr bool showingSetPreview = false;
#endif

  // Convert to display units
  float displayTarget = target;
  float displayCurrent = current;
  if (units == UNITS_METRIC) {
    displayTarget  = 200.0f + (target  - 3.0f) * 70.833f;
    displayCurrent = 200.0f + (current - 3.0f) * 70.833f;
    if (displayTarget  < 0.0f) displayTarget  = 0.0f;
    if (displayCurrent < 0.0f) displayCurrent = 0.0f;
  }

  // Color: always green (theme-aware) while running, grey when motor off or reading invalid
  uint16_t color = COLOR_SUCCESS;
  if (target <= 0.05f) color = TFT_DARKGREY;  // motor off / dialled to zero
  else if (!valid)     color = TFT_DARKGREY;

  if (!forceRedraw && fabsf(target - lastTarget) < 0.05f &&
      fabsf(current - lastCurrent) < 0.05f &&
      valid == lastValid && color == lastColor && units == lastUnits
#ifdef SHOW_SET_PREVIEW
      && showingSetPreview == lastShowingSetPreview
#endif
     ) {
    return;
  }

  lastTarget  = target;
  lastCurrent = current;
  lastValid   = valid;
  lastColor   = color;
  lastUnits   = units;
#ifdef SHOW_SET_PREVIEW
  lastShowingSetPreview = showingSetPreview;
#endif

  // Usable drawing height (above the label strip)
  const int zoneH = THIRD_1_Y - 24;

  // Helper lambda: build the numeric and unit strings from displayValue.
  // Called inside both the sprite and fallback branches.
  // With SHOW_SET_PREVIEW: show set pressure briefly when knob is turned.
  // Without SHOW_SET_PREVIEW (default): always show actual measured pressure.
  auto buildStrings = [&](char* numStr, size_t numSz, char* unitStr, size_t unitSz) {
#ifdef SHOW_SET_PREVIEW
    float displayValue = showingSetPreview ? displayTarget : displayCurrent;
#else
    float displayValue = displayCurrent;
#endif
    if (units == UNITS_IMPERIAL) {
      float v = displayValue;
      if (v < 0.0f)  v = 0.0f;
      if (v > 99.9f) v = 99.9f;
      snprintf(numStr,  numSz,  "%04.1f", v);
      snprintf(unitStr, unitSz, "PSI");
    } else {
      float v = displayValue;
      if (v < 0.0f)   v = 0.0f;
      if (v > 9999.0f) v = 9999.0f;
      snprintf(numStr,  numSz,  "%.0f", v);
      snprintf(unitStr, unitSz, "mbar");
    }
  };

  if (pressSpritReady) {
    // ---- Sprite path: render off-screen then blit atomically (no flicker) ----
    pressSprite.fillSprite(COLOR_BG);

    {
      char numStr[8], unitStr[8];
      buildStrings(numStr, sizeof(numStr), unitStr, sizeof(unitStr));

      pressSprite.setFreeFont(&FreeSansBold24pt7b);
      pressSprite.setTextSize(2);
      int16_t numW = pressSprite.textWidth(numStr);
      int16_t numH = pressSprite.fontHeight();

      pressSprite.setTextSize(1);
      int16_t unitW = pressSprite.textWidth(unitStr);

      const int gap    = 110;
      int startX       = (SCREEN_WIDTH - (numW + gap + unitW)) / 2;
      int baselineY    = (zoneH + numH) / 2 - 10;

      pressSprite.setFreeFont(&FreeSansBold24pt7b);
      pressSprite.setTextSize(3);
      pressSprite.setTextColor(color, COLOR_BG);
      pressSprite.setCursor(startX + 40, baselineY);
      pressSprite.print(numStr);

      pressSprite.setTextSize(1);
      pressSprite.setTextColor(color, COLOR_BG);
      pressSprite.setCursor(startX + numW + gap + 20, baselineY);
      pressSprite.print(unitStr);
    }

    pressSprite.pushSprite(0, 0);

  } else {
    // ---- Fallback: direct draw (visible flicker, used only if sprite alloc failed) ----
    tft.fillRect(0, 0, SCREEN_WIDTH, zoneH, COLOR_BG);

    {
      char numStr[8], unitStr[8];
      buildStrings(numStr, sizeof(numStr), unitStr, sizeof(unitStr));

      tft.setFreeFont(&FreeSansBold24pt7b);
      tft.setTextSize(2);
      int16_t numW = tft.textWidth(numStr);
      int16_t numH = tft.fontHeight();

      tft.setTextSize(1);
      int16_t unitW = tft.textWidth(unitStr);

      const int gap  = 110;
      int startX     = (SCREEN_WIDTH - (numW + gap + unitW)) / 2;
      int baselineY  = (zoneH + numH) / 2 - 10;

      tft.setFreeFont(&FreeSansBold24pt7b);
      tft.setTextSize(3);
      tft.setTextColor(color, COLOR_BG);
      tft.setCursor(startX + 40, baselineY);
      tft.print(numStr);

      tft.setTextSize(1);
      tft.setTextColor(color, COLOR_BG);
      tft.setCursor(startX + numW + gap + 20, baselineY);
      tft.print(unitStr);
    }
  }

  // Reset font
  tft.setTextFont(1);
  tft.setTextSize(1);
}

void drawRuntimeJobTime(uint32_t jobTimeSeconds, bool forceRedraw) {
  uint32_t hours   = jobTimeSeconds / 3600;
  uint32_t minutes = (jobTimeSeconds % 3600) / 60;

  char buf[16];
  snprintf(buf, sizeof(buf), "%03lu:%02lu",
           (unsigned long)hours, (unsigned long)minutes);

  static char lastBuf[16] = "";
  if (!forceRedraw && strcmp(buf, lastBuf) == 0) {
    return;
  }
  strncpy(lastBuf, buf, sizeof(lastBuf));

  // Bottom zone — middle third (160-319): job time, centred at x=240
  // 6 chars × 12 px/char = 72 px → start at 240 - 36 = 204
  const int infoLineY = THIRD_2_Y + 22;
  tft.setTextFont(1);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  tft.setCursor(204, infoLineY);
  tft.print(buf);
  tft.setTextSize(1);
}

void drawRuntimeMainsVoltage(float volts, bool forceRedraw) {
  if (volts < 0.0f) volts = 0.0f;

  char buf[16];
  snprintf(buf, sizeof(buf), "%3.0f VAC ", volts);

  static char lastBuf[16] = "";
  if (!forceRedraw && strcmp(buf, lastBuf) == 0) {
    return;
  }
  strncpy(lastBuf, buf, sizeof(lastBuf));

  // Bottom zone — right third (320-479): mains voltage
  const int infoLineY = THIRD_2_Y + 22;
  tft.setTextFont(1);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_DEBUG, COLOR_BG);
  tft.setCursor(326, infoLineY);
  tft.print(buf);
  tft.setTextSize(1);
}

void drawRuntimeTemperature(float tempC, DisplayUnits units, bool forceRedraw) {
  // Build the display string first — compare before touching the display
  // All branches are padded to the same 14-char width to avoid leftover pixels
  char buf[32];
  if (overTempShutdown) {
    snprintf(buf, sizeof(buf), "TEMP  SHUTDOWN");
  } else if (overTempWarning) {
    if (tempC < -100.0f) {
      snprintf(buf, sizeof(buf), "TEMP  WARN    ");
    } else {
      if (units == UNITS_IMPERIAL) {
        float tempF = (tempC * 9.0f / 5.0f) + 32.0f;
        snprintf(buf, sizeof(buf), "TEMP  %3.0f F! ", tempF);
      } else {
        snprintf(buf, sizeof(buf), "TEMP  %3.0f C! ", tempC);
      }
    }
  } else if (tempC < -100.0f) {
    snprintf(buf, sizeof(buf), "TEMP  ---     ");
  } else {
    if (units == UNITS_IMPERIAL) {
      float tempF = (tempC * 9.0f / 5.0f) + 32.0f;
      snprintf(buf, sizeof(buf), "TEMP  %3.0f F  ", tempF);
    } else {
      snprintf(buf, sizeof(buf), "TEMP  %3.0f C  ", tempC);
    }
  }

  static char lastBuf[32] = "";
  if (!forceRedraw && strcmp(buf, lastBuf) == 0) {
    return;
  }
  strncpy(lastBuf, buf, sizeof(lastBuf));

  // Bottom zone — left half: temperature
  // Overdraw with background colour — no fillRect flash
  const int infoLineY = THIRD_2_Y + 22;
  tft.setTextFont(1);
  tft.setTextSize(2);
  uint16_t tempTextColor = COLOR_TEXT_SECONDARY;
  if (overTempShutdown)     tempTextColor = COLOR_TEMP_WARNING;
  else if (overTempWarning) tempTextColor = COLOR_WARNING;
  tft.setTextColor(tempTextColor, COLOR_BG);
  // Left-aligned in left third (0-159)
  tft.setCursor(4, infoLineY);
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

  // Clamp to 0-100 for display
  uint16_t pct = (motorSpeed > 1000) ? 100 : (motorSpeed / 10);

  char buf[8];
  snprintf(buf, sizeof(buf), "%3u%%", pct);

  const int zoneTop = THIRD_1_Y + 1;
  const int zoneBot = THIRD_2_Y - 24;  // leave room for static label
  const int zoneH   = zoneBot - zoneTop;

  // Light mode → dark blue (navy); dark mode → light blue (cyan)
  uint16_t motorPctColor = lightThemeEnabled ? (uint16_t)TFT_NAVY : (uint16_t)TFT_CYAN;

  if (motorSpriteReady) {
    // ---- Sprite path: render off-screen then blit atomically (no flicker) ----
    motorSprite.fillSprite(COLOR_BG);
    motorSprite.setFreeFont(&FreeSansBold18pt7b);
    motorSprite.setTextSize(2);
    motorSprite.setTextColor(motorPctColor, COLOR_BG);

    int16_t w = motorSprite.textWidth(buf);
    int16_t h = motorSprite.fontHeight();
    int x = (SCREEN_WIDTH - w) / 2;
    int y = (zoneH + h) / 2 - 15;  // vertically centre baseline within sprite
    motorSprite.setCursor(x, y);
    motorSprite.print(buf);
    motorSprite.pushSprite(0, zoneTop);
  } else {
    // ---- Fallback: direct draw ----
    tft.fillRect(0, zoneTop, SCREEN_WIDTH, zoneH, COLOR_BG);
    tft.setFreeFont(&FreeSansBold18pt7b);
    tft.setTextSize(2);
    tft.setTextColor(motorPctColor, COLOR_BG);

    int16_t w = tft.textWidth(buf);
    int16_t h = tft.fontHeight();
    int x = (SCREEN_WIDTH - w) / 2;
    int y = zoneTop + (zoneH + h) / 2 - 15;
    tft.setCursor(x, y);
    tft.print(buf);
  }

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
    tft.setTextSize(3);
    {
      const char* sub = "PowerPause Active";
      int tw = strlen(sub) * 18;
      tft.setCursor(OX + (OW - tw) / 2, OY + 38);
      tft.print(sub);
    }

    // Bottom white section
    tft.fillRect(OX + 3, botY, OW - 6, botH - 3, TFT_WHITE);

    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.setFreeFont(nullptr);

    if (idleState == IDLE_STATE_PID_RAMP) {
      tft.setTextSize(3);
      const char* l1 = "RAMPING TO IDLE SPEED";
      tft.setCursor(OX + (OW - (int)strlen(l1) * 18) / 2, botY + (botH / 2) - 12);
      tft.print(l1);
    } else if (idleState == IDLE_STATE_HOLD) {
      const char* l1 = "TURN DIAL OR PULL TRIGGER";
      const char* l2 = "TO RESUME MOTOR";
      tft.setTextSize(3);
      int lineH = 28;
      if (secondsRemaining != UINT32_MAX) {
        int totalH = lineH + lineH + 38;
        int startY = botY + (botH - totalH) / 2;
        tft.setCursor(OX + (OW - (int)strlen(l1) * 18) / 2, startY);
        tft.print(l1);
        tft.setCursor(OX + (OW - (int)strlen(l2) * 18) / 2, startY + lineH);
        tft.print(l2);
      } else {
        int startY = botY + (botH / 2) - lineH;
        tft.setCursor(OX + (OW - (int)strlen(l1) * 18) / 2, startY);
        tft.print(l1);
        tft.setCursor(OX + (OW - (int)strlen(l2) * 18) / 2, startY + lineH);
        tft.print(l2);
      }
    }
  }

  // ---- Countdown clock: redrawn every tick, only when a timer is active ----
  if (idleState == IDLE_STATE_HOLD && secondsRemaining != UINT32_MAX) {
    int lineH  = 28;
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

// ---------------------------------------------------------------------------
// Over-temperature WARNING overlay (>= 230 F): motor keeps running
// Matches the reference design: red banner top, white message bottom
// ---------------------------------------------------------------------------
void drawRuntimeOverTempOverlay(float tempC, bool forceRedraw) {
  static bool  lastShown      = false;
  static float lastTemp       = -999.0f;
  static bool  lastShutdown   = false;

  bool isShutdown = overTempShutdown;

  // If only the temperature number changed (and chrome is already drawn) just repaint the value cell
  bool chromeOnly = false;
  if (!forceRedraw && lastShown && lastShutdown == isShutdown &&
      fabsf(tempC - lastTemp) >= 0.5f) {
    chromeOnly = true;  // skip full redraw, update number in-place
  } else if (!forceRedraw && lastShown && lastShutdown == isShutdown &&
             fabsf(tempC - lastTemp) < 0.5f) {
    return;  // nothing changed at all
  }

  lastShown    = true;
  lastTemp     = tempC;
  lastShutdown = isShutdown;

  const int OX = 10;
  const int OY = 40;
  const int OW = SCREEN_WIDTH  - (OX * 2);   // 460
  const int OH = SCREEN_HEIGHT - (OY * 2);   // 240
  const int bannerH = OH / 2;

  if (!chromeOnly) {
    // ---- Outer border (double-line red) ----
    tft.fillRect(OX, OY, OW, OH, TFT_BLACK);
    for (int t = 0; t < 3; t++) {
      tft.drawRect(OX + t, OY + t, OW - t * 2, OH - t * 2, TFT_RED);
    }

    // ---- Top red banner ----
    tft.fillRect(OX + 3, OY + 3, OW - 6, bannerH - 3, TFT_RED);

    // Warning triangles
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

    // "HIGH MOTOR TEMP"
    tft.setTextSize(3);
    {
      const char* line2 = "HIGH MOTOR TEMP";
      int tw = strlen(line2) * 18;
      tft.setCursor(OX + (OW - tw) / 2, OY + 38);
      tft.print(line2);
    }
  }

  // Temperature value — repaint over a solid red background to erase previous number
  if (tempC > -100.0f) {
    float tempF = (tempC * 9.0f / 5.0f) + 32.0f;
    char tempStr[12];
    snprintf(tempStr, sizeof(tempStr), "%.0f F", tempF);
    // Clear the value cell first to avoid ghost digits
    tft.fillRect(OX + 3, OY + 62, OW - 6, 28, TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setFreeFont(nullptr);
    tft.setTextSize(3);
    int tw = strlen(tempStr) * 18;
    tft.setCursor(OX + (OW - tw) / 2, OY + 66);
    tft.print(tempStr);
  }

  if (chromeOnly) {
    // Only needed to update the number — done
    tft.setTextFont(1);
    tft.setTextSize(1);
    return;
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
    const char* l2 = "RESTART UNIT TO CONTINUE";
    const char* l3 = "CHECK FILTERS FIRST";
    const int lineH = 28;
    const int textBlockH = lineH * 3;
    int startY = botY + (botH - textBlockH) / 2;
    tft.setTextSize(3);
    tft.setCursor(OX + (OW - (int)strlen(l1) * 18) / 2, startY);
    tft.print(l1);
    tft.setCursor(OX + (OW - (int)strlen(l2) * 18) / 2, startY + lineH);
    tft.print(l2);
    tft.setCursor(OX + (OW - (int)strlen(l3) * 18) / 2, startY + lineH * 2);
    tft.print(l3);
  } else {
    // Warning-only message — motor still running; show "Press to continue" prompt
    const char* l1 = "CHECK FILTER CONDITION";
    const char* l2 = "CLEANING MAY BE REQUIRED";
    const int lineH = 28;
    const int textBlockH = lineH * 2;
    int startY = botY + (botH - textBlockH) / 2;
    tft.setTextSize(3);
    tft.setCursor(OX + (OW - (int)strlen(l1) * 18) / 2, startY);
    tft.print(l1);
    tft.setCursor(OX + (OW - (int)strlen(l2) * 18) / 2, startY + lineH);
    tft.print(l2);
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

  // Sub-title line (split to two lines at size 3 to fit 460px overlay width)
  tft.setTextSize(3);
  {
    const char* sub1 = "FILTER MAINTENANCE";
    const char* sub2 = "REQUIRED";
    int tw1 = strlen(sub1) * 18;
    int tw2 = strlen(sub2) * 18;
    tft.setCursor(OX + (OW - tw1) / 2, OY + 38);
    tft.print(sub1);
    tft.setCursor(OX + (OW - tw2) / 2, OY + 66);
    tft.print(sub2);
  }

  // ---- Bottom white section ----
  int botY = OY + bannerH;
  int botH = OH - bannerH;
  tft.fillRect(OX + 3, botY, OW - 6, botH - 3, TFT_WHITE);

  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setFreeFont(nullptr);
  tft.setTextSize(3);

  const char* l1 = "CLEAN YOUR FILTERS";
  const char* l2 = "RESET FILTER TIMER";
  const int lineH = 28;
  const int textBlockH = lineH * 2;
  int startY = botY + (botH - textBlockH) / 2;

  tft.setCursor(OX + (OW - (int)strlen(l1) * 18) / 2, startY);
  tft.print(l1);
  tft.setCursor(OX + (OW - (int)strlen(l2) * 18) / 2, startY + lineH);
  tft.print(l2);

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
  // 4 items from y=68, step=56: last item ends ~296, footer at 280 (items are clipped above footer)
  const int startY  = 68;
  const int stepY   = 56;
  const int boxH    = 44;  // tall enough for size-3 text (24px) with 10px padding each side

  for (uint8_t i = 0; i < 4; i++) {
    int y = startY + i * stepY;
    if (i == menuIndex) {
      tft.fillRect(0, y - 8, SCREEN_WIDTH, boxH, COLOR_MENU_SELECT);
      tft.setTextColor(TFT_BLACK, COLOR_MENU_SELECT);
    } else {
      tft.fillRect(0, y - 8, SCREEN_WIDTH, boxH, COLOR_BG);
      tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
    }
    // Auto-size: size 3 if the string fits within the screen width (minus 16px margin), else size 2
    int len = (int)strlen(options[i]);
    if (len * 18 <= SCREEN_WIDTH - 8) {
      tft.setTextSize(3);
      tft.setCursor((SCREEN_WIDTH - len * 18) / 2, y);
    } else {
      tft.setTextSize(2);
      tft.setCursor((SCREEN_WIDTH - len * 12) / 2, y + 4);  // +4 to vertically centre smaller text
    }
    tft.print(options[i]);
  }
}

void drawSupportFaqScreen(void) {
  tft.fillScreen(COLOR_BG);

  // Title — two size-3 lines (single line at size 3 is 468px, too tight)
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  const char* t1 = "Frequently Asked";
  const char* t2 = "Questions";
  tft.setCursor((SCREEN_WIDTH - (int)strlen(t1) * 18) / 2, 6);
  tft.print(t1);
  tft.setCursor((SCREEN_WIDTH - (int)strlen(t2) * 18) / 2, 34);
  tft.print(t2);

  // QR code centred between title and footer
  int qrX = (SCREEN_WIDTH - 200) / 2;
  int qrY = 68;
  tft.pushImage(qrX, qrY, 200, 200, qrFaq);

  // Footer
  drawMenuFooter("Press to return to menu", COLOR_SUCCESS);
}

void drawSupportTechScreen(void) {
  tft.fillScreen(COLOR_BG);

  // Title lines — size 3
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  const char* t1 = "USERS MANUAL";
  const char* t2 = "Technical Information";
  tft.setCursor((SCREEN_WIDTH - (int)strlen(t1) * 18) / 2, 6);
  tft.print(t1);
  tft.setCursor((SCREEN_WIDTH - (int)strlen(t2) * 18) / 2, 34);
  tft.print(t2);

  // QR code centred between title and footer
  int qrX = (SCREEN_WIDTH - 200) / 2;
  int qrY = 68;
  tft.pushImage(qrX, qrY, 200, 200, qrTech);

  // Footer
  drawMenuFooter("Press to return to menu", COLOR_SUCCESS);
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

  // Contact details — size 3 (strings kept short enough to fit 480px)
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  struct { const char* text; int y; } lines[] = {
    { "Apollo Sprayers Inc.",   58  },
    { "1030 Joshua Way",        92  },
    { "Vista, CA. 92081",       126 },
    { "Phone: (888) 900-4857",    160 },
    { "Email: info@hvlp.com",          194 },
    { "Web: www.HVLP.com",           228 },
  };
  for (auto& l : lines) {
    int w = (int)strlen(l.text) * 18;
    tft.setCursor((SCREEN_WIDTH - w) / 2, l.y);
    tft.print(l.text);
  }

  // Footer
  drawMenuFooter("Press to return to menu", COLOR_SUCCESS);
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
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(20, block1Y);
  tft.print("JOB TIMER");

  {
    uint32_t jHours  = totalJobTimeTenths / 10;
    uint8_t  jTenths = totalJobTimeTenths % 10;
    char buf[12];
    snprintf(buf, sizeof(buf), "%02lu:%02u", (unsigned long)(jHours % 100), jTenths * 6);
    tft.setFreeFont(&FreeSansBold18pt7b);
    tft.setTextColor(COLOR_RUNTIME, COLOR_BG);  // always green
    tft.setCursor(100, 155);
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
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(20, block2Y);
  tft.print("FILTER MAINTENANCE TIMER");

  {
    uint32_t fHours  = totalRuntimeTenths / 10;
    uint8_t  fTenths = totalRuntimeTenths % 10;
    // Green while healthy (< 10 hours), red when >= 10 hours (100 tenths)
    uint16_t fColor  = (totalRuntimeTenths < 100) ? (uint16_t)COLOR_SUCCESS : (uint16_t)COLOR_ERROR;
    char buf[12];
    snprintf(buf, sizeof(buf), "%02lu:%02u", (unsigned long)(fHours % 100), fTenths * 6);
    tft.setFreeFont(&FreeSansBold18pt7b);
    tft.setTextColor(fColor, COLOR_BG);
    tft.setCursor(100, 273);
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

void drawAboutScreen(uint32_t totalSystemTimeTenths, const char* firmwareVersion, bool confirmVisible) {
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
  tft.setTextSize(3);
  const char* prodName = "PRECISION 6 PRO ELITE";
  tft.setCursor((SCREEN_WIDTH - (int)strlen(prodName) * 18) / 2, 66);
  tft.print(prodName);

  // Software version
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  {
    // Build full string to centre it
    char verBuf[32];
    snprintf(verBuf, sizeof(verBuf), "Version V%s", firmwareVersion);
    tft.setCursor((SCREEN_WIDTH - (int)strlen(verBuf) * 18) / 2, 110);
    tft.print(verBuf);
  }

  // System hours
  {
    uint32_t sHours  = totalSystemTimeTenths / 10;
    uint8_t  sTenths = totalSystemTimeTenths % 10;
    char buf[24];
    snprintf(buf, sizeof(buf), "System Hours %02lu:%02u",
             (unsigned long)sHours, sTenths * 6);
    tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
    tft.setTextSize(3);
    tft.setCursor((SCREEN_WIDTH - (int)strlen(buf) * 18) / 2, 154);
    tft.print(buf);
  }

  // Assembled in USA
  tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
  tft.setTextSize(3);
  const char* assembled = "Manufactured in USA";
  tft.setCursor((SCREEN_WIDTH - (int)strlen(assembled) * 18) / 2, 198);
  tft.print(assembled);

  // Footer
  if (!confirmVisible) {
    drawMenuFooter("Press to return to menu", COLOR_SUCCESS);
  }
  // When confirmVisible the secret menu is drawn separately via drawSecretMenu()
}

// ---------------------------------------------------------------------------
// Secret menu — reached by turning the encoder 50 times on the About screen.
// Options: 0=Set System Hours  1=PP Sensitivity  2=Motor Test  3=Return
// ---------------------------------------------------------------------------

static const char* kSecretMenuLabels[4] = {
    "Set System Hours",
    "PP Sensitivity",
    "Motor Test",
    "Return"
};

void drawSecretMenu(uint8_t selectedOption, bool forceRedraw) {
  (void)forceRedraw;
  tft.fillScreen(COLOR_BG);

  // Title bar
  tft.setTextColor((uint16_t)COLOR_ERROR, COLOR_BG);
  tft.setTextSize(3);
  const char* title = "SERVICE MENU";
  tft.setCursor((SCREEN_WIDTH - (int)strlen(title) * 18) / 2, 12);
  tft.print(title);

  tft.drawFastHLine(0, 46, SCREEN_WIDTH, (uint16_t)COLOR_ERROR);

  // Menu rows
  const int ROW_H  = 52;
  const int ROW_Y0 = 56;

  for (uint8_t i = 0; i < 4; i++) {
    int rowY = ROW_Y0 + i * ROW_H;
    bool sel = (i == selectedOption);

    if (sel) {
      tft.fillRect(0, rowY, SCREEN_WIDTH, ROW_H - 2, (uint16_t)COLOR_MENU_SELECT);
      tft.setTextColor(TFT_BLACK, (uint16_t)COLOR_MENU_SELECT);
    } else {
      tft.fillRect(0, rowY, SCREEN_WIDTH, ROW_H - 2, COLOR_BG);
      tft.drawRect(0, rowY, SCREEN_WIDTH, ROW_H - 2, (uint16_t)COLOR_TEXT_SECONDARY);
      tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
    }

    tft.setTextSize(3);
    const char* lbl = kSecretMenuLabels[i];
    int lw = (int)strlen(lbl) * 18;
    tft.setCursor((SCREEN_WIDTH - lw) / 2, rowY + 14);
    tft.print(lbl);
  }
}

void drawSecretSetHoursScreen(uint32_t hours, bool forceRedraw) {
  (void)forceRedraw;
  tft.fillScreen(COLOR_BG);

  // Title
  tft.setTextColor((uint16_t)COLOR_ERROR, COLOR_BG);
  tft.setTextSize(3);
  const char* title = "SET SYSTEM HOURS";
  tft.setCursor((SCREEN_WIDTH - (int)strlen(title) * 18) / 2, 12);
  tft.print(title);
  tft.drawFastHLine(0, 46, SCREEN_WIDTH, (uint16_t)COLOR_ERROR);

  // Large hours value centred
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu hrs", (unsigned long)hours);
  tft.setTextColor(COLOR_TEXT_SECRET, COLOR_BG);
  tft.setTextSize(5);
  int w = (int)strlen(buf) * 30;
  tft.setCursor((SCREEN_WIDTH - w) / 2, 110);
  tft.print(buf);

  // Instruction footer
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  tft.setTextSize(2);
  const char* hint = "Rotate to adjust  Press to save";
  tft.setCursor((SCREEN_WIDTH - (int)strlen(hint) * 12) / 2, SCREEN_HEIGHT - 28);
  tft.print(hint);
}

void drawSecretSensitivityScreen(uint16_t sensitivityPct, bool forceRedraw) {
  (void)forceRedraw;
  tft.fillScreen(COLOR_BG);

  // Title
  tft.setTextColor((uint16_t)COLOR_ERROR, COLOR_BG);
  tft.setTextSize(3);
  const char* title = "PP SENSITIVITY";
  tft.setCursor((SCREEN_WIDTH - (int)strlen(title) * 18) / 2, 12);
  tft.print(title);
  tft.drawFastHLine(0, 46, SCREEN_WIDTH, (uint16_t)COLOR_ERROR);

  // Description
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  tft.setTextSize(2);
  const char* desc = "Spike threshold multiplier";
  tft.setCursor((SCREEN_WIDTH - (int)strlen(desc) * 12) / 2, 58);
  tft.print(desc);
  const char* desc2 = "Lower = more sensitive";
  tft.setCursor((SCREEN_WIDTH - (int)strlen(desc2) * 12) / 2, 80);
  tft.print(desc2);

  // Large percentage value
  char buf[12];
  snprintf(buf, sizeof(buf), "%u%%", sensitivityPct);
  tft.setTextColor(COLOR_TEXT_SECRET, COLOR_BG);
  tft.setTextSize(6);
  int w = (int)strlen(buf) * 36;
  tft.setCursor((SCREEN_WIDTH - w) / 2, 110);
  tft.print(buf);

  // Range hint
  tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
  tft.setTextSize(2);
  char rangeHint[24];
  snprintf(rangeHint, sizeof(rangeHint), "Range: %u%% - %u%%", PP_SENSITIVITY_MIN, PP_SENSITIVITY_MAX);
  tft.setCursor((SCREEN_WIDTH - (int)strlen(rangeHint) * 12) / 2, SCREEN_HEIGHT - 52);
  tft.print(rangeHint);

  // Instruction footer
  const char* hint = "Rotate to adjust  Press to save";
  tft.setCursor((SCREEN_WIDTH - (int)strlen(hint) * 12) / 2, SCREEN_HEIGHT - 28);
  tft.print(hint);
}

// ---------------------------------------------------------------------------
// Debug overlay preview carousel
// stage 0 : Filter warning
// stage 1 : PowerPause — PID ramp to idle
// stage 2 : PowerPause — Idle hold (with 7:30 countdown)
// stage 3 : OverTemp warning (motor keeps running)
// stage 4 : OverTemp shutdown (motor stopped)
// ---------------------------------------------------------------------------
#if DEBUG_OVERLAY_PREVIEW
void drawDebugOverlayPreview(uint8_t stage) {
  // Draw a minimal runtime backdrop so overlays have something to sit on top of
  drawRuntimeStatic(UNITS_IMPERIAL);
  drawRuntimeTarget(5.0f, 4.8f, UNITS_IMPERIAL, true, true, 650);
  drawRuntimeMotorPower(650, true);
  drawRuntimeJobTime(3723, true);
  drawRuntimeTemperature(85.0f, UNITS_IMPERIAL, true);

  // Stage label printed to serial so the developer can follow along
  const char* stageNames[] = {
    "Filter Warning",
    "PowerPause: PID Ramp",
    "PowerPause: Idle Hold",
    "OverTemp Warning",
    "OverTemp Shutdown",
  };
  Serial.printf("[DBG OVERLAY] Stage %u: %s\n", stage, stageNames[stage]);

  switch (stage) {
    case 0:
      drawRuntimeFilterWarningOverlay();
      break;

    case 1:
      drawRuntimePowerPauseOverlay(IDLE_STATE_PID_RAMP, UINT32_MAX, true);
      break;

    case 2:
      drawRuntimePowerPauseOverlay(IDLE_STATE_HOLD, 450, true);
      break;

    case 3: {
      // Temporarily set warning flags so the overlay renders the warning variant
      extern bool overTempWarning;
      extern bool overTempShutdown;
      bool savedW = overTempWarning, savedS = overTempShutdown;
      overTempWarning  = true;
      overTempShutdown = false;
      drawRuntimeOverTempOverlay(115.0f, true);
      overTempWarning  = savedW;
      overTempShutdown = savedS;
      break;
    }

    case 4: {
      extern bool overTempWarning;
      extern bool overTempShutdown;
      bool savedW = overTempWarning, savedS = overTempShutdown;
      overTempWarning  = true;
      overTempShutdown = true;
      drawRuntimeOverTempOverlay(135.0f, true);
      overTempWarning  = savedW;
      overTempShutdown = savedS;
      break;
    }

    default:
      break;
  }
}
#endif

// ---------------------------------------------------------------------------
// OTA Update screen
//
// Layout (480x320 landscape):
//   Title bar  : y 0-48    "FIRMWARE UPDATE"
//   Divider    : y 48
//   Content    : y 48-278  (varies by state)
//   Footer     : y 278-320 (instructions / progress bar label)
//
// WAITING_CREDS: QR code (left half) + text instructions (right half)
// All other states: centred status text
// ---------------------------------------------------------------------------

void drawOtaScreen(OtaState state, const char* detail, int progress,
                   uint8_t selectedOption, bool forceRedraw)
{
  static OtaState  lastState          = (OtaState)255;
  static int       lastProgress       = -1;
  static uint8_t   lastSelectedOption = 255;

  bool stateChanged   = forceRedraw || (state != lastState);
  bool progressChanged = (progress != lastProgress);
  bool selChanged     = (selectedOption != lastSelectedOption);

  if (!stateChanged && !progressChanged && !selChanged) return;

  lastState          = state;
  lastProgress       = progress;
  lastSelectedOption = selectedOption;

  if (stateChanged) {
    tft.fillScreen(COLOR_BG);

    // ---- Title bar ----
    tft.setFreeFont(nullptr);
    tft.setTextSize(3);
    tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
    {
      const char* title = "FIRMWARE UPDATE";
      int tw = strlen(title) * 18;
      tft.setCursor((SCREEN_WIDTH - tw) / 2, 10);
      tft.print(title);
    }
    tft.drawFastHLine(0, 48, SCREEN_WIDTH, COLOR_TEXT_SECONDARY);
  }

  // ---- Content area (y 55 to 270) ----
  const int CY  = 55;    // content top
  const int CH  = 215;   // content height (excludes footer strip)
  const int MCY = CY + CH / 2;  // vertical centre of content area

  switch (state) {

    // ------------------------------------------------------------------
    case OTA_STATE_STARTING_AP:
    case OTA_STATE_WAITING_CREDS:
      if (stateChanged) {
        tft.fillRect(0, CY, SCREEN_WIDTH, CH, COLOR_BG);

        // Left panel: QR code (150x150) centred vertically
        const int qrSize = 150;
        const int qrX    = 10;
        const int qrY    = CY + (CH - qrSize) / 2;
        tft.pushImage(qrX, qrY, qrSize, qrSize, qrOta);

        // Right panel: instructions
        const int TX = 175;
        tft.setFreeFont(nullptr);
        tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);

        tft.setTextSize(2);
        tft.setCursor(TX, CY + 4);
        tft.print("1. Scan QR code");
        tft.setCursor(TX, CY + 26);
        tft.print("   or connect to:");

        tft.setTextSize(3);
        tft.setTextColor(COLOR_SUCCESS, COLOR_BG);
        tft.setCursor(TX, CY + 54);
        tft.print(OTA_AP_SSID);

        tft.setTextSize(2);
        tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
        tft.setCursor(TX, CY + 90);
        tft.print("2. Browser opens");
        tft.setCursor(TX, CY + 110);
        tft.print("   auto. If not:");

        tft.setTextSize(2);
        tft.setTextColor(COLOR_SUCCESS, COLOR_BG);
        tft.setCursor(TX, CY + 132);
        tft.print(OTA_AP_IP);

        tft.setTextSize(2);
        tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
        tft.setCursor(TX, CY + 156);
        tft.print("3. Enter your home");
        tft.setCursor(TX, CY + 176);
        tft.print("   WiFi password");
      }
      break;

    // ------------------------------------------------------------------
    case OTA_STATE_CONNECTING_STA:
      if (stateChanged) {
        tft.fillRect(0, CY, SCREEN_WIDTH, CH, COLOR_BG);
        tft.setFreeFont(nullptr);
        tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
        tft.setTextSize(2);
        {
          const char* ln = "Connecting to WiFi...";
          tft.setCursor((SCREEN_WIDTH - (int)strlen(ln) * 12) / 2, MCY - 10);
          tft.print(ln);
        }
      }
      break;

    // ------------------------------------------------------------------
    case OTA_STATE_CHECKING_VERSION:
      if (stateChanged) {
        tft.fillRect(0, CY, SCREEN_WIDTH, CH, COLOR_BG);
        tft.setFreeFont(nullptr);
        tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
        tft.setTextSize(2);
        {
          const char* ln = "Checking for updates...";
          tft.setCursor((SCREEN_WIDTH - (int)strlen(ln) * 12) / 2, MCY - 10);
          tft.print(ln);
        }
      }
      break;

    // ------------------------------------------------------------------
    case OTA_STATE_VERSION_CURRENT:
      if (stateChanged || selChanged) {
        if (stateChanged) {
          tft.fillRect(0, CY, SCREEN_WIDTH, CH, COLOR_BG);
          tft.setFreeFont(nullptr);

          tft.setTextSize(3);
          tft.setTextColor(COLOR_SUCCESS, COLOR_BG);
          {
            const char* ln = "Firmware up to date!";
            tft.setCursor((SCREEN_WIDTH - (int)strlen(ln) * 18) / 2, MCY - 60);
            tft.print(ln);
          }

          tft.setTextSize(2);
          tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
          {
            char ln[40];
            snprintf(ln, sizeof(ln), "Current version: %s", FIRMWARE_VERSION);
            tft.setCursor((SCREEN_WIDTH - (int)strlen(ln) * 12) / 2, MCY - 20);
            tft.print(ln);
          }
        }

        // Two button options: RETURN (safe default) and REINSTALL
        const int BTN_Y   = MCY + 10;
        const int BTN_H   = 40;
        const int BTN_W   = 160;
        const int BTN0_X  = (SCREEN_WIDTH / 2) - BTN_W - 10;
        const int BTN1_X  = (SCREEN_WIDTH / 2) + 10;

        // RETURN button (option 0 — safe default)
        {
          bool sel = (selectedOption == 0);
          tft.fillRect(BTN0_X, BTN_Y, BTN_W, BTN_H,
                       sel ? COLOR_SUCCESS : COLOR_BG);
          tft.drawRect(BTN0_X, BTN_Y, BTN_W, BTN_H, COLOR_SUCCESS);
          tft.setTextColor(sel ? TFT_BLACK : COLOR_SUCCESS,
                           sel ? COLOR_SUCCESS : COLOR_BG);
          tft.setTextSize(2);
          const char* lbl = " RETURN";
          tft.setCursor(BTN0_X + (BTN_W - (int)strlen(lbl) * 12) / 2,
                        BTN_Y + (BTN_H - 16) / 2);
          tft.print(lbl);
        }
        // REINSTALL button (option 1)
        {
          bool sel = (selectedOption == 1);
          tft.fillRect(BTN1_X, BTN_Y, BTN_W, BTN_H,
                       sel ? COLOR_WARNING : COLOR_BG);
          tft.drawRect(BTN1_X, BTN_Y, BTN_W, BTN_H, COLOR_WARNING);
          tft.setTextColor(sel ? TFT_BLACK : COLOR_WARNING,
                           sel ? COLOR_WARNING : COLOR_BG);
          tft.setTextSize(2);
          const char* lbl = "REINSTALL";
          tft.setCursor(BTN1_X + (BTN_W - (int)strlen(lbl) * 12) / 2,
                        BTN_Y + (BTN_H - 16) / 2);
          tft.print(lbl);
        }
      }
      break;

    // ------------------------------------------------------------------
    case OTA_STATE_UPDATE_AVAILABLE:
      if (stateChanged || selChanged) {
        if (stateChanged) {
          tft.fillRect(0, CY, SCREEN_WIDTH, CH, COLOR_BG);
          tft.setFreeFont(nullptr);

          tft.setTextSize(3);
          tft.setTextColor(COLOR_WARNING, COLOR_BG);
          {
            const char* ln = "Update Available!";
            tft.setCursor((SCREEN_WIDTH - (int)strlen(ln) * 18) / 2, CY + 10);
            tft.print(ln);
          }

          tft.setTextSize(2);
          tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
          {
            char ln[40];
            snprintf(ln, sizeof(ln), "Installed:  %s", FIRMWARE_VERSION);
            tft.setCursor((SCREEN_WIDTH - (int)strlen(ln) * 12) / 2, CY + 52);
            tft.print(ln);
          }
          {
            char ln[40];
            snprintf(ln, sizeof(ln), "Available:  %s", detail ? detail : "");
            tft.setCursor((SCREEN_WIDTH - (int)strlen(ln) * 12) / 2, CY + 74);
            tft.print(ln);
          }
        }

        // Two button options at the bottom of the content area
        const int BTN_Y   = CY + 120;
        const int BTN_H   = 40;
        const int BTN_W   = 160;
        const int BTN0_X  = (SCREEN_WIDTH / 2) - BTN_W - 10;
        const int BTN1_X  = (SCREEN_WIDTH / 2) + 10;

        // Install button
        {
          bool sel = (selectedOption == 0);
          tft.fillRect(BTN0_X, BTN_Y, BTN_W, BTN_H,
                       sel ? COLOR_SUCCESS : COLOR_BG);
          tft.drawRect(BTN0_X, BTN_Y, BTN_W, BTN_H, COLOR_SUCCESS);
          tft.setTextColor(sel ? TFT_BLACK : COLOR_SUCCESS, sel ? COLOR_SUCCESS : COLOR_BG);
          tft.setTextSize(2);
          const char* lbl = " INSTALL";
          tft.setCursor(BTN0_X + (BTN_W - (int)strlen(lbl) * 12) / 2,
                        BTN_Y + (BTN_H - 16) / 2);
          tft.print(lbl);
        }
        // Cancel button
        {
          bool sel = (selectedOption == 1);
          tft.fillRect(BTN1_X, BTN_Y, BTN_W, BTN_H,
                       sel ? COLOR_TEXT_SECONDARY : COLOR_BG);
          tft.drawRect(BTN1_X, BTN_Y, BTN_W, BTN_H, COLOR_TEXT_SECONDARY);
          tft.setTextColor(sel ? TFT_BLACK : COLOR_TEXT_SECONDARY,
                           sel ? COLOR_TEXT_SECONDARY : COLOR_BG);
          tft.setTextSize(2);
          const char* lbl = " CANCEL";
          tft.setCursor(BTN1_X + (BTN_W - (int)strlen(lbl) * 12) / 2,
                        BTN_Y + (BTN_H - 16) / 2);
          tft.print(lbl);
        }
      }
      break;

    // ------------------------------------------------------------------
    case OTA_STATE_DOWNLOADING:
      if (stateChanged) {
        tft.fillRect(0, CY, SCREEN_WIDTH, CH, COLOR_BG);
        tft.setFreeFont(nullptr);
        tft.setTextSize(3);
        tft.setTextColor(COLOR_TEXT_PRIMARY, COLOR_BG);
        {
          const char* ln = "Downloading...";
          tft.setCursor((SCREEN_WIDTH - (int)strlen(ln) * 18) / 2, MCY - 50);
          tft.print(ln);
        }
        tft.setTextSize(2);
        tft.setTextColor(COLOR_WARNING, COLOR_BG);
        {
          const char* ln = "Do not power off";
          tft.setCursor((SCREEN_WIDTH - (int)strlen(ln) * 12) / 2, MCY + 40);
          tft.print(ln);
        }
      }
      // Progress bar (updated every loop call)
      if (stateChanged || progressChanged) {
        const int PBX = 40;
        const int PBY = MCY - 16;
        const int PBW = SCREEN_WIDTH - 80;
        const int PBH = 24;

        tft.drawRect(PBX, PBY, PBW, PBH, COLOR_TEXT_SECONDARY);
        int filled = (int)((uint32_t)(PBW - 4) * (uint32_t)progress / 100);
        tft.fillRect(PBX + 2, PBY + 2, filled,       PBH - 4, COLOR_SUCCESS);
        tft.fillRect(PBX + 2 + filled, PBY + 2,
                     PBW - 4 - filled, PBH - 4, COLOR_BG);

        tft.setFreeFont(nullptr);
        tft.setTextSize(2);
        tft.setTextColor(COLOR_SUCCESS, COLOR_BG);
        char pctStr[8];
        snprintf(pctStr, sizeof(pctStr), "%3d%%", progress);
        tft.setCursor((SCREEN_WIDTH - 4 * 12) / 2, PBY + PBH + 6);
        tft.print(pctStr);
      }
      break;

    // ------------------------------------------------------------------
    case OTA_STATE_SUCCESS:
      if (stateChanged) {
        tft.fillRect(0, CY, SCREEN_WIDTH, CH, COLOR_BG);
        tft.setFreeFont(nullptr);

        tft.setTextSize(3);
        tft.setTextColor(COLOR_SUCCESS, COLOR_BG);
        {
          const char* ln = "Update Complete!";
          tft.setCursor((SCREEN_WIDTH - (int)strlen(ln) * 18) / 2, MCY - 30);
          tft.print(ln);
        }

        tft.setTextSize(2);
        tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
        {
          const char* ln = "Rebooting in 3 seconds...";
          tft.setCursor((SCREEN_WIDTH - (int)strlen(ln) * 12) / 2, MCY + 10);
          tft.print(ln);
        }
      }
      break;

    // ------------------------------------------------------------------
    case OTA_STATE_FAILED:
      if (stateChanged) {
        tft.fillRect(0, CY, SCREEN_WIDTH, CH, COLOR_BG);
        tft.setFreeFont(nullptr);

        tft.setTextSize(3);
        tft.setTextColor(COLOR_ERROR, COLOR_BG);
        {
          const char* ln = "Update Failed";
          tft.setCursor((SCREEN_WIDTH - (int)strlen(ln) * 18) / 2, MCY - 40);
          tft.print(ln);
        }

        if (detail && detail[0]) {
          tft.setTextSize(2);
          tft.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
          // Wrap to two lines if too long
          int len = (int)strlen(detail);
          if (len * 12 <= SCREEN_WIDTH - 20) {
            tft.setCursor((SCREEN_WIDTH - len * 12) / 2, MCY);
            tft.print(detail);
          } else {
            // Split at the nearest space around the midpoint
            char buf[96];
            strlcpy(buf, detail, sizeof(buf));
            int mid = len / 2;
            int split = mid;
            while (split > 0 && buf[split] != ' ') split--;
            if (split == 0) split = mid;
            buf[split] = '\0';
            tft.setCursor(10, MCY - 4);
            tft.print(buf);
            tft.setCursor(10, MCY + 18);
            tft.print(buf + split + 1);
          }
        }
      }
      break;

    // ------------------------------------------------------------------
    case OTA_STATE_CANCELLED:
    default:
      break;
  }

  // ---- Footer ----
  if (stateChanged) {
    tft.fillRect(0, 278, SCREEN_WIDTH, 42, COLOR_BG);
    tft.drawFastHLine(0, 278, SCREEN_WIDTH, COLOR_TEXT_SECONDARY);
    tft.setFreeFont(nullptr);

    const char* footerText = nullptr;
    uint16_t    footerColor = COLOR_TEXT_SECONDARY;

    switch (state) {
      case OTA_STATE_WAITING_CREDS:
      case OTA_STATE_STARTING_AP:
        footerText  = "Press button to cancel";
        footerColor = COLOR_TEXT_SECONDARY;
        break;
      case OTA_STATE_VERSION_CURRENT:
        footerText  = "Rotate to select, press to confirm";
        footerColor = COLOR_TEXT_SECONDARY;
        break;
      case OTA_STATE_FAILED:
      case OTA_STATE_CANCELLED:
        footerText  = "Press button to return";
        footerColor = COLOR_SUCCESS;
        break;
      case OTA_STATE_UPDATE_AVAILABLE:
        footerText  = "Rotate to select, press to confirm";
        footerColor = COLOR_TEXT_SECONDARY;
        break;
      case OTA_STATE_DOWNLOADING:
      case OTA_STATE_SUCCESS:
        footerText  = nullptr;
        break;
      default:
        break;
    }

    if (footerText) {
      int len = (int)strlen(footerText);
      tft.setTextSize(2);
      tft.setTextColor(footerColor, COLOR_BG);
      tft.setCursor((SCREEN_WIDTH - len * 12) / 2, 286);
      tft.print(footerText);
    }
  }

  tft.setFreeFont(nullptr);
  tft.setTextSize(1);
}

// ---------------------------------------------------------------------------
// Rollback confirmation popup
//
// Shown at startup when a new firmware has not yet been validated.
// Full-screen modal: red banner top, white content area with countdown.
// ---------------------------------------------------------------------------
void drawRollbackPopup(const char* newVersion, uint32_t secondsRemaining,
                       uint8_t selectedOption)
{
  static uint32_t lastSeconds       = UINT32_MAX;
  static uint8_t  lastSelectedOption = 255;
  static bool     chromePainted      = false;

  bool selChanged     = (selectedOption != lastSelectedOption);
  bool timeChanged    = (secondsRemaining != lastSeconds);
  bool needFullRepaint = !chromePainted || selChanged;

  lastSeconds        = secondsRemaining;
  lastSelectedOption = selectedOption;

  const int OX = 20;
  const int OY = 30;
  const int OW = SCREEN_WIDTH  - OX * 2;   // 440
  const int OH = SCREEN_HEIGHT - OY * 2;   // 260
  const int bannerH = 90;

  if (needFullRepaint) {
    chromePainted = true;
    tft.fillRect(OX, OY, OW, OH, TFT_BLACK);
    for (int t = 0; t < 3; t++) {
      tft.drawRect(OX + t, OY + t, OW - t * 2, OH - t * 2, COLOR_WARNING);
    }

    // ---- Orange/amber banner ----
    tft.fillRect(OX + 3, OY + 3, OW - 6, bannerH - 3, COLOR_WARNING);
    tft.setTextColor(TFT_BLACK, COLOR_WARNING);
    tft.setFreeFont(nullptr);

    tft.setTextSize(3);
    {
      const char* t1 = "NEW FIRMWARE";
      tft.setCursor(OX + (OW - (int)strlen(t1) * 18) / 2, OY + 8);
      tft.print(t1);
    }
    tft.setTextSize(2);
    {
      char t2[40];
      snprintf(t2, sizeof(t2), "Version: %s", newVersion ? newVersion : "unknown");
      tft.setCursor(OX + (OW - (int)strlen(t2) * 12) / 2, OY + 42);
      tft.print(t2);
    }
    tft.setTextSize(2);
    {
      const char* t3 = "Confirm new firmware is working";
      tft.setCursor(OX + (OW - (int)strlen(t3) * 12) / 2, OY + 64);
      tft.print(t3);
    }

    // ---- White content area ----
    int botY = OY + bannerH;
    int botH = OH - bannerH;
    tft.fillRect(OX + 3, botY, OW - 6, botH - 3, TFT_WHITE);

    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.setTextSize(2);
    {
      const char* l1 = "Press button to confirm.";
      tft.setCursor(OX + (OW - (int)strlen(l1) * 12) / 2, botY + 10);
      tft.print(l1);
    }
    {
      const char* l2 = "Auto-rollback if not confirmed:";
      tft.setCursor(OX + (OW - (int)strlen(l2) * 12) / 2, botY + 32);
      tft.print(l2);
    }

    // ---- Two buttons: CONFIRM / ROLLBACK ----
    const int BTN_Y  = botY + 65;
    const int BTN_H  = 36;
    const int BTN_W  = 150;
    const int BTN0_X = OX + (OW / 2) - BTN_W - 8;
    const int BTN1_X = OX + (OW / 2) + 8;

    {
      bool sel = (selectedOption == 0);
      tft.fillRect(BTN0_X, BTN_Y, BTN_W, BTN_H,
                   sel ? (uint16_t)TFT_DARKGREEN : (uint16_t)TFT_WHITE);
      tft.drawRect(BTN0_X, BTN_Y, BTN_W, BTN_H, TFT_DARKGREEN);
      tft.setTextColor(sel ? TFT_WHITE : TFT_DARKGREEN,
                       sel ? (uint16_t)TFT_DARKGREEN : (uint16_t)TFT_WHITE);
      tft.setTextSize(2);
      const char* lbl = "CONFIRM";
      tft.setCursor(BTN0_X + (BTN_W - (int)strlen(lbl) * 12) / 2,
                    BTN_Y + (BTN_H - 16) / 2);
      tft.print(lbl);
    }
    {
      bool sel = (selectedOption == 1);
      tft.fillRect(BTN1_X, BTN_Y, BTN_W, BTN_H,
                   sel ? (uint16_t)TFT_RED : (uint16_t)TFT_WHITE);
      tft.drawRect(BTN1_X, BTN_Y, BTN_W, BTN_H, TFT_RED);
      tft.setTextColor(sel ? TFT_WHITE : TFT_RED,
                       sel ? (uint16_t)TFT_RED : (uint16_t)TFT_WHITE);
      tft.setTextSize(2);
      const char* lbl = "ROLLBACK";
      tft.setCursor(BTN1_X + (BTN_W - (int)strlen(lbl) * 12) / 2,
                    BTN_Y + (BTN_H - 16) / 2);
      tft.print(lbl);
    }
  }

  // ---- Countdown timer (redrawn each second) ----
  if (needFullRepaint || timeChanged) {
    int botY = OY + bannerH;
    int timeY = botY + 48;
    // Erase only the countdown cell
    tft.fillRect(OX + 3, timeY - 2, OW - 6, 20, TFT_WHITE);
    tft.setFreeFont(nullptr);
    tft.setTextColor(TFT_RED, TFT_WHITE);
    tft.setTextSize(2);
    char cntStr[16];
    snprintf(cntStr, sizeof(cntStr), "%lus", (unsigned long)secondsRemaining);
    tft.setCursor(OX + (OW - (int)strlen(cntStr) * 12) / 2, timeY);
    tft.print(cntStr);
  }

  tft.setFreeFont(nullptr);
  tft.setTextSize(1);
}
