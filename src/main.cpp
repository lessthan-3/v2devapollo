#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <ESP32Encoder.h>
#include <Preferences.h>
#include "pressure_sensor.h"
#include "motor_control.h"
#include "dual_core_motor.h"

// Create TFT display instance
TFT_eSPI tft = TFT_eSPI();

// Rotary encoder instance
ESP32Encoder encoder;

// Preferences for storing hour meter
Preferences preferences;

// Encoder pins
#define ENCODER_CLK 41  // A (Right)
#define ENCODER_DT  42  // B (Left)
#define ENCODER_BTN 5   // Button

// Button detection workaround - use ADC instead of digital read
// TODO: Remove this workaround once board is fixed
#define USE_ADC_FOR_BUTTON 0  // Set to 0 to use digitalRead instead
#define BUTTON_ADC_THRESHOLD 2.5f  // Voltage threshold for button press
#define BUTTON_ADC_THRESHOLD_RAW ((uint16_t)((BUTTON_ADC_THRESHOLD / 3.3f) * 4095))  // ~3103 for 2.5V

// Backlight pin
#define TFT_BL 21

// Pressure reading interval
#define PRESSURE_READ_INTERVAL_MS 5  // Set to 0 for continuous reading (as fast as possible)

// Display update intervals (separate from data polling for performance)
#define DISPLAY_PRESSURE_INTERVAL_MS  100   // Update pressure display at 10Hz
#define DISPLAY_DEBUG_INTERVAL_MS     500   // Update debug info at 2Hz
#define SERIAL_DEBUG_INTERVAL_MS      1000  // Serial output at 1Hz

// Target pressure settings
#define TARGET_PSI_MIN      3.0f    // Minimum target PSI
#define TARGET_PSI_MAX      9.0f    // Maximum target PSI
#define TARGET_PSI_DEFAULT  6.0f    // Default target PSI
#define TARGET_PSI_STEP     0.1f    // Encoder step size

// Display layout constants (480x320 landscape)
#define SCREEN_WIDTH        480
#define SCREEN_HEIGHT       320
#define PRESSURE_ZONE_HEIGHT 213    // Top 2/3 for pressure displays
#define STATUS_ZONE_Y       220     // Bottom 1/3 starts here
#define LEFT_COLUMN_X       10      // Left pressure column
#define RIGHT_COLUMN_X      245     // Right pressure column
#define COLUMN_WIDTH        225     // Width of each pressure column

// MIN/MAX flash duration
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

// PID gain adjustment step sizes
#define KP_STEP             1.0f
#define KI_STEP             0.5f
#define KD_STEP             0.5f
#define KP_MIN              1.0f
#define KP_MAX              100.0f
#define KI_MIN              0.0f
#define KI_MAX              100.0f
#define KD_MIN              0.0f
#define KD_MAX              100.0f

// Encoder mode enumeration
typedef enum {
    MODE_TARGET_PRESSURE = 0,
    MODE_KP,
    MODE_KI,
    MODE_KD,
    MODE_COUNT  // Number of modes for cycling
} EncoderMode;

// Global state variables
float targetPsi = TARGET_PSI_DEFAULT;
int64_t lastEncoderCount = 0;
unsigned long minMaxFlashStart = 0;
bool showingMinMax = false;
bool isMinNotMax = false;
uint32_t totalRuntimeTenths = 0;      // Total runtime in 0.1 hour increments (persistent)
uint32_t sessionStartMillis = 0;       // Session start time
unsigned long lastHourMeterSave = 0;   // Last time we saved hour meter

// Encoder mode state
EncoderMode currentEncoderMode = MODE_TARGET_PRESSURE;

// Debug display variables
unsigned long lastPidCalcTime = 0;     // Timestamp of last PID calculation
unsigned long pidLoopIntervalMs = 0;   // Time between PID calculations
float currentTemperatureC = 0.0f;      // Current temperature in Celsius

// Function declarations
void drawStaticUI();
void drawTargetPressure(bool forceRedraw = false);
void drawCurrentPressure(float psi, bool valid);
void drawTemperature(float tempC);
void flashMinMax(bool isMin);
void updateMinMaxFlash();
void loadHourMeter();
void saveHourMeter();
void drawDebugInfo();
void drawModeIndicator();
void cycleEncoderMode();
const char* getModeName(EncoderMode mode);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Apollo Sprayers HVLP - ESP32-S3");
  Serial.println("Initializing...");

  // Initialize backlight pin
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);  // Turn on backlight

  // Initialize the display
  tft.init();
  tft.setRotation(1);  // Landscape mode (480x320)
  
  // Fill screen with black
  tft.fillScreen(COLOR_BG);
  
  // Display startup message
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
  tft.setCursor(180, 220);
  tft.println("Initializing...");
  
  Serial.println("Display initialized successfully!");
  
  // Initialize rotary encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachHalfQuad(ENCODER_DT, ENCODER_CLK);
  encoder.setCount((int64_t)(TARGET_PSI_DEFAULT / TARGET_PSI_STEP));  // Start at default
  lastEncoderCount = encoder.getCount();
  
  // Initialize encoder button
  pinMode(ENCODER_BTN, INPUT_PULLUP);

  
  Serial.println("Encoder initialized");
  tft.setCursor(180, 240);
  tft.setTextColor(TFT_GREEN, COLOR_BG);
  tft.println("Encoder: OK");
  
  // Note: Pressure sensor will be initialized on Core 0 in motorControlTask
  // This is required because Wire/I2C must be used from the same core it was initialized on
  tft.setCursor(180, 250);
  tft.setTextColor(TFT_YELLOW, COLOR_BG);
  tft.println("Pressure: Core0");
  
  // Load hour meter from flash
  loadHourMeter();
  tft.setCursor(180, 260);
  tft.setTextColor(TFT_GREEN, COLOR_BG);
  tft.printf("Hours: %lu.%lu", totalRuntimeTenths / 10, totalRuntimeTenths % 10);
  
  // Initialize motor control (zero crossing interrupt)
  delay(100);
  if (motorControlInit()) {
    tft.setCursor(180, 270);
    tft.setTextColor(TFT_GREEN, COLOR_BG);
    tft.println("Motor: OK");
    Serial.println("Motor control initialized");
    
    // Initialize the pressure PID controller (loads saved gains from NVS)
    pressurePidInit();
    tft.setCursor(180, 275);
    tft.setTextColor(TFT_GREEN, COLOR_BG);
    tft.println("PID: OK");
    
    // Sync initial PID gains to the shared data for dual-core motor task
    {
      PidController* pid = getPressurePid();
      float kp, ki, kd;
      pidGetGains(pid, &kp, &ki, &kd);
      setPidGainsSafe(kp, ki, kd);
      Serial.printf("Synced initial PID gains: Kp=%.1f Ki=%.1f Kd=%.1f\n", kp, ki, kd);
    }
    
    // Detect AC frequency
    AcFrequency acFreq = detectAcFrequency();
    tft.setCursor(180, 280);
    if (acFreq != AC_FREQ_UNKNOWN) {
      tft.setTextColor(TFT_GREEN, COLOR_BG);
      tft.printf("AC: %dHz", acFreq);
    } else {
      tft.setTextColor(TFT_ORANGE, COLOR_BG);
      tft.println("AC: Unknown");
    }
    
    // Initialize dual-core motor control (runs PID on Core 0)
    if (dualCoreMotorInit()) {
      tft.setCursor(180, 290);
      tft.setTextColor(TFT_GREEN, COLOR_BG);
      tft.println("Dual-Core: OK");
      Serial.println("Dual-core motor control initialized");
      
      // Set initial target pressure
      setTargetPressureSafe(TARGET_PSI_DEFAULT);
    } else {
      tft.setCursor(180, 290);
      tft.setTextColor(TFT_RED, COLOR_BG);
      tft.println("Dual-Core: FAIL");
      Serial.println("Dual-core motor control FAILED");
    }
  } else {
    tft.setCursor(180, 270);
    tft.setTextColor(TFT_RED, COLOR_BG);
    tft.println("Motor: FAIL");
    Serial.println("Motor control FAILED");
  }
  
  // Record session start time
  sessionStartMillis = millis();
  
  delay(2000);
  
  // Clear screen and draw main UI
  tft.fillScreen(COLOR_BG);
  drawStaticUI();
  drawTargetPressure(true);
  drawTemperature(-999.0f);  // Initialize with invalid temp
  drawDebugInfo();
  drawModeIndicator();
  
  // Enable motor (PID on Core 0 will control speed based on pressure)
  setMotorEnabledSafe(true);
  Serial.println("Motor enabled with dual-core PID pressure control");
}

/**
 * @brief Load hour meter from flash storage
 */
void loadHourMeter() {
  if (preferences.begin("apollo", true)) {  // Read-only
    totalRuntimeTenths = preferences.getULong("hourTenths", 0);
    preferences.end();
    Serial.printf("Hour meter loaded: %lu.%lu hours\n", totalRuntimeTenths / 10, totalRuntimeTenths % 10);
  } else {
    // NVS namespace doesn't exist yet, create it
    Serial.println("Hour meter: No saved data, initializing...");
    preferences.end();
    
    // Create the namespace by writing initial value
    if (preferences.begin("apollo", false)) {  // Read-write to create
      preferences.putULong("hourTenths", 0);
      preferences.end();
      Serial.println("Hour meter: Initialized to 0.0 hours");
    } else {
      Serial.println("Hour meter: WARNING - NVS not available, data won't persist");
    }
    totalRuntimeTenths = 0;
  }
}

/**
 * @brief Save hour meter to flash storage
 */
void saveHourMeter() {
  if (preferences.begin("apollo", false)) {  // Read-write
    preferences.putULong("hourTenths", totalRuntimeTenths);
    preferences.end();
    Serial.printf("Hour meter saved: %lu.%lu hours\n", totalRuntimeTenths / 10, totalRuntimeTenths % 10);
  } else {
    Serial.println("Hour meter: WARNING - Failed to save to NVS");
  }
}

/**
 * @brief Draw the static UI elements (labels and dividers)
 */
void drawStaticUI() {
  // Draw horizontal divider between pressure zone and status zone
  tft.drawFastHLine(0, STATUS_ZONE_Y - 5, SCREEN_WIDTH, TFT_DARKGREY);
  
  // Draw vertical divider between target and current pressure
  tft.drawFastVLine(SCREEN_WIDTH / 2, 0, STATUS_ZONE_Y - 5, TFT_DARKGREY);
  
  // Target pressure label (left side)
  tft.setTextColor(COLOR_LABEL, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(LEFT_COLUMN_X + 50, 15);
  tft.print("TARGET");
  
  // Current pressure label (right side)
  tft.setCursor(RIGHT_COLUMN_X + 40, 15);
  tft.print("CURRENT");
  
  // Status zone labels
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

/**
 * @brief Draw the target pressure value
 * @param forceRedraw Force full redraw even if value unchanged
 */
void drawTargetPressure(bool forceRedraw) {
  static float lastDisplayedTarget = -1.0f;
  
  if (!forceRedraw && targetPsi == lastDisplayedTarget && !showingMinMax) {
    return;
  }
  
  lastDisplayedTarget = targetPsi;
  
  // Clear the target pressure area
  tft.fillRect(LEFT_COLUMN_X, 50, COLUMN_WIDTH, 150, COLOR_BG);
  
  if (showingMinMax) {
    // Display MIN or MAX in red
    tft.setTextColor(COLOR_MINMAX, COLOR_BG);
    tft.setTextSize(5);
    tft.setCursor(LEFT_COLUMN_X + 40, 90);
    tft.print(isMinNotMax ? "MIN" : "MAX");
  } else {
    // Display target PSI value
    tft.setTextColor(COLOR_TARGET, COLOR_BG);
    tft.setTextSize(6);
    tft.setCursor(LEFT_COLUMN_X + 20, 70);
    
    char psiStr[10];
    snprintf(psiStr, sizeof(psiStr), "%4.1f", targetPsi);
    tft.print(psiStr);
    
    // PSI unit label
    tft.setTextSize(3);
    tft.setCursor(LEFT_COLUMN_X + 60, 140);
    tft.print("PSI");
  }
}

/**
 * @brief Draw the current pressure reading
 * @param psi Current pressure in PSI
 * @param valid Whether the reading is valid
 * 
 * Optimized to minimize display updates:
 * - Uses 0.1 PSI threshold to reduce redraws
 * - Only clears/redraws the number area, not the PSI label
 * - Skips redundant draws when value unchanged
 */
void drawCurrentPressure(float psi, bool valid) {
  static float lastDisplayedCurrent = -999.0f;
  static bool lastValid = true;
  static bool labelDrawn = false;
  
  // Only redraw if value changed significantly (0.1 PSI = one decimal place)
  if (valid == lastValid && valid && fabsf(psi - lastDisplayedCurrent) < 0.1f) {
    return;
  }
  
  // Check if validity state changed (requires full redraw)
  bool validityChanged = (valid != lastValid);
  lastDisplayedCurrent = psi;
  lastValid = valid;
  
  if (!valid) {
    // Error state - full redraw needed
    tft.fillRect(RIGHT_COLUMN_X, 50, COLUMN_WIDTH, 150, COLOR_BG);
    tft.setTextColor(TFT_RED, COLOR_BG);
    tft.setTextSize(4);
    tft.setCursor(RIGHT_COLUMN_X + 30, 90);
    tft.print("ERROR");
    labelDrawn = false;
  } else {
    // Only clear the number area (not the PSI label) for faster updates
    // Number area is roughly 200x70 pixels starting at y=70
    tft.fillRect(RIGHT_COLUMN_X + 5, 65, 210, 70, COLOR_BG);
    
    tft.setTextColor(COLOR_CURRENT, COLOR_BG);
    tft.setTextSize(6);
    tft.setCursor(RIGHT_COLUMN_X + 10, 70);
    
    char psiStr[10];
    snprintf(psiStr, sizeof(psiStr), "%5.1f", psi);
    tft.print(psiStr);
    
    // Only draw PSI label once or after error state
    if (!labelDrawn || validityChanged) {
      tft.fillRect(RIGHT_COLUMN_X + 45, 135, 60, 30, COLOR_BG);
      tft.setTextSize(3);
      tft.setCursor(RIGHT_COLUMN_X + 50, 140);
      tft.print("PSI");
      labelDrawn = true;
    }
  }
}

/**
 * @brief Draw the temperature display
 * @param tempC Temperature in Celsius
 */
void drawTemperature(float tempC) {
  static float lastTemp = -999.0f;
  
  // Only update if changed significantly
  if (fabsf(tempC - lastTemp) < 0.5f && lastTemp != -999.0f) {
    return;
  }
  lastTemp = tempC;
  currentTemperatureC = tempC;
  
  // Clear temperature value area
  tft.fillRect(LEFT_COLUMN_X, STATUS_ZONE_Y + 18, 65, 30, COLOR_BG);
  
  tft.setTextColor(COLOR_TEMP, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(LEFT_COLUMN_X, STATUS_ZONE_Y + 22);
  
  if (tempC < -100.0f) {
    tft.print("--");
  } else {
    // Convert to Fahrenheit for display
    float tempF = (tempC * 9.0f / 5.0f) + 32.0f;
    char tempStr[10];
    snprintf(tempStr, sizeof(tempStr), "%3.0f", tempF);
    tft.print(tempStr);
  }
  tft.setTextSize(1);
  tft.print("F");
}

/**
 * @brief Draw the debug info display (motor speed, AC freq, loop time, PID values)
 */
void drawDebugInfo() {
  static uint8_t lastMotorSpeed = 255;
  static AcFrequency lastAcFreq = AC_FREQ_UNKNOWN;
  static unsigned long lastLoopTime = 9999;
  static float lastKp = -1, lastKi = -1, lastKd = -1;
  static uint32_t lastZcCount = 0xFFFFFFFF;
  
  // Get current values
  uint8_t motorSpeed = motorState.motorSpeed;
  AcFrequency acFreq = motorState.acFrequency;
  uint32_t zeroCrossingCount = getZeroCrossingCount();
  PidController* pid = getPressurePid();
  float kp, ki, kd;
  pidGetGains(pid, &kp, &ki, &kd);
  
  // Motor speed
  if (motorSpeed != lastMotorSpeed) {
    lastMotorSpeed = motorSpeed;
    tft.fillRect(LEFT_COLUMN_X + 80, STATUS_ZONE_Y + 18, 50, 25, COLOR_BG);
    tft.setTextColor(COLOR_RUNTIME, COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(LEFT_COLUMN_X + 80, STATUS_ZONE_Y + 22);
    tft.printf("%3d%%", motorSpeed);
  }
  
  // AC Frequency
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
  
  // Loop time
  if (pidLoopIntervalMs != lastLoopTime) {
    lastLoopTime = pidLoopIntervalMs;
    tft.fillRect(LEFT_COLUMN_X + 200, STATUS_ZONE_Y + 18, 60, 25, COLOR_BG);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(LEFT_COLUMN_X + 200, STATUS_ZONE_Y + 22);
    tft.printf("%3lu", pidLoopIntervalMs);
  }
  
  // PID values
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
    tft.setCursor(LEFT_COLUMN_X + 50, STATUS_ZONE_Y -28);
    tft.printf("%lu", (unsigned long)zeroCrossingCount);
  }
}

/**
 * @brief Get mode name string
 */
const char* getModeName(EncoderMode mode) {
  switch (mode) {
    case MODE_TARGET_PRESSURE: return "TARGET";
    case MODE_KP: return "Kp";
    case MODE_KI: return "Ki";
    case MODE_KD: return "Kd";
    default: return "???";
  }
}

/**
 * @brief Draw the mode indicator at bottom of screen
 */
void drawModeIndicator() {
  static EncoderMode lastMode = MODE_COUNT;  // Invalid initial value to force first draw
  
  // Only redraw if mode changed
  if (lastMode == currentEncoderMode) {
    // Still update the session time
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
  
  // Clear mode indicator area
  tft.fillRect(0, STATUS_ZONE_Y + 50, SCREEN_WIDTH, 30, COLOR_BG);
  
  // Draw current mode with large highlight
  tft.setTextSize(2);
  tft.setCursor(LEFT_COLUMN_X, STATUS_ZONE_Y + 55);
  
  // Show current mode prominently
  tft.setTextColor(TFT_BLACK, TFT_YELLOW);
  tft.printf(" MODE: %s ", getModeName(currentEncoderMode));
  
  // Show all modes with highlight on current
  tft.setCursor(LEFT_COLUMN_X + 150, STATUS_ZONE_Y + 55);
  for (int i = 0; i < MODE_COUNT; i++) {
    if (i == currentEncoderMode) {
      tft.setTextColor(TFT_BLACK, TFT_GREEN);  // Highlighted
    } else {
      tft.setTextColor(TFT_DARKGREY, COLOR_BG);  // Dimmed
    }
    tft.printf("[%d]", i);
  }
  
  // Show session/total hours on the right
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
  
  Serial.printf("Mode indicator updated: %s (mode %d)\n", getModeName(currentEncoderMode), currentEncoderMode);
}

/**
 * @brief Cycle to next encoder mode
 */
void cycleEncoderMode() {
  EncoderMode oldMode = currentEncoderMode;
  currentEncoderMode = (EncoderMode)((currentEncoderMode + 1) % MODE_COUNT);
  
  Serial.printf("cycleEncoderMode: %d -> %d (%s -> %s)\n", 
                oldMode, currentEncoderMode, 
                getModeName(oldMode), getModeName(currentEncoderMode));
  
  // Update encoder count to match current value for smooth transitions
  PidController* pid = getPressurePid();
  float kp, ki, kd;
  pidGetGains(pid, &kp, &ki, &kd);
  
  switch (currentEncoderMode) {
    case MODE_TARGET_PRESSURE:
      encoder.setCount((int64_t)(targetPsi / TARGET_PSI_STEP));
      Serial.printf("  Set encoder for TARGET: count=%lld\n", encoder.getCount());
      break;
    case MODE_KP:
      encoder.setCount((int64_t)(kp / KP_STEP));
      Serial.printf("  Set encoder for Kp: count=%lld, kp=%.1f\n", encoder.getCount(), kp);
      break;
    case MODE_KI:
      encoder.setCount((int64_t)(ki / KI_STEP));
      Serial.printf("  Set encoder for Ki: count=%lld, ki=%.1f\n", encoder.getCount(), ki);
      break;
    case MODE_KD:
      encoder.setCount((int64_t)(kd / KD_STEP));
      Serial.printf("  Set encoder for Kd: count=%lld, kd=%.1f\n", encoder.getCount(), kd);
      break;
    default:
      break;
  }
  lastEncoderCount = encoder.getCount();
  
  // Force redraw of mode indicator by invalidating the static cache
  // Clear and redraw entire mode area
  tft.fillRect(0, STATUS_ZONE_Y + 50, SCREEN_WIDTH, 30, COLOR_BG);
  
  // Draw current mode with large highlight
  tft.setTextSize(2);
  tft.setCursor(LEFT_COLUMN_X, STATUS_ZONE_Y + 55);
  tft.setTextColor(TFT_BLACK, TFT_YELLOW);
  tft.printf(" MODE: %s ", getModeName(currentEncoderMode));
  
  // Show mode number
  tft.setCursor(LEFT_COLUMN_X + 150, STATUS_ZONE_Y + 55);
  for (int i = 0; i < MODE_COUNT; i++) {
    if (i == currentEncoderMode) {
      tft.setTextColor(TFT_BLACK, TFT_GREEN);
    } else {
      tft.setTextColor(TFT_DARKGREY, COLOR_BG);
    }
    tft.printf("[%d]", i);
  }
  
  Serial.printf("Mode changed to: %s\n", getModeName(currentEncoderMode));
}

/**
 * @brief Flash MIN or MAX when user hits boundary
 * @param isMin True for MIN, false for MAX
 */
void flashMinMax(bool isMin) {
  showingMinMax = true;
  isMinNotMax = isMin;
  minMaxFlashStart = millis();
  drawTargetPressure(true);
}

/**
 * @brief Update MIN/MAX flash state (call in loop)
 */
void updateMinMaxFlash() {
  if (showingMinMax && (millis() - minMaxFlashStart >= MINMAX_FLASH_MS)) {
    showingMinMax = false;
    drawTargetPressure(true);
  }
}

void loop() {
  // Timing variables
  static unsigned long lastPressureRead = 0;
  static unsigned long lastDisplayUpdate = 0;
  static unsigned long lastDebugUpdate = 0;
  static unsigned long lastSerialDebug = 0;
  static unsigned long lastHourMeterUpdate = 0;
  static float smoothedPressure = 0.0f;
  static float displayPressure = 0.0f;  // Cached value for display
  static bool displayValid = false;
  static uint8_t displayMotorSpeed = 0;
  
  // Handle MIN/MAX flash timeout
  updateMinMaxFlash();
  
  // Read encoder and update value based on current mode
  int64_t encoderCount = encoder.getCount();
  if (encoderCount != lastEncoderCount) {
    int64_t delta = encoderCount - lastEncoderCount;
    lastEncoderCount = encoderCount;
    
    PidController* pid = getPressurePid();
    float kp, ki, kd;
    pidGetGains(pid, &kp, &ki, &kd);
    
    switch (currentEncoderMode) {
      case MODE_TARGET_PRESSURE: {
        float newTarget = targetPsi + (delta * TARGET_PSI_STEP);
        
        // Check boundaries
        if (newTarget < TARGET_PSI_MIN) {
          newTarget = TARGET_PSI_MIN;
          encoder.setCount((int64_t)(TARGET_PSI_MIN / TARGET_PSI_STEP));
          lastEncoderCount = encoder.getCount();
          if (delta < 0) flashMinMax(true);
        } else if (newTarget > TARGET_PSI_MAX) {
          newTarget = TARGET_PSI_MAX;
          encoder.setCount((int64_t)(TARGET_PSI_MAX / TARGET_PSI_STEP));
          lastEncoderCount = encoder.getCount();
          if (delta > 0) flashMinMax(false);
        }
        
        targetPsi = newTarget;
        setTargetPressureSafe(targetPsi);  // Thread-safe update to motor task
        
        if (!showingMinMax) {
          drawTargetPressure();
        }
        Serial.printf("Target PSI: %.1f\n", targetPsi);
        break;
      }
      
      case MODE_KP: {
        float newKp = kp + (delta * KP_STEP);
        if (newKp < KP_MIN) newKp = KP_MIN;
        if (newKp > KP_MAX) newKp = KP_MAX;
        pidSetGains(pid, newKp, ki, kd);
        setPidGainsSafe(newKp, ki, kd);  // Sync to motor task
        drawDebugInfo();
        Serial.printf("Kp: %.1f\n", newKp);
        break;
      }
      
      case MODE_KI: {
        float newKi = ki + (delta * KI_STEP);
        if (newKi < KI_MIN) newKi = KI_MIN;
        if (newKi > KI_MAX) newKi = KI_MAX;
        pidSetGains(pid, kp, newKi, kd);
        setPidGainsSafe(kp, newKi, kd);  // Sync to motor task
        drawDebugInfo();
        Serial.printf("Ki: %.1f\n", newKi);
        break;
      }
      
      case MODE_KD: {
        float newKd = kd + (delta * KD_STEP);
        if (newKd < KD_MIN) newKd = KD_MIN;
        if (newKd > KD_MAX) newKd = KD_MAX;
        pidSetGains(pid, kp, ki, newKd);
        setPidGainsSafe(kp, ki, newKd);  // Sync to motor task
        drawDebugInfo();
        Serial.printf("Kd: %.1f\n", newKd);
        break;
      }
      
      default:
        break;
    }
  }
  

  bool buttonPressed = (digitalRead(ENCODER_BTN) == LOW);
  static bool lastButtonState = false;
  static unsigned long buttonPressStart = 0;
  static unsigned long lastButtonDebug = 0;
  
  
  if (buttonPressed != lastButtonState) {
    lastButtonState = buttonPressed;
    if (buttonPressed) {
      buttonPressStart = millis();
      Serial.println("Button PRESSED");
    } else {
      // Button released - check duration
      unsigned long pressDuration = millis() - buttonPressStart;
      Serial.printf("Button RELEASED after %lu ms\n", pressDuration);
      
      if (pressDuration >= 3000) {
        // Long press (3+ seconds) - reset PID to defaults
        Serial.println("Long press detected - RESET PID");
        PidController* pid = getPressurePid();
        pidResetToDefaults(pid);
        requestPidReset();  // Also reset the motor task's PID
        
        // Sync the reset gains to motor task
        float kp, ki, kd;
        pidGetGains(pid, &kp, &ki, &kd);
        setPidGainsSafe(kp, ki, kd);
        
        // Visual feedback
        tft.fillRect(0, 0, SCREEN_WIDTH, 30, TFT_RED);
        tft.setTextColor(TFT_WHITE, TFT_RED);
        tft.setTextSize(2);
        tft.setCursor(100, 8);
        tft.print("PID RESET TO DEFAULTS");
        delay(1000);
        tft.fillRect(0, 0, SCREEN_WIDTH, 30, COLOR_BG);
        drawStaticUI();
        drawDebugInfo();
        drawModeIndicator();
      } else if (pressDuration >= 1000) {
        // Medium press (1-3s) - save PID to NVS
        Serial.println("Medium press detected - SAVE PID");
        PidController* pid = getPressurePid();
        pidMarkAsLearned(pid, true);
        
        // Visual feedback
        tft.fillRect(0, 0, SCREEN_WIDTH, 30, TFT_GREEN);
        tft.setTextColor(TFT_WHITE, TFT_GREEN);
        tft.setTextSize(2);
        tft.setCursor(140, 8);
        tft.print("PID SAVED");
        delay(500);
        tft.fillRect(0, 0, SCREEN_WIDTH, 30, COLOR_BG);
        drawStaticUI();
      } else if (pressDuration >= 50) {
        // Short press - cycle encoder mode
        Serial.printf("Short press detected - CYCLE MODE (was: %s)\n", getModeName(currentEncoderMode));
        cycleEncoderMode();
        Serial.printf("New mode: %s\n", getModeName(currentEncoderMode));
      }
    }
  }
  
  // ========================================================================
  // FAST PATH: Read pressure data from motor task (non-blocking, every loop)
  // Motor control runs on Core 0 at high frequency, we just cache the results
  // ========================================================================
  {
    // Read shared data from motor control task (thread-safe, very fast)
    portENTER_CRITICAL(&motorShared.mutex);
    smoothedPressure = motorShared.smoothedPsi;
    displayValid = motorShared.pressureValid;
    displayMotorSpeed = motorShared.motorSpeed;
    uint32_t loopUs = motorShared.loopTimeUs;
    portEXIT_CRITICAL(&motorShared.mutex);
    
    // Update loop timing for display (but don't display yet)
    pidLoopIntervalMs = loopUs / 1000;
  }
  
  // ========================================================================
  // SLOW PATH: Update pressure display at reduced rate (10Hz)
  // ========================================================================
  unsigned long now = millis();
  if (now - lastDisplayUpdate >= DISPLAY_PRESSURE_INTERVAL_MS) {
    lastDisplayUpdate = now;
    
    if (displayValid) {
      displayPressure = smoothedPressure;
      drawCurrentPressure(displayPressure, true);
    } else {
      drawCurrentPressure(0.0f, false);
    }
  }
  

  
  // Update debug display and mode indicator at reduced rate
  if (now - lastDebugUpdate >= DISPLAY_DEBUG_INTERVAL_MS) {
    lastDebugUpdate = now;
    drawDebugInfo();
    drawModeIndicator();
    
    // Toggle activity indicator
    static bool toggle = false;
    toggle = !toggle;
    tft.fillRect(SCREEN_WIDTH - 15, 5, 10, 10, toggle ? TFT_GREEN : COLOR_BG);
  }
  
  // Update hour meter every 6 minutes (0.1 hours)
  if (millis() - lastHourMeterUpdate >= 360000) {
    lastHourMeterUpdate = millis();
    totalRuntimeTenths++;
    saveHourMeter();
  }
}