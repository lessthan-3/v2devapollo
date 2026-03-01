#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <ESP32Encoder.h>
#include <Preferences.h>
#include "pressure_sensor.h"
#include "motor_control.h"
#include "dual_core_motor.h"
#include "display_ui.h"

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

// Runtime auto-pause (seconds). Set to 0 to disable auto-pause.
#define RUNTIME_AUTO_PAUSE_SEC  0

// Firmware version
#define FIRMWARE_VERSION    "2.0.0"

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

// Screen state enumeration
typedef enum {
  SCREEN_STARTUP = 0,
  SCREEN_MENU,
  SCREEN_RUNTIME
} ScreenState;

// Global state variables
float targetPsi = TARGET_PSI_DEFAULT;
int64_t lastEncoderCount = 0;
unsigned long minMaxFlashStart = 0;
bool showingMinMax = false;
bool isMinNotMax = false;
uint32_t totalRuntimeTenths = 0;      // Total runtime in 0.1 hour increments (persistent)
uint32_t sessionStartMillis = 0;       // Session start time
unsigned long lastHourMeterSave = 0;   // Last time we saved hour meter

// Screen/menu state
ScreenState currentScreen = SCREEN_STARTUP;
uint8_t menuIndex = 0;
uint32_t runtimeStartMillis = 0;
uint32_t runtimePauseDeadlineSec = 0;

// Encoder mode state
EncoderMode currentEncoderMode = MODE_TARGET_PRESSURE;

// Debug display variables
unsigned long lastPidCalcTime = 0;     // Timestamp of last PID calculation
unsigned long pidLoopIntervalMs = 0;   // Time between PID calculations
float currentTemperatureC = -999.0f;   // Current temperature in Celsius

// Function declarations
void loadHourMeter();
void saveHourMeter();
void enterMenuScreen();
void enterRuntimeScreen();

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
  
  // Display startup screen
  drawStartupScreen();
  
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
  
  // Startup screen before menu
  delay(1500);
  enterMenuScreen();
  Serial.println("Entered menu screen; motor disabled");
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

void enterMenuScreen() {
  currentScreen = SCREEN_MENU;
  setMotorEnabledSafe(false);
  menuIndex = 0;
  encoder.setCount(menuIndex);
  lastEncoderCount = encoder.getCount();
  drawMenuScreen(menuIndex, true);
}

void enterRuntimeScreen() {
  currentScreen = SCREEN_RUNTIME;
  runtimeStartMillis = millis();

  setMotorEnabledSafe(true);
  drawRuntimeStatic();
  float currentPsi = 0.0f;
  bool pressureValid = false;
  uint32_t idleSecondsRemaining = UINT32_MAX;
  portENTER_CRITICAL(&motorShared.mutex);
  currentPsi = motorShared.smoothedPsi;
  pressureValid = motorShared.pressureValid;
  idleSecondsRemaining = motorShared.idleSecondsRemaining;
  portEXIT_CRITICAL(&motorShared.mutex);
  drawRuntimeTarget(targetPsi, currentPsi, pressureValid);
  drawRuntimeTemperature(currentTemperatureC);
  drawRuntimePauseCountdown(idleSecondsRemaining);
  drawRuntimeFooter();
}


void loop() {
  // Timing variables
  static unsigned long lastDisplayUpdate = 0;
  static unsigned long lastPauseUpdate = 0;
  static unsigned long lastHourMeterUpdate = 0;
  static float smoothedPressure = 0.0f;
  static bool displayValid = false;
  static uint32_t idleSecondsRemaining = UINT32_MAX;

  // Read shared data from motor control task (thread-safe)
  {
    portENTER_CRITICAL(&motorShared.mutex);
    smoothedPressure = motorShared.smoothedPsi;
    displayValid = motorShared.pressureValid;
    idleSecondsRemaining = motorShared.idleSecondsRemaining;
    uint32_t loopUs = motorShared.loopTimeUs;
    portEXIT_CRITICAL(&motorShared.mutex);
    pidLoopIntervalMs = loopUs / 1000;
  }

  // Encoder handling based on screen
  int64_t encoderCount = encoder.getCount();
  if (encoderCount != lastEncoderCount) {
    int64_t delta = encoderCount - lastEncoderCount;
    lastEncoderCount = encoderCount;

    if (currentScreen == SCREEN_MENU) {
      int32_t newIndex = (int32_t)menuIndex + (int32_t)delta;
      if (newIndex < 0) newIndex = 0;
      if (newIndex >= MENU_OPTION_COUNT) newIndex = MENU_OPTION_COUNT - 1;
      menuIndex = (uint8_t)newIndex;
      encoder.setCount(menuIndex);
      lastEncoderCount = encoder.getCount();
      drawMenuScreen(menuIndex, false);
    } else if (currentScreen == SCREEN_RUNTIME) {
      float newTarget = targetPsi + (delta * TARGET_PSI_STEP);
      if (newTarget < TARGET_PSI_MIN) newTarget = TARGET_PSI_MIN;
      if (newTarget > TARGET_PSI_MAX) newTarget = TARGET_PSI_MAX;
      targetPsi = newTarget;
      setTargetPressureSafe(targetPsi);
      drawRuntimeTarget(targetPsi, smoothedPressure, displayValid);
    }
  }

  // Button handling (simple debounce)
  static bool lastButtonState = false;
  static unsigned long lastButtonChange = 0;
  bool buttonPressed = (digitalRead(ENCODER_BTN) == LOW);
  if (buttonPressed != lastButtonState && (millis() - lastButtonChange) > 30) {
    lastButtonChange = millis();
    lastButtonState = buttonPressed;
    if (!buttonPressed) {
      if (currentScreen == SCREEN_MENU) {
        if (menuIndex == 0) {
          enterRuntimeScreen();
        } else if (menuIndex == 1) {
          drawMenuFooter("Settings: not implemented", TFT_ORANGE);
        } else if (menuIndex == 2) {
          drawMenuFooter("Firmware " FIRMWARE_VERSION, TFT_CYAN);
        }
      } else if (currentScreen == SCREEN_RUNTIME) {
        enterMenuScreen();
      }
    }
  }

  unsigned long now = millis();

  if (currentScreen == SCREEN_RUNTIME) {
    if (now - lastDisplayUpdate >= DISPLAY_PRESSURE_INTERVAL_MS) {
      lastDisplayUpdate = now;
      drawRuntimeTarget(targetPsi, smoothedPressure, displayValid);
      drawRuntimeTemperature(currentTemperatureC);
    }

    if (now - lastPauseUpdate >= 1000) {
      lastPauseUpdate = now;
      drawRuntimePauseCountdown(idleSecondsRemaining);
    }
  }

  // Update hour meter every 6 minutes (0.1 hours)
  if (millis() - lastHourMeterUpdate >= 360000) {
    lastHourMeterUpdate = millis();
    totalRuntimeTenths++;
    saveHourMeter();
  }
}