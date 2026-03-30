#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <ESP32Encoder.h>
#include <Preferences.h>
#include "pressure_sensor.h"
#include "motor_control.h"
#include "dual_core_motor.h"
#include "display_ui.h"
#include "temp_sensor.h"
#include "beeper.h"

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

// Debug serial output control
#define DEBUG_SERIAL_OUTPUT           1     // Set to 1 to enable, 0 to disable

// Temperature sensor polling and limits
#define TEMP_READ_INTERVAL_MS  1000   // Temperature update interval
#define TEMP_OVERHEAT_SETPOINT 115.0f // Overtemperature shutdown setpoint
#define TEMP_OVERHEAT_CLEAR    100.0f // Clear overtemperature below this

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
#define KP_STEP             0.05f
#define KI_STEP             0.05f
#define KD_STEP             0.05f
#define KP_MIN              0.25f
#define KP_MAX              100.0f
#define KI_MIN              0.0f
#define KI_MAX              100.0f
#define KD_MIN              0.0f
#define KD_MAX              100.0f

// Idle entry deviation tuning
#define IDLE_DEV_STEP       0.05f
#define IDLE_DEV_MIN        0.05f
#define IDLE_DEV_MAX        2.0f

// Power pause settings
#define POWER_PAUSE_SEC_MIN 20
#define POWER_PAUSE_SEC_MAX 600
#define POWER_PAUSE_SEC_STEP 1
#define POWER_PAUSE_WARN_SEC 10  // Fixed warning time

// Screen state enumeration
typedef enum {
  SCREEN_STARTUP = 0,
  SCREEN_MENU,
  SCREEN_RUNTIME,
  SCREEN_SETTINGS,
  SCREEN_POWERPAUSE_SETTINGS,
  SCREEN_SUPPORT,
  SCREEN_ABOUT
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

// Job time tracking (resets when motor starts)
uint32_t jobStartMillis = 0;           // Job start time
uint32_t jobElapsedMillis = 0;         // Total job elapsed time (excludes power pause)
uint32_t jobPauseStartMillis = 0;      // When current pause started
bool jobTimerActive = false;           // Whether job timer is running

// Screen/menu state
ScreenState currentScreen = SCREEN_STARTUP;
uint8_t menuIndex = 0;
int32_t menuScrollAccumulator = 0;
int32_t settingsScrollAccumulator = 0;
int32_t powerPauseScrollAccumulator = 0;
uint32_t runtimeStartMillis = 0;
uint32_t runtimePauseDeadlineSec = 0;

// Settings screen state
uint8_t settingsIndex = 0;
bool settingsEditing = false;
float settingsKp = PID_KP_DEFAULT;
float settingsKi = PID_KI_DEFAULT;
float settingsKd = PID_KD_DEFAULT;
float settingsIdleDev = IDLE_ENTRY_DEVIATION_PSI;
float settingsStartPsi = TARGET_PSI_DEFAULT;
bool settingsDirty = false;

// Power pause settings state
uint8_t powerPauseIndex = 0;
bool powerPauseEditing = false;
uint16_t powerPauseSeconds = IDLE_ENTRY_SECONDS;
bool powerPauseBeeperEnabled = true;
const uint16_t powerPauseWarnSeconds = POWER_PAUSE_WARN_SEC;  // Fixed at 10 seconds
DisplayUnits displayUnits = UNITS_IMPERIAL;
bool powerPauseDirty = false;

// Encoder mode state
EncoderMode currentEncoderMode = MODE_TARGET_PRESSURE;

// Debug display variables
unsigned long lastPidCalcTime = 0;     // Timestamp of last PID calculation
unsigned long pidLoopIntervalMs = 0;   // Time between PID calculations
float currentTemperatureC = -999.0f;   // Current temperature in Celsius
bool overTempActive = false;           // Overtemperature flag

// Function declarations
void loadHourMeter();
void saveHourMeter();
void enterMenuScreen();
void enterRuntimeScreen();
void enterSettingsScreen();
void enterSupportScreen();
void enterAboutScreen();
void syncPowerPauseSettings(bool saveToNvs);
void loadSettings();
void saveSettings();
void startJobTimer();
void pauseJobTimer();
void resumeJobTimer();
uint32_t getJobTimeSeconds();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Apollo Sprayers HVLP - ESP32-S3");
  Serial.println("Initializing...");

  beeperInit();

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

  // Initialize temperature sensor
  pinMode(TEMP_SENSOR_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(TEMP_SENSOR_PIN, ADC_11db);
  tempSensorInit();
  
  // Initialize encoder button
  pinMode(ENCODER_BTN, INPUT_PULLUP);

  
  Serial.println("Encoder initialized");
  // tft.setCursor(180, 240);
  // tft.setTextColor(TFT_GREEN, COLOR_BG);
  // tft.println("Encoder: OK");
  
  // Note: Pressure sensor will be initialized on Core 0 in motorControlTask
  // This is required because Wire/I2C must be used from the same core it was initialized on
  // tft.setCursor(180, 250);
  // tft.setTextColor(TFT_YELLOW, COLOR_BG);
  // tft.println("Pressure: Core0");
  
  // Load hour meter from flash
  loadHourMeter();
  // tft.setCursor(180, 260);
  // tft.setTextColor(TFT_GREEN, COLOR_BG);
  // tft.printf("Hours: %lu.%lu", totalRuntimeTenths / 10, totalRuntimeTenths % 10);

  // Load settings from flash
  loadSettings();

  targetPsi = settingsStartPsi;
  setTargetPressureSafe(targetPsi);
  encoder.setCount((int64_t)lroundf(targetPsi / TARGET_PSI_STEP));
  lastEncoderCount = encoder.getCount();
  
  // Initialize motor control (zero crossing interrupt)
  delay(100);
  if (motorControlInit()) {
    //tft.setCursor(180, 270);
    //tft.setTextColor(TFT_GREEN, COLOR_BG);
    //tft.println("Motor: OK");
    Serial.println("Motor control initialized");
    
    // Initialize the pressure PID controller (loads saved gains from NVS)
    pressurePidInit();
    //tft.setCursor(180, 275);
    //tft.setTextColor(TFT_GREEN, COLOR_BG);
    //tft.println("PID: OK");
    
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
      //tft.setTextColor(TFT_GREEN, COLOR_BG);
      //tft.printf("AC: %dHz", acFreq);
    } else {
      //tft.setTextColor(TFT_ORANGE, COLOR_BG);
      //tft.println("AC: Unknown");
    }
    
    // Initialize dual-core motor control (runs PID on Core 0)
    if (dualCoreMotorInit()) {
      //tft.setCursor(180, 290);
      //tft.setTextColor(TFT_GREEN, COLOR_BG);
      //tft.println("Dual-Core: OK");
      Serial.println("Dual-core motor control initialized");
      
      // Set initial target pressure
      setTargetPressureSafe(TARGET_PSI_DEFAULT);
    } else {
      //tft.setCursor(180, 290);
      //tft.setTextColor(TFT_RED, COLOR_BG);
      //tft.println("Dual-Core: FAIL");
      Serial.println("Dual-core motor control FAILED");
    }
  } else {
    //tft.setCursor(180, 270);
    tft.setTextColor(TFT_RED, COLOR_BG);
    //tft.println("Motor: FAIL");
    Serial.println("Motor control FAILED");
  }
  
  // Record session start time
  sessionStartMillis = millis();
  
  delay(2000);
  
  // Startup screen before entering runtime
  delay(1500);
  enterRuntimeScreen();  // Changed from enterMenuScreen()
  Serial.println("Entered runtime screen; motor starts at 0 PSI");
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

void loadSettings() {
  if (preferences.begin("apollo", true)) {
    settingsIdleDev = preferences.getFloat("idleDev", IDLE_ENTRY_DEVIATION_PSI);
    settingsStartPsi = preferences.getFloat("startPsi", TARGET_PSI_DEFAULT);
    powerPauseSeconds = preferences.getUShort("idleSec", IDLE_ENTRY_SECONDS);
    powerPauseBeeperEnabled = preferences.getBool("warnBeep", true);
    // powerPauseWarnSeconds is now a constant
    displayUnits = (DisplayUnits)preferences.getUChar("units", UNITS_IMPERIAL);
    preferences.end();
  } else {
    preferences.end();
    settingsIdleDev = IDLE_ENTRY_DEVIATION_PSI;
    settingsStartPsi = TARGET_PSI_DEFAULT;
    powerPauseSeconds = IDLE_ENTRY_SECONDS;
    powerPauseBeeperEnabled = true;
    // powerPauseWarnSeconds is now a constant
  }
  settingsIdleDev = constrain(settingsIdleDev, IDLE_DEV_MIN, IDLE_DEV_MAX);
  settingsStartPsi = constrain(settingsStartPsi, TARGET_PSI_MIN, TARGET_PSI_MAX);
  powerPauseSeconds = constrain(powerPauseSeconds, POWER_PAUSE_SEC_MIN, POWER_PAUSE_SEC_MAX);
  // powerPauseWarnSeconds is now a constant
  setIdleEntryDeviationSafe(settingsIdleDev);
  setIdleEntrySecondsSafe(powerPauseSeconds);
}

void saveSettings() {
  if (preferences.begin("apollo", false)) {
    preferences.putFloat("idleDev", settingsIdleDev);
    preferences.putFloat("startPsi", settingsStartPsi);
    preferences.putUShort("idleSec", powerPauseSeconds);
    preferences.putBool("warnBeep", powerPauseBeeperEnabled);
    // powerPauseWarnSeconds is now a constant
    preferences.putUChar("units", (uint8_t)displayUnits);
    preferences.end();
  } else {
    preferences.end();
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
  menuScrollAccumulator = 0;
  encoder.setCount(menuIndex);
  lastEncoderCount = encoder.getCount();
  drawMenuScreen(menuIndex, true);
}

void enterRuntimeScreen() {
  currentScreen = SCREEN_RUNTIME;
  runtimeStartMillis = millis();
  
  // Always start at 0 PSI
  targetPsi = 0.0f;
  setTargetPressureSafe(targetPsi);
  encoder.setCount(0);
  lastEncoderCount = encoder.getCount();
  
  // Start job timer
  startJobTimer();

  setMotorEnabledSafe(true);
  if (overTempActive) {
    setMotorEnabledSafe(false);
  }
  drawRuntimeStatic(displayUnits);
  float currentPsi = 0.0f;
  bool pressureValid = false;
  uint32_t idleSecondsRemaining = UINT32_MAX;
  portENTER_CRITICAL(&motorShared.mutex);
  currentPsi = motorShared.smoothedPsi;
  pressureValid = motorShared.pressureValid;
  idleSecondsRemaining = motorShared.idleSecondsRemaining;
  portEXIT_CRITICAL(&motorShared.mutex);
  drawRuntimeTarget(targetPsi, currentPsi, displayUnits, pressureValid, true);
  drawRuntimeJobTime(0, true);
  drawRuntimeTemperature(currentTemperatureC, displayUnits, true);
#if DEBUG_DISPLAY_SENSOR_PRESSURE
  float rawPsi = 0.0f;
  int32_t rawValue = 0;
  bool rawValid = false;
  getRawPressureSafe(&rawPsi, &rawValue, &rawValid);
  drawRuntimeSensorPressureDebug(rawPsi, rawValue, rawValid, true);
#endif
}

void enterSettingsScreen() {
  currentScreen = SCREEN_SETTINGS;
  settingsEditing = false;
  settingsDirty = false;
  settingsIndex = 0;
  settingsScrollAccumulator = 0;

  // Load current power pause settings
  powerPauseSeconds = getIdleEntrySecondsSafe();
  powerPauseSeconds = constrain(powerPauseSeconds, POWER_PAUSE_SEC_MIN, POWER_PAUSE_SEC_MAX);

  encoder.setCount(settingsIndex);
  lastEncoderCount = encoder.getCount();

  drawPowerPauseSettingsScreen(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, settingsEditing, true);
}

void enterSupportScreen() {
  currentScreen = SCREEN_SUPPORT;
  encoder.setCount(0);
  lastEncoderCount = encoder.getCount();
  drawSupportScreen();
}

void enterAboutScreen() {
  currentScreen = SCREEN_ABOUT;
  encoder.setCount(0);
  lastEncoderCount = encoder.getCount();
  drawAboutScreen(totalRuntimeTenths, FIRMWARE_VERSION);
}

// Job timer functions
void startJobTimer() {
  jobStartMillis = millis();
  jobElapsedMillis = 0;
  jobPauseStartMillis = 0;
  jobTimerActive = true;
}

void pauseJobTimer() {
  if (jobTimerActive && jobPauseStartMillis == 0) {
    jobPauseStartMillis = millis();
  }
}

void resumeJobTimer() {
  if (jobPauseStartMillis > 0) {
    // Add the paused time to elapsed
    uint32_t pauseDuration = millis() - jobPauseStartMillis;
    jobElapsedMillis += pauseDuration;
    jobPauseStartMillis = 0;
  }
}

uint32_t getJobTimeSeconds() {
  if (!jobTimerActive) {
    return 0;
  }
  
  uint32_t currentMillis = millis();
  uint32_t totalElapsed;
  
  if (jobPauseStartMillis > 0) {
    // Currently paused - don't count time since pause started
    totalElapsed = jobPauseStartMillis - jobStartMillis - jobElapsedMillis;
  } else {
    // Running - count all time except previous pauses
    totalElapsed = currentMillis - jobStartMillis - jobElapsedMillis;
  }
  
  return totalElapsed / 1000;
}

void syncPidGainsFromSettings(bool saveToNvs) {
  PidController* pid = getPressurePid();
  pidSetGains(pid, settingsKp, settingsKi, settingsKd);
  setPidGainsSafe(settingsKp, settingsKi, settingsKd);
  requestPidReset();

  setIdleEntryDeviationSafe(settingsIdleDev);

  if (saveToNvs) {
    pidSaveGains(pid);
    saveSettings();
    settingsDirty = false;
  }
}

void syncPowerPauseSettings(bool saveToNvs) {
  powerPauseSeconds = constrain(powerPauseSeconds, POWER_PAUSE_SEC_MIN, POWER_PAUSE_SEC_MAX);

  setIdleEntrySecondsSafe(powerPauseSeconds);

  if (saveToNvs) {
    saveSettings();
    powerPauseDirty = false;
  }
}


void loop() {
  // Timing variables
  static unsigned long lastDisplayUpdate = 0;
  static unsigned long lastPauseUpdate = 0;
  static unsigned long lastHourMeterUpdate = 0;
  static unsigned long lastTempRead = 0;
  static unsigned long lastBeeperToggle = 0;
  static bool beeperOutput = false;
  static float smoothedPressure = 0.0f;
  static bool displayValid = false;
  static uint32_t idleSecondsRemaining = UINT32_MAX;
  static IdleState idleState = IDLE_STATE_OFF;
  static IdleState lastOverlayState = IDLE_STATE_OFF;
#if DEBUG_DISPLAY_SENSOR_PRESSURE
  static float rawSensorPressure = 0.0f;
  static int32_t rawSensorValue = 0;
  static bool rawSensorValid = false;
#endif

  // Read shared data from motor control task (thread-safe)
  {
    portENTER_CRITICAL(&motorShared.mutex);
    smoothedPressure = motorShared.smoothedPsi;
    displayValid = motorShared.pressureValid;
    idleSecondsRemaining = motorShared.idleSecondsRemaining;
    idleState = (IdleState)motorShared.idleState;
    uint32_t loopUs = motorShared.loopTimeUs;
    portEXIT_CRITICAL(&motorShared.mutex);
    pidLoopIntervalMs = loopUs / 1000;
  }

#if DEBUG_DISPLAY_SENSOR_PRESSURE
  getRawPressureSafe(&rawSensorPressure, &rawSensorValue, &rawSensorValid);
#endif

  // Encoder handling based on screen
  int64_t encoderCount = encoder.getCount();
  if (encoderCount != lastEncoderCount) {
    int64_t delta = encoderCount - lastEncoderCount;
    lastEncoderCount = encoderCount;

    if (currentScreen == SCREEN_MENU) {
      menuScrollAccumulator += (int32_t)delta;
      int32_t menuSteps = 0;
      while (menuScrollAccumulator >= 2) {
        menuSteps++;
        menuScrollAccumulator -= 2;
      }
      while (menuScrollAccumulator <= -2) {
        menuSteps--;
        menuScrollAccumulator += 2;
      }

      if (menuSteps != 0) {
        int32_t newIndex = (int32_t)menuIndex + menuSteps;
        if (newIndex < 0) newIndex = 0;
        if (newIndex >= MENU_OPTION_COUNT) newIndex = MENU_OPTION_COUNT - 1;
        menuIndex = (uint8_t)newIndex;
        encoder.setCount(menuIndex);
        lastEncoderCount = encoder.getCount();
        drawMenuScreen(menuIndex, false);
      }
    } else if (currentScreen == SCREEN_RUNTIME) {
      if (idleState != IDLE_STATE_OFF) {
        requestIdleExitSafe();
        resumeJobTimer();  // Resume job timer when exiting power pause
      }
      float newTarget = targetPsi + (delta * TARGET_PSI_STEP);
      if (newTarget < TARGET_PSI_MIN) newTarget = TARGET_PSI_MIN;
      if (newTarget > TARGET_PSI_MAX) newTarget = TARGET_PSI_MAX;
      targetPsi = newTarget;
      setTargetPressureSafe(targetPsi);
      drawRuntimeTarget(targetPsi, smoothedPressure, displayUnits, displayValid);
    } else if (currentScreen == SCREEN_SETTINGS) {
      if (settingsEditing) {
        if (settingsIndex == 0) {
          int32_t change = (int32_t)delta * POWER_PAUSE_SEC_STEP;
          int32_t nextValue = (int32_t)powerPauseSeconds + change;
          powerPauseSeconds = (uint16_t)constrain(nextValue, POWER_PAUSE_SEC_MIN, POWER_PAUSE_SEC_MAX);
          // Warning seconds is now a fixed constant
          settingsDirty = true;
          syncPowerPauseSettings(false);
          drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, true, true);
        } else if (settingsIndex == 1) {
          // Require 2 ticks to toggle to avoid accidental changes
          static int32_t beeperAccumulator = 0;
          beeperAccumulator += (int32_t)delta;
          if (beeperAccumulator >= 2 || beeperAccumulator <= -2) {
            powerPauseBeeperEnabled = !powerPauseBeeperEnabled;
            settingsDirty = true;
            drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, true, true);
            beeperAccumulator = 0;  // Reset accumulator
          }
        } else if (settingsIndex == 2) {
          // Units toggle - require 2 ticks to toggle to avoid accidental changes
          static int32_t unitsAccumulator = 0;
          unitsAccumulator += (int32_t)delta;
          if (unitsAccumulator >= 2 || unitsAccumulator <= -2) {
            displayUnits = (displayUnits == UNITS_IMPERIAL) ? UNITS_METRIC : UNITS_IMPERIAL;
            settingsDirty = true;
            drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, true, true);
            unitsAccumulator = 0;  // Reset accumulator
          }
        }
      } else {
        settingsScrollAccumulator += (int32_t)delta;
        int32_t settingsSteps = 0;
        while (settingsScrollAccumulator >= 2) {
          settingsSteps++;
          settingsScrollAccumulator -= 2;
        }
        while (settingsScrollAccumulator <= -2) {
          settingsSteps--;
          settingsScrollAccumulator += 2;
        }

        if (settingsSteps != 0) {
          uint8_t previousIndex = settingsIndex;
          int32_t newIndex = (int32_t)settingsIndex + settingsSteps;
          if (newIndex < 0) newIndex = 0;
          if (newIndex >= SETTINGS_OPTION_COUNT) newIndex = SETTINGS_OPTION_COUNT - 1;
          settingsIndex = (uint8_t)newIndex;
          encoder.setCount(settingsIndex);
          lastEncoderCount = encoder.getCount();
          if (previousIndex != settingsIndex) {
            drawPowerPauseSettingsRow(previousIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, false, false);
            drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, true, false);
          }
        }
      }
    } else if (currentScreen == SCREEN_POWERPAUSE_SETTINGS) {
      if (powerPauseEditing) {
        if (powerPauseIndex == 0) {
          int32_t change = (int32_t)delta * POWER_PAUSE_SEC_STEP;
          int32_t nextValue = (int32_t)powerPauseSeconds + change;
          powerPauseSeconds = (uint16_t)constrain(nextValue, POWER_PAUSE_SEC_MIN, POWER_PAUSE_SEC_MAX);
          // Warning seconds is now a fixed constant
          powerPauseDirty = true;
          syncPowerPauseSettings(false);
          drawPowerPauseSettingsRow(powerPauseIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, true, true);
        } else if (powerPauseIndex == 1) {
          if (delta != 0) {
            powerPauseBeeperEnabled = !powerPauseBeeperEnabled;
            powerPauseDirty = true;
            drawPowerPauseSettingsRow(powerPauseIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, true, true);
          }
        } else if (powerPauseIndex == 2) {
          // Units selection
          if (delta != 0) {
            displayUnits = (displayUnits == UNITS_IMPERIAL) ? UNITS_METRIC : UNITS_IMPERIAL;
            powerPauseDirty = true;
            drawPowerPauseSettingsRow(powerPauseIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, true, true);
          }
        }
      } else {
        powerPauseScrollAccumulator += (int32_t)delta;
        int32_t powerPauseSteps = 0;
        while (powerPauseScrollAccumulator >= 2) {
          powerPauseSteps++;
          powerPauseScrollAccumulator -= 2;
        }
        while (powerPauseScrollAccumulator <= -2) {
          powerPauseSteps--;
          powerPauseScrollAccumulator += 2;
        }

        if (powerPauseSteps != 0) {
          uint8_t previousIndex = powerPauseIndex;
          int32_t newIndex = (int32_t)powerPauseIndex + powerPauseSteps;
          if (newIndex < 0) newIndex = 0;
          if (newIndex > 3) newIndex = 3;
          powerPauseIndex = (uint8_t)newIndex;
          encoder.setCount(powerPauseIndex);
          lastEncoderCount = encoder.getCount();
          if (previousIndex != powerPauseIndex) {
            drawPowerPauseSettingsRow(previousIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, false, false);
            drawPowerPauseSettingsRow(powerPauseIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, true, false);
          }
        }
      }
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
          enterSettingsScreen();
        } else if (menuIndex == 2) {
          enterSupportScreen();
        } else if (menuIndex == 3) {
          enterAboutScreen();
        }
      } else if (currentScreen == SCREEN_RUNTIME) {
        // Pause job timer and go to menu
        pauseJobTimer();
        setMotorEnabledSafe(false);
        enterMenuScreen();
      } else if (currentScreen == SCREEN_SETTINGS) {
        if (settingsEditing) {
          settingsEditing = false;
          drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, true, false);
          drawPowerPauseSettingsFooter("Press to edit / select", COLOR_SUCCESS);
        } else {
          if (settingsIndex <= 2) {
            settingsEditing = true;
            drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, true, true);
            drawPowerPauseSettingsFooter("Rotate to adjust, press to exit", COLOR_MENU_EDIT);
          } else if (settingsIndex == 3) {
            // Exit Settings - save if dirty
            if (settingsDirty) {
              syncPowerPauseSettings(true);
            }
            enterMenuScreen();
          }
        }
      } else if (currentScreen == SCREEN_SUPPORT || currentScreen == SCREEN_ABOUT) {
        enterMenuScreen();
      } else if (currentScreen == SCREEN_POWERPAUSE_SETTINGS) {
        if (powerPauseEditing) {
          powerPauseEditing = false;
          drawPowerPauseSettingsRow(powerPauseIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, true, false);
          drawPowerPauseSettingsFooter("Press to edit / select", TFT_GREEN);
        } else {
          if (powerPauseIndex <= 2) {
            powerPauseEditing = true;
            drawPowerPauseSettingsRow(powerPauseIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, displayUnits, true, true);
            drawPowerPauseSettingsFooter("Rotate to adjust, press to exit", TFT_YELLOW);
          } else if (powerPauseIndex == 3) {
            // Exit - save if dirty
            if (powerPauseDirty) {
              syncPowerPauseSettings(true);
            }
            enterMenuScreen();
          }
        }
      }
    }
  }

  unsigned long now = millis();

#if DEBUG_SERIAL_OUTPUT
  // Debug serial output - runs in display loop using safe functions
  static unsigned long lastSerialDebug = 0;
  if (now - lastSerialDebug >= SERIAL_DEBUG_INTERVAL_MS) {
    lastSerialDebug = now;
    
    // Read shared data safely
    float currentPsi;
    uint32_t powerPauseTicks;
    {
      portENTER_CRITICAL(&motorShared.mutex);
      currentPsi = motorShared.smoothedPsi;
      powerPauseTicks = motorShared.idleSecondsRemaining;
      portEXIT_CRITICAL(&motorShared.mutex);
    }
    
    // Print to serial
    Serial.print("[DEBUG] PSI: ");
    Serial.print(currentPsi, 2);
    Serial.print(" | Power Pause Ticks: ");
    if (powerPauseTicks == UINT32_MAX) {
      Serial.println("N/A (not active)");
    } else {
      Serial.print(powerPauseTicks);
      Serial.println(" sec");
    }
  }
#endif

  if (now - lastTempRead >= TEMP_READ_INTERVAL_MS) {
    //toggleBeeper();  // Toggle beeper state for testing
    lastTempRead = now;
    uint16_t tempAdc = tempSensorReadAdc();
    currentTemperatureC = -1.75f * (float)tempAdc  + 207.0f; 


    if (currentTemperatureC > TEMP_OVERHEAT_SETPOINT) {
      if (!overTempActive) {
        overTempActive = true;
        setMotorEnabledSafe(false);
        requestPidReset();
        Serial.println("Overtemperature detected: motor disabled");
      }
    } else if (overTempActive && currentTemperatureC < TEMP_OVERHEAT_CLEAR) {
      overTempActive = false;
      if (currentScreen == SCREEN_RUNTIME) {
        setMotorEnabledSafe(true);
      }
      Serial.println("Temperature back to normal: motor enabled");
    }
  }

  if (currentScreen == SCREEN_RUNTIME) {
    // Handle job timer pause/resume based on idle state
    static IdleState lastIdleState = IDLE_STATE_OFF;
    static uint32_t idleHoldEntryTime = 0;  // Track when idle hold state started
    
    if (idleState != lastIdleState) {
      if (idleState != IDLE_STATE_OFF && lastIdleState == IDLE_STATE_OFF) {
        pauseJobTimer();
      } else if (idleState == IDLE_STATE_OFF && lastIdleState != IDLE_STATE_OFF) {
        resumeJobTimer();
      }
      
      // Track when we enter idle hold state
      if (idleState == IDLE_STATE_HOLD && lastIdleState != IDLE_STATE_HOLD) {
        idleHoldEntryTime = millis();
      }
      
      lastIdleState = idleState;
    }
    
    // Handle idle state changes and overlays
    if (idleState != lastOverlayState) {
      if (idleState == IDLE_STATE_OFF) {
        drawRuntimeStatic(displayUnits);
        drawRuntimeTarget(targetPsi, smoothedPressure, displayUnits, displayValid, true);
        drawRuntimeJobTime(getJobTimeSeconds(), true);
        drawRuntimeTemperature(currentTemperatureC, displayUnits, true);
#if DEBUG_DISPLAY_SENSOR_PRESSURE
        drawRuntimeSensorPressureDebug(rawSensorPressure, rawSensorValue, rawSensorValid, true);
#endif
      }
      lastOverlayState = idleState;
    }
    
    // Show appropriate overlay (update every cycle when active)
    if (overTempActive) {
      drawRuntimeOverTempOverlay(currentTemperatureC, false);
    } else if (idleState != IDLE_STATE_OFF) {
      // Calculate actual timeout remaining
      uint32_t timeoutRemaining = UINT32_MAX;
      
      // When in idle hold state, calculate time remaining until menu timeout
      if (idleState == IDLE_STATE_HOLD) {
        uint32_t idleDuration = (millis() - idleHoldEntryTime) / 1000;
        uint32_t maxIdleTime = 900;  // 15 minutes default
        
        if (idleDuration < maxIdleTime) {
          timeoutRemaining = maxIdleTime - idleDuration;
        } else {
          timeoutRemaining = 0;
        }
      }
      
      drawRuntimePowerPauseOverlay(idleState, timeoutRemaining, false);
    }
    
    // Check if power pause has timed out (10 minutes default)
    if (idleState == IDLE_STATE_HOLD) {
      uint32_t idleDuration = (millis() - idleHoldEntryTime) / 1000;
      uint32_t maxIdleTime = 600;  // 10 minutes default
      
      if (idleDuration >= maxIdleTime) {
        // Timeout - exit to menu
        pauseJobTimer();
        setMotorEnabledSafe(false);
        requestPidReset();
        enterMenuScreen();
        Serial.println("Power pause timeout - returned to menu");
        return;  // Early return to prevent further processing
      }
    }

    if (now - lastDisplayUpdate >= DISPLAY_PRESSURE_INTERVAL_MS) {
      lastDisplayUpdate = now;
      if (idleState == IDLE_STATE_OFF && !overTempActive) {
        drawRuntimeTarget(targetPsi, smoothedPressure, displayUnits, displayValid);
        drawRuntimeJobTime(getJobTimeSeconds());
        drawRuntimeTemperature(currentTemperatureC, displayUnits);
      }
#if DEBUG_DISPLAY_SENSOR_PRESSURE
      if (idleState == IDLE_STATE_OFF && !overTempActive) {
        drawRuntimeSensorPressureDebug(rawSensorPressure, rawSensorValue, rawSensorValid);
      }
#endif
    }

    bool warningActive = false;
    uint32_t warningSeconds = powerPauseWarnSeconds;
    if (powerPauseBeeperEnabled && idleState == IDLE_STATE_OFF && idleSecondsRemaining != UINT32_MAX) {
      if (warningSeconds > 0 && idleSecondsRemaining > 0 && idleSecondsRemaining <= warningSeconds) {
        warningActive = true;
      }
    }

    if (warningActive) {
      uint32_t maxIntervalMs = 1200;
      uint32_t minIntervalMs = 200;
      uint32_t span = (warningSeconds > 1) ? (warningSeconds - 1) : 1;
      uint32_t remaining = (idleSecondsRemaining > 1) ? (idleSecondsRemaining - 1) : 0;
      if (remaining > span) {
        remaining = span;
      }
      uint32_t intervalMs = minIntervalMs + ((maxIntervalMs - minIntervalMs) * remaining) / span;

      if (now - lastBeeperToggle >= intervalMs) {
        lastBeeperToggle = now;
        beeperOutput = !beeperOutput;
        setBeeper(beeperOutput);
      }
    } else {
      if (beeperOutput) {
        beeperOutput = false;
        setBeeper(false);
      }
      lastBeeperToggle = now;
    }
  } else {
    if (beeperOutput) {
      beeperOutput = false;
      setBeeper(false);
    }
  }

#ifdef TRIAC_DEBUG_SERIAL
  motorControlDebugReport();
#endif

  // Update hour meter every 6 minutes (0.1 hours)
  if (millis() - lastHourMeterUpdate >= 360000) {
    lastHourMeterUpdate = millis();
    totalRuntimeTenths++;
    saveHourMeter();
  }
}