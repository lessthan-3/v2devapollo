#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <ESP32Encoder.h>
#include <Preferences.h>
#include "config.h"
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

// Screen state enumeration
typedef enum {
  SCREEN_STARTUP = 0,
  SCREEN_MENU,
  SCREEN_RUNTIME,
  SCREEN_SETTINGS,
  SCREEN_POWERPAUSE_SETTINGS
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
int32_t menuScrollAccumulator = 0;
int32_t settingsScrollAccumulator = 0;
int32_t powerPauseScrollAccumulator = 0;
uint32_t runtimeStartMillis = 0;
uint32_t runtimePauseDeadlineSec = 0;

// Settings screen state
uint8_t settingsIndex = 0;
bool settingsEditing = false;
float settingsIdleDev = IDLE_ENTRY_DEVIATION_PSI;
float settingsStartPsi = TARGET_PSI_DEFAULT;
bool settingsDirty = false;

// Power pause settings state
uint8_t powerPauseIndex = 0;
bool powerPauseEditing = false;
uint16_t powerPauseSeconds = IDLE_ENTRY_SECONDS;
bool powerPauseBeeperEnabled = true;
uint16_t powerPauseWarnSeconds = 5;
bool powerPauseDirty = false;

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
void enterPowerPauseSettingsScreen();
void loadSettings();
void saveSettings();
void syncPowerPauseSettings(bool saveToNvs);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Apollo Sprayers HVLP - ESP32-S3");
  Serial.println("Initializing...");

  beeperInit();

  // Initialize backlight pin
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, HIGH);  // Turn on backlight

  // Initialize the display
  tft.init();
  tft.setRotation(1);  // Landscape mode (480x320)
  
  // Display startup screen
  drawStartupScreen();
  
  Serial.println("Display initialized successfully!");
  
  // Initialize rotary encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachHalfQuad(PIN_ENCODER_DT, PIN_ENCODER_CLK);
  encoder.setCount((int64_t)(TARGET_PSI_DEFAULT / TARGET_PSI_STEP));  // Start at default
  lastEncoderCount = encoder.getCount();

  // Initialize temperature sensor
  pinMode(TEMP_SENSOR_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(TEMP_SENSOR_PIN, ADC_11db);
  tempSensorInit();

  // Initialize encoder button
  pinMode(PIN_ENCODER_BTN, INPUT_PULLUP);

  Serial.println("Encoder initialized");

  // Load hour meter from flash
  loadHourMeter();

  // Load settings from flash
  loadSettings();

  targetPsi = settingsStartPsi;
  setTargetPressureSafe(targetPsi);
  encoder.setCount((int64_t)lroundf(targetPsi / TARGET_PSI_STEP));
  lastEncoderCount = encoder.getCount();

  // Initialize motor control (zero crossing interrupt)
  delay(100);
  if (motorControlInit()) {
    Serial.println("Motor control initialized");

    // Initialize the pressure PID controller (loads saved gains from NVS)
    pressurePidInit();

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
    if (acFreq == AC_FREQ_UNKNOWN) {
      Serial.println("AC frequency: Unknown");
    }

    // Initialize dual-core motor control (runs PID on Core 0)
    if (dualCoreMotorInit()) {
      Serial.println("Dual-core motor control initialized");
      setTargetPressureSafe(TARGET_PSI_DEFAULT);
    } else {
      Serial.println("Dual-core motor control FAILED");
    }
  } else {
    Serial.println("Motor control FAILED");
  }

  // Record session start time
  sessionStartMillis = millis();

  delay(3500);
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

void loadSettings() {
  if (preferences.begin("apollo", true)) {
    settingsIdleDev = preferences.getFloat("idleDev", IDLE_ENTRY_DEVIATION_PSI);
    settingsStartPsi = preferences.getFloat("startPsi", TARGET_PSI_DEFAULT);
    powerPauseSeconds = preferences.getUShort("idleSec", IDLE_ENTRY_SECONDS);
    powerPauseBeeperEnabled = preferences.getBool("warnBeep", true);
    powerPauseWarnSeconds = preferences.getUShort("warnSec", 5);
    preferences.end();
  } else {
    preferences.end();
    settingsIdleDev = IDLE_ENTRY_DEVIATION_PSI;
    settingsStartPsi = TARGET_PSI_DEFAULT;
    powerPauseSeconds = IDLE_ENTRY_SECONDS;
    powerPauseBeeperEnabled = true;
    powerPauseWarnSeconds = 5;
  }
  settingsIdleDev = constrain(settingsIdleDev, IDLE_DEV_MIN, IDLE_DEV_MAX);
  settingsStartPsi = constrain(settingsStartPsi, TARGET_PSI_MIN, TARGET_PSI_MAX);
  powerPauseSeconds = constrain(powerPauseSeconds, POWER_PAUSE_SEC_MIN, POWER_PAUSE_SEC_MAX);
  powerPauseWarnSeconds = constrain(powerPauseWarnSeconds, POWER_PAUSE_WARN_SEC_MIN, POWER_PAUSE_WARN_SEC_MAX);
  if (powerPauseWarnSeconds > powerPauseSeconds) {
    powerPauseWarnSeconds = powerPauseSeconds;
  }
  setIdleEntryDeviationSafe(settingsIdleDev);
  setIdleEntrySecondsSafe(powerPauseSeconds);
}

void saveSettings() {
  if (preferences.begin("apollo", false)) {
    preferences.putFloat("idleDev", settingsIdleDev);
    preferences.putFloat("startPsi", settingsStartPsi);
    preferences.putUShort("idleSec", powerPauseSeconds);
    preferences.putBool("warnBeep", powerPauseBeeperEnabled);
    preferences.putUShort("warnSec", powerPauseWarnSeconds);
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

  setMotorEnabledSafe(true);
  if (overTempActive) {
    setMotorEnabledSafe(false);
  }
  drawRuntimeStatic();
  float currentPsi = 0.0f;
  bool pressureValid = false;
  uint32_t idleSecondsRemaining = UINT32_MAX;
  portENTER_CRITICAL(&motorShared.mutex);
  currentPsi = motorShared.smoothedPsi;
  pressureValid = motorShared.pressureValid;
  idleSecondsRemaining = motorShared.idleSecondsRemaining;
  portEXIT_CRITICAL(&motorShared.mutex);
  drawRuntimeTarget(targetPsi, currentPsi, pressureValid, true);
  drawRuntimeTemperature(currentTemperatureC, true);
  drawRuntimePauseCountdown(idleSecondsRemaining, true);
  drawRuntimeFooter();
  drawRuntimeMotorPower(getMotorSpeedSafe(), true);
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

  settingsIdleDev = getIdleEntryDeviationSafe();

  encoder.setCount(settingsIndex);
  lastEncoderCount = encoder.getCount();

  drawSettingsScreen(settingsIndex, settingsIdleDev, settingsStartPsi, settingsEditing, true);
}

void enterPowerPauseSettingsScreen() {
  currentScreen = SCREEN_POWERPAUSE_SETTINGS;
  powerPauseEditing = false;
  powerPauseDirty = false;
  powerPauseIndex = 0;
  powerPauseScrollAccumulator = 0;

  powerPauseSeconds = getIdleEntrySecondsSafe();
  powerPauseSeconds = constrain(powerPauseSeconds, POWER_PAUSE_SEC_MIN, POWER_PAUSE_SEC_MAX);
  powerPauseWarnSeconds = constrain(powerPauseWarnSeconds, POWER_PAUSE_WARN_SEC_MIN, POWER_PAUSE_WARN_SEC_MAX);
  if (powerPauseWarnSeconds > powerPauseSeconds) {
    powerPauseWarnSeconds = powerPauseSeconds;
  }

  encoder.setCount(powerPauseIndex);
  lastEncoderCount = encoder.getCount();

  drawPowerPauseSettingsScreen(powerPauseIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, powerPauseEditing, true);
}

void syncPowerPauseSettings(bool saveToNvs) {
  powerPauseSeconds = constrain(powerPauseSeconds, POWER_PAUSE_SEC_MIN, POWER_PAUSE_SEC_MAX);
  powerPauseWarnSeconds = constrain(powerPauseWarnSeconds, POWER_PAUSE_WARN_SEC_MIN, POWER_PAUSE_WARN_SEC_MAX);
  if (powerPauseWarnSeconds > powerPauseSeconds) {
    powerPauseWarnSeconds = powerPauseSeconds;
  }

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
      }
      float newTarget = targetPsi + (delta * TARGET_PSI_STEP);
      if (newTarget < TARGET_PSI_MIN) newTarget = TARGET_PSI_MIN;
      if (newTarget > TARGET_PSI_MAX) newTarget = TARGET_PSI_MAX;
      targetPsi = newTarget;
      setTargetPressureSafe(targetPsi);
      drawRuntimeTarget(targetPsi, smoothedPressure, displayValid);
    } else if (currentScreen == SCREEN_SETTINGS) {
      if (settingsEditing) {
        // Index 0 = Idle dev, Index 1 = Start psi
        float step = 0.0f;
        if (settingsIndex == 0) step = IDLE_DEV_STEP;
        if (settingsIndex == 1) step = TARGET_PSI_STEP;

        if (step > 0.0f) {
          float change = (float)delta * step;
          if (settingsIndex == 0) {
            settingsIdleDev = constrain(settingsIdleDev + change, IDLE_DEV_MIN, IDLE_DEV_MAX);
            setIdleEntryDeviationSafe(settingsIdleDev);
          } else if (settingsIndex == 1) {
            settingsStartPsi = constrain(settingsStartPsi + change, TARGET_PSI_MIN, TARGET_PSI_MAX);
          }
          settingsDirty = true;
          drawSettingsRow(settingsIndex, settingsIdleDev, settingsStartPsi, true, true);
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
            drawSettingsRow(previousIndex, settingsIdleDev, settingsStartPsi, false, false);
            drawSettingsRow(settingsIndex, settingsIdleDev, settingsStartPsi, true, false);
          }
        }
      }
    } else if (currentScreen == SCREEN_POWERPAUSE_SETTINGS) {
      if (powerPauseEditing) {
        if (powerPauseIndex == 0) {
          int32_t change = (int32_t)delta * POWER_PAUSE_SEC_STEP;
          int32_t nextValue = (int32_t)powerPauseSeconds + change;
          powerPauseSeconds = (uint16_t)constrain(nextValue, POWER_PAUSE_SEC_MIN, POWER_PAUSE_SEC_MAX);
          if (powerPauseWarnSeconds > powerPauseSeconds) {
            powerPauseWarnSeconds = powerPauseSeconds;
          }
          powerPauseDirty = true;
          syncPowerPauseSettings(false);
          drawPowerPauseSettingsRow(powerPauseIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, true, true);
        } else if (powerPauseIndex == 1) {
          if (delta != 0) {
            powerPauseBeeperEnabled = !powerPauseBeeperEnabled;
            powerPauseDirty = true;
            drawPowerPauseSettingsRow(powerPauseIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, true, true);
          }
        } else if (powerPauseIndex == 2) {
          int32_t change = (int32_t)delta * POWER_PAUSE_WARN_SEC_STEP;
          int32_t nextValue = (int32_t)powerPauseWarnSeconds + change;
          powerPauseWarnSeconds = (uint16_t)constrain(nextValue, POWER_PAUSE_WARN_SEC_MIN, POWER_PAUSE_WARN_SEC_MAX);
          if (powerPauseWarnSeconds > powerPauseSeconds) {
            powerPauseWarnSeconds = powerPauseSeconds;
          }
          powerPauseDirty = true;
          drawPowerPauseSettingsRow(powerPauseIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, true, true);
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
          if (newIndex > 4) newIndex = 4;
          powerPauseIndex = (uint8_t)newIndex;
          encoder.setCount(powerPauseIndex);
          lastEncoderCount = encoder.getCount();
          if (previousIndex != powerPauseIndex) {
            drawPowerPauseSettingsRow(previousIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, false, false);
            drawPowerPauseSettingsRow(powerPauseIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, true, false);
          }
        }
      }
    }
  }

  // Button handling (simple debounce)
  static bool lastButtonState = false;
  static unsigned long lastButtonChange = 0;
  bool buttonPressed = (digitalRead(PIN_ENCODER_BTN) == LOW);
  if (buttonPressed != lastButtonState && (millis() - lastButtonChange) > BUTTON_DEBOUNCE_MS) {
    lastButtonChange = millis();
    lastButtonState = buttonPressed;
    if (!buttonPressed) {
      if (currentScreen == SCREEN_MENU) {
        if (menuIndex == 0) {
          enterRuntimeScreen();
        } else if (menuIndex == 1) {
          enterSettingsScreen();
        } else if (menuIndex == 2) {
          enterPowerPauseSettingsScreen();
        } else if (menuIndex == 3) {
          drawMenuFooter("Firmware " FIRMWARE_VERSION, TFT_CYAN);
        }
      } else if (currentScreen == SCREEN_RUNTIME) {
        enterMenuScreen();
      } else if (currentScreen == SCREEN_SETTINGS) {
        if (settingsEditing) {
          settingsEditing = false;
          drawSettingsRow(settingsIndex, settingsIdleDev, settingsStartPsi, true, false);
          drawSettingsFooter("Press to edit / select", TFT_GREEN);
        } else {
          if (settingsIndex <= 1) {
            // Index 0 = Idle dev, Index 1 = Start psi — both editable
            settingsEditing = true;
            drawSettingsRow(settingsIndex, settingsIdleDev, settingsStartPsi, true, true);
            drawSettingsFooter("Rotate to adjust, press to exit", TFT_YELLOW);
          } else if (settingsIndex == 2) {
            // Save
            if (settingsDirty) {
              saveSettings();
              targetPsi = settingsStartPsi;
              setTargetPressureSafe(targetPsi);
              encoder.setCount((int64_t)lroundf(targetPsi / TARGET_PSI_STEP));
              lastEncoderCount = encoder.getCount();
              settingsDirty = false;
              drawSettingsFooter("Settings saved", TFT_GREEN);
            } else {
              drawSettingsFooter("No changes to save", TFT_ORANGE);
            }
          } else if (settingsIndex == 3) {
            // Back
            enterMenuScreen();
          }
        }
      } else if (currentScreen == SCREEN_POWERPAUSE_SETTINGS) {
        if (powerPauseEditing) {
          powerPauseEditing = false;
          drawPowerPauseSettingsRow(powerPauseIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, true, false);
          drawPowerPauseSettingsFooter("Press to edit / select", TFT_GREEN);
        } else {
          if (powerPauseIndex <= 2) {
            powerPauseEditing = true;
            drawPowerPauseSettingsRow(powerPauseIndex, powerPauseSeconds, powerPauseBeeperEnabled, powerPauseWarnSeconds, true, true);
            drawPowerPauseSettingsFooter("Rotate to adjust, press to exit", TFT_YELLOW);
          } else if (powerPauseIndex == 3) {
            if (powerPauseDirty) {
              syncPowerPauseSettings(true);
              drawPowerPauseSettingsFooter("Settings saved", TFT_GREEN);
            } else {
              drawPowerPauseSettingsFooter("No changes to save", TFT_ORANGE);
            }
          } else if (powerPauseIndex == 4) {
            enterMenuScreen();
          }
        }
      }
    }
  }

  unsigned long now = millis();

  if (now - lastTempRead >= TEMP_READ_INTERVAL_MS) {
    //toggleBeeper();  // Toggle beeper state for testing
    lastTempRead = now;
    uint16_t tempAdc = tempSensorReadAdc();
    currentTemperatureC = ((float)tempAdc - (float)TEMP_OFFSET) * (float)TEMP_MULT / (float)TEMP_DIVISOR;
    //Serial.printf("Temp ADC=%u, C=%.2f, pin=%u\n", tempAdc, currentTemperatureC, (unsigned)TEMP_SENSOR_PIN);

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
    if (idleState != lastOverlayState) {
      if (idleState == IDLE_STATE_OFF) {
        drawRuntimeStatic();
        drawRuntimeTarget(targetPsi, smoothedPressure, displayValid, true);
        drawRuntimeTemperature(currentTemperatureC, true);
        drawRuntimePauseCountdown(idleSecondsRemaining, true);
        drawRuntimeFooter();
        drawRuntimeMotorPower(getMotorSpeedSafe(), true);
#if DEBUG_DISPLAY_SENSOR_PRESSURE
        drawRuntimeSensorPressureDebug(rawSensorPressure, rawSensorValue, rawSensorValid, true);
#endif
      }
      drawRuntimePowerPauseOverlay(idleState, true);
      lastOverlayState = idleState;
    }

    if (now - lastDisplayUpdate >= DISPLAY_PRESSURE_INTERVAL_MS) {
      lastDisplayUpdate = now;
      if (idleState == IDLE_STATE_OFF) {
        drawRuntimeTarget(targetPsi, smoothedPressure, displayValid);
        drawRuntimeTemperature(currentTemperatureC);
        drawRuntimeMotorPower(getMotorSpeedSafe());
      }
#if DEBUG_DISPLAY_SENSOR_PRESSURE
      if (idleState == IDLE_STATE_OFF) {
        drawRuntimeSensorPressureDebug(rawSensorPressure, rawSensorValue, rawSensorValid);
      }
#endif
    }

    if (now - lastPauseUpdate >= 1000) {
      lastPauseUpdate = now;
      if (idleState == IDLE_STATE_OFF) {
        drawRuntimePauseCountdown(idleSecondsRemaining);
      }
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
  if (millis() - lastHourMeterUpdate >= HOUR_METER_UPDATE_INTERVAL_MS) {
    lastHourMeterUpdate = millis();
    totalRuntimeTenths++;
    saveHourMeter();
  }
}