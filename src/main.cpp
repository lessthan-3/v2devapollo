#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <ESP32Encoder.h>
#include "config.h"
#include "pressure_sensor.h"
#include "motor_control.h"
#include "dual_core_motor.h"
#include "display_ui.h"
#include "temp_sensor.h"
#include "beeper.h"
#include "storage.h"
#include "job_timer.h"

// ---------------------------------------------------------------------------
// Hardware instances
// ---------------------------------------------------------------------------
TFT_eSPI tft = TFT_eSPI();
ESP32Encoder encoder;

// ---------------------------------------------------------------------------
// Screen state
// ---------------------------------------------------------------------------
typedef enum {
    SCREEN_STARTUP = 0,
    SCREEN_MENU,
    SCREEN_RUNTIME,
    SCREEN_SETTINGS,
    SCREEN_TIMERS,
    SCREEN_SUPPORT,
    SCREEN_SUPPORT_FAQ,
    SCREEN_SUPPORT_TECH,
    SCREEN_SUPPORT_CONTACT,
    SCREEN_ABOUT
} ScreenState;

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------

// Persistent storage variables (also declared extern in storage.h)
uint32_t    totalRuntimeTenths      = 0;
uint32_t    totalSystemTimeTenths   = 0;
float       settingsIdleDev         = IDLE_ENTRY_DEVIATION_PSI;
float       settingsStartPsi        = TARGET_PSI_DEFAULT;
uint16_t    powerPauseSeconds       = IDLE_ENTRY_SECONDS;
bool        powerPauseBeeperEnabled = true;
DisplayUnits displayUnits           = UNITS_IMPERIAL;
bool        lightThemeEnabled       = false;

// Runtime target pressure
float       targetPsi               = TARGET_PSI_DEFAULT;

// Encoder tracking
int64_t     lastEncoderCount        = 0;

// Temperature / overtemp
float       currentTemperatureC          = -999.0f;
bool        overTempActive               = false;    // true when either warning or shutdown is active
bool        overTempWarning              = false;    // true when >= TEMP_WARN_SETPOINT (motor keeps running)
bool        overTempShutdown             = false;    // true when >= TEMP_SHUTDOWN_SETPOINT (restart required)
unsigned long overTempWarnBuzzStart      = 0;        // millis() when warning buzz started (0 = not active)

// Screen / menu navigation
ScreenState currentScreen           = SCREEN_STARTUP;
uint8_t     menuIndex               = 0;
int32_t     menuScrollAccumulator   = 0;

// Settings screen state (power pause settings)
uint8_t     settingsIndex           = 0;
bool        settingsEditing         = false;
bool        settingsDirty           = false;
int32_t     settingsScrollAccumulator = 0;
const uint16_t powerPauseWarnSeconds = POWER_PAUSE_WARN_SEC;

// About screen
uint8_t     aboutIndex              = 0;
int32_t     aboutScrollAccumulator  = 0;

// Timers screen
uint8_t     timersIndex             = 0;
int32_t     timersScrollAccumulator = 0;

// Support submenu
uint8_t     supportMenuIndex        = 0;
int32_t     supportScrollAccumulator = 0;

// Hour meter save tracking
unsigned long lastHourMeterUpdate   = 0;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void enterMenuScreen(void);
void enterRuntimeScreen(void);
void enterSettingsScreen(void);
void enterSupportScreen(void);
void enterSupportFaqScreen(void);
void enterSupportTechScreen(void);
void enterSupportContactScreen(void);
void enterTimersScreen(void);
void enterAboutScreen(void);
void syncPowerPauseSettings(bool saveToNvs);

// ---------------------------------------------------------------------------
// Screen transition helpers
// ---------------------------------------------------------------------------

void enterMenuScreen(void) {
    currentScreen = SCREEN_MENU;
    setMotorEnabledSafe(false);
    menuIndex = 0;
    menuScrollAccumulator = 0;
    encoder.setCount(menuIndex);
    lastEncoderCount = encoder.getCount();
    drawMenuScreen(menuIndex, true);
}

void enterRuntimeScreen(void) {
    currentScreen = SCREEN_RUNTIME;

    // Restore the previously set target pressure so returning from the menu
    // does not reset the setpoint back to zero.
    setTargetPressureSafe(targetPsi);
    encoder.setCount((int64_t)lroundf(targetPsi / TARGET_PSI_STEP));
    lastEncoderCount = encoder.getCount();

    // Seed the display timer from persisted flash value so it starts from
    // the saved total and counts up smoothly from there.
    startJobTimer(totalJobTimeTenths * 360UL);
    // If the target is at minimum (motor off), start the timer in a paused state
    if (targetPsi <= TARGET_PSI_MIN) {
        pauseJobTimer();
    }
    // Do not enable motor if any temp condition is active
    // (warning allows motor to run but shutdown does not)
    if (!overTempShutdown) {
        setMotorEnabledSafe(true);
    } else {
        setMotorEnabledSafe(false);
    }

    drawRuntimeStatic(displayUnits);

    float currentPsi = 0.0f;
    bool pressureValid = false;
    portENTER_CRITICAL(&motorShared.mutex);
    currentPsi    = motorShared.smoothedPsi;
    pressureValid = motorShared.pressureValid;
    portEXIT_CRITICAL(&motorShared.mutex);

    drawRuntimeTarget(targetPsi, currentPsi, displayUnits, pressureValid, true);
    drawRuntimeMotorPower(0, true);
    drawRuntimeJobTime(getJobTimeSeconds(), true);
    drawRuntimeTemperature(currentTemperatureC, displayUnits, true);
}

void enterSettingsScreen(void) {
    currentScreen = SCREEN_SETTINGS;
    settingsEditing = false;
    settingsDirty   = false;
    settingsIndex   = 0;
    settingsScrollAccumulator = 0;

    powerPauseSeconds = getIdleEntrySecondsSafe();
    powerPauseSeconds = constrain(powerPauseSeconds, POWER_PAUSE_SEC_MIN, POWER_PAUSE_SEC_MAX);

    encoder.setCount(settingsIndex);
    lastEncoderCount = encoder.getCount();
    drawPowerPauseSettingsScreen(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled,
                                 powerPauseWarnSeconds, displayUnits, settingsEditing, true);
}

void enterSupportScreen(void) {
    currentScreen = SCREEN_SUPPORT;
    supportMenuIndex = 0;
    supportScrollAccumulator = 0;
    encoder.setCount(0);
    lastEncoderCount = encoder.getCount();
    drawSupportMenuScreen(supportMenuIndex);
}

void enterSupportFaqScreen(void) {
    currentScreen = SCREEN_SUPPORT_FAQ;
    encoder.setCount(0);
    lastEncoderCount = encoder.getCount();
    drawSupportFaqScreen();
}

void enterSupportTechScreen(void) {
    currentScreen = SCREEN_SUPPORT_TECH;
    encoder.setCount(0);
    lastEncoderCount = encoder.getCount();
    drawSupportTechScreen();
}

void enterSupportContactScreen(void) {
    currentScreen = SCREEN_SUPPORT_CONTACT;
    encoder.setCount(0);
    lastEncoderCount = encoder.getCount();
    drawSupportContactScreen();
}

void enterTimersScreen(void) {
    currentScreen = SCREEN_TIMERS;
    timersIndex = 0;
    timersScrollAccumulator = 0;
    encoder.setCount(0);
    lastEncoderCount = encoder.getCount();
    drawTimersScreen(totalRuntimeTenths, totalJobTimeTenths, timersIndex);
}

void enterAboutScreen(void) {
    currentScreen = SCREEN_ABOUT;
    encoder.setCount(0);
    lastEncoderCount = encoder.getCount();
    drawAboutScreen(totalSystemTimeTenths, FIRMWARE_VERSION);
}

// ---------------------------------------------------------------------------
// Settings sync helper
// ---------------------------------------------------------------------------

void syncPowerPauseSettings(bool saveToNvs) {
    powerPauseSeconds = constrain(powerPauseSeconds, POWER_PAUSE_SEC_MIN, POWER_PAUSE_SEC_MAX);
    setIdleEntrySecondsSafe(powerPauseSeconds);
    if (saveToNvs) {
        saveSettings();
        settingsDirty = false;
    }
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Apollo Sprayers HVLP - ESP32-S3");
    Serial.println("Initialising...");

    beeperInit();

    // Backlight on
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    // Display
    tft.init();
    tft.setRotation(1);  // 480x320 landscape
    drawStartupScreen();
    Serial.println("Display initialised");

    // Rotary encoder
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachHalfQuad(ENCODER_DT, ENCODER_CLK);
    encoder.setCount((int64_t)(TARGET_PSI_DEFAULT / TARGET_PSI_STEP));
    lastEncoderCount = encoder.getCount();

    // Temperature sensor (ADC)
    pinMode(TEMP_SENSOR_PIN, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(TEMP_SENSOR_PIN, ADC_11db);
    tempSensorInit();

    // Encoder button
    pinMode(ENCODER_BTN, INPUT_PULLUP);
    Serial.println("Encoder initialised");

    // NVS: load persistent data
    // Note: Pressure sensor is initialised on Core 0 inside motorControlTask
    loadHourMeter();
    loadSystemTimer();
    loadJobTimer();
    loadSettings();

    targetPsi = settingsStartPsi;
    setTargetPressureSafe(targetPsi);
    encoder.setCount((int64_t)lroundf(targetPsi / TARGET_PSI_STEP));
    lastEncoderCount = encoder.getCount();

    // Motor control
    delay(100);
    if (motorControlInit()) {
        Serial.println("Motor control initialised");

        pressurePidInit();

        // Push saved PID gains to the shared motor-task structure
        PidController *pid = getPressurePid();
        float kp, ki, kd;
        pidGetGains(pid, &kp, &ki, &kd);
        setPidGainsSafe(kp, ki, kd);
        Serial.printf("Initial PID gains: Kp=%.1f Ki=%.1f Kd=%.1f\n", kp, ki, kd);

        AcFrequency acFreq = detectAcFrequency();
        if (acFreq == AC_FREQ_UNKNOWN) {
            Serial.println("AC frequency: unknown (check AC power)");
        } else {
            Serial.printf("AC frequency: %d Hz\n", (int)acFreq);
        }

        if (dualCoreMotorInit()) {
            Serial.println("Dual-core motor task started");
            setTargetPressureSafe(TARGET_PSI_DEFAULT);
        } else {
            Serial.println("ERROR: Dual-core motor init FAILED");
        }
    } else {
        tft.setTextColor(TFT_RED, COLOR_BG);
        Serial.println("ERROR: Motor control init FAILED");
    }

    delay(2000);
    delay(1500);
    enterRuntimeScreen();
    Serial.println("Entered runtime screen");
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------

void loop() {
    static unsigned long lastDisplayUpdate  = 0;
    static unsigned long lastTempRead       = 0;
    static unsigned long lastBeeperToggle   = 0;
    static bool          beeperOutput       = false;
    static float         smoothedPressure   = 0.0f;
    static bool          displayValid       = false;
    static uint32_t      idleSecondsRemaining = UINT32_MAX;
    static IdleState     idleState          = IDLE_STATE_OFF;
    static IdleState     lastOverlayState   = IDLE_STATE_OFF;
    static uint16_t      motorSpeed         = 0;

    // Read shared motor-task data
    {
        portENTER_CRITICAL(&motorShared.mutex);
        smoothedPressure     = motorShared.smoothedPsi;
        displayValid         = motorShared.pressureValid;
        idleSecondsRemaining = motorShared.idleSecondsRemaining;
        idleState            = (IdleState)motorShared.idleState;
        motorSpeed           = motorShared.motorSpeed;
        portEXIT_CRITICAL(&motorShared.mutex);
    }

    // displaySpeed: clamp to 0 when target is zero (motor off) so the UI
    // correctly shows 0% even if the motor task retains a small residual value.
    uint16_t displaySpeed = (targetPsi <= TARGET_PSI_MIN) ? 0 : motorSpeed;

    // ------------------------------------------------------------------
    // Encoder input
    // ------------------------------------------------------------------
    int64_t encoderCount = encoder.getCount();
    if (encoderCount != lastEncoderCount) {
        int64_t delta    = encoderCount - lastEncoderCount;
        lastEncoderCount = encoderCount;

        if (currentScreen == SCREEN_MENU) {
            menuScrollAccumulator += (int32_t)delta;
            int32_t steps = 0;
            while (menuScrollAccumulator >= 2)  { steps++;  menuScrollAccumulator -= 2; }
            while (menuScrollAccumulator <= -2) { steps--;  menuScrollAccumulator += 2; }
            if (steps != 0) {
                int32_t newIdx = constrain((int32_t)menuIndex + steps, 0, MENU_OPTION_COUNT - 1);
                menuIndex = (uint8_t)newIdx;
                encoder.setCount(menuIndex);
                lastEncoderCount = encoder.getCount();
                drawMenuScreen(menuIndex, false);
            }

        } else if (currentScreen == SCREEN_RUNTIME) {
            // Shutdown locks the encoder entirely — nothing should change on that screen
            if (overTempShutdown) {
                lastEncoderCount = encoderCount;  // consume the movement silently
            } else {
                // Accumulate raw ticks; 2 ticks = 1 step of TARGET_PSI_STEP (0.1 PSI),
                // matching the 2-tick-per-step behaviour used on all other menus.
                static int32_t runtimeEncAcc = 0;
                runtimeEncAcc += (int32_t)delta;
                int32_t steps = 0;
                while (runtimeEncAcc >= 2)  { steps++;  runtimeEncAcc -= 2; }
                while (runtimeEncAcc <= -2) { steps--;  runtimeEncAcc += 2; }

                if (steps != 0) {
                    if (idleState != IDLE_STATE_OFF) {
                        requestIdleExitSafe();
                        resumeJobTimer();
                    }
                    float newTarget = targetPsi + (steps * TARGET_PSI_STEP);
                    targetPsi = constrain(newTarget, TARGET_PSI_MIN, TARGET_PSI_MAX);
                    setTargetPressureSafe(targetPsi);
                    requestPidReset();

                    if (targetPsi <= TARGET_PSI_MIN) {
                        setMotorEnabledSafe(false);
                        pauseJobTimer();
                        // Only update display if the overtemp overlay isn't covering the screen
                        if (!overTempWarning) drawRuntimeMotorPower(0, true);
                    } else {
                        setMotorEnabledSafe(true);
                        resumeJobTimer();
                    }
                    // Don't paint over the overtemp overlay — the new target will be
                    // reflected on screen once the overlay clears
                    if (!overTempWarning) {
                        drawRuntimeTarget(targetPsi, smoothedPressure, displayUnits, displayValid, false, motorSpeed);
                    }
                }
            }

        } else if (currentScreen == SCREEN_SETTINGS) {
            if (settingsEditing) {
                if (settingsIndex == 0) {
                    static int32_t pauseAcc = 0;
                    pauseAcc += (int32_t)delta;
                    int32_t steps = 0;
                    while (pauseAcc >= 2)  { steps++;  pauseAcc -= 2; }
                    while (pauseAcc <= -2) { steps--;  pauseAcc += 2; }
                    if (steps != 0) {
                        int32_t next = (int32_t)powerPauseSeconds + steps * POWER_PAUSE_SEC_STEP;
                        powerPauseSeconds = (uint16_t)constrain(next, POWER_PAUSE_SEC_MIN, POWER_PAUSE_SEC_MAX);
                        settingsDirty = true;
                        syncPowerPauseSettings(false);
                        drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled,
                                                  powerPauseWarnSeconds, displayUnits, true, true);
                    }
                } else if (settingsIndex == 1) {
                    static int32_t beeperAcc = 0;
                    beeperAcc += (int32_t)delta;
                    if (beeperAcc >= 2 || beeperAcc <= -2) {
                        powerPauseBeeperEnabled = !powerPauseBeeperEnabled;
                        settingsDirty = true;
                        drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled,
                                                  powerPauseWarnSeconds, displayUnits, true, true);
                        beeperAcc = 0;
                    }
                } else if (settingsIndex == 2) {
                    static int32_t unitsAcc = 0;
                    unitsAcc += (int32_t)delta;
                    if (unitsAcc >= 2 || unitsAcc <= -2) {
                        displayUnits = (displayUnits == UNITS_IMPERIAL) ? UNITS_METRIC : UNITS_IMPERIAL;
                        settingsDirty = true;
                        drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled,
                                                  powerPauseWarnSeconds, displayUnits, true, true);
                        unitsAcc = 0;
                    }
                } else if (settingsIndex == 3) {
                    static int32_t themeAcc = 0;
                    themeAcc += (int32_t)delta;
                    if (themeAcc >= 2 || themeAcc <= -2) {
                        lightThemeEnabled = !lightThemeEnabled;
                        settingsDirty = true;
                        // Force full screen redraw — background colour has changed
                        drawPowerPauseSettingsScreen(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled,
                                                     powerPauseWarnSeconds, displayUnits, true, true);
                        themeAcc = 0;
                    }
                }
            } else {
                settingsScrollAccumulator += (int32_t)delta;
                int32_t steps = 0;
                while (settingsScrollAccumulator >= 2)  { steps++;  settingsScrollAccumulator -= 2; }
                while (settingsScrollAccumulator <= -2) { steps--;  settingsScrollAccumulator += 2; }
                if (steps != 0) {
                    uint8_t prev = settingsIndex;
                    settingsIndex = (uint8_t)constrain((int32_t)settingsIndex + steps, 0, SETTINGS_OPTION_COUNT - 1);
                    encoder.setCount(settingsIndex);
                    lastEncoderCount = encoder.getCount();
                    if (prev != settingsIndex) {
                        drawPowerPauseSettingsRow(prev, powerPauseSeconds, powerPauseBeeperEnabled,
                                                  powerPauseWarnSeconds, displayUnits, false, false);
                        drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled,
                                                  powerPauseWarnSeconds, displayUnits, true, false);
                    }
                }
            }

        } else if (currentScreen == SCREEN_SUPPORT) {
            supportScrollAccumulator += (int32_t)delta;
            int32_t steps = 0;
            while (supportScrollAccumulator >= 2)  { steps++;  supportScrollAccumulator -= 2; }
            while (supportScrollAccumulator <= -2) { steps--;  supportScrollAccumulator += 2; }
            if (steps != 0) {
                uint8_t prev = supportMenuIndex;
                supportMenuIndex = (uint8_t)constrain((int32_t)supportMenuIndex + steps, 0, SUPPORT_OPTION_COUNT - 1);
                encoder.setCount(supportMenuIndex);
                lastEncoderCount = encoder.getCount();
                if (prev != supportMenuIndex) {
                    drawSupportMenuScreen(supportMenuIndex);
                }
            }

        } else if (currentScreen == SCREEN_TIMERS) {
            timersScrollAccumulator += (int32_t)delta;
            int32_t steps = 0;
            while (timersScrollAccumulator >= 2)  { steps++;  timersScrollAccumulator -= 2; }
            while (timersScrollAccumulator <= -2) { steps--;  timersScrollAccumulator += 2; }
            if (steps != 0) {
                timersIndex = (uint8_t)constrain((int32_t)timersIndex + steps, 0, 2);
                encoder.setCount(timersIndex);
                lastEncoderCount = encoder.getCount();
                drawTimersScreen(totalRuntimeTenths, totalJobTimeTenths, timersIndex);
            }
        }
    }

    // ------------------------------------------------------------------
    // Button input (debounced)
    // ------------------------------------------------------------------
    static bool          lastButtonState  = false;
    static unsigned long lastButtonChange = 0;
    bool buttonPressed = (digitalRead(ENCODER_BTN) == LOW);
    if (buttonPressed != lastButtonState && (millis() - lastButtonChange) > 30) {
        lastButtonChange = millis();
        lastButtonState  = buttonPressed;

        if (!buttonPressed) {  // Act on release
            if (currentScreen == SCREEN_MENU) {
                switch (menuIndex) {
                    case 0: enterRuntimeScreen();  break;
                    case 1: enterSettingsScreen(); break;
                    case 2: enterTimersScreen();   break;
                    case 3: enterSupportScreen();  break;
                    case 4: enterAboutScreen();    break;
                }

            } else if (currentScreen == SCREEN_RUNTIME) {
                if (overTempShutdown) {
                    // Hard shutdown — button does nothing, unit must be restarted
                    // (silently consume the press)
                } else {
                    // Normal press (includes during overTempWarning): stop motor and go to menu.
                    // The overtemp overlay will persist on next entry to runtime until temp drops.
                    pauseJobTimer();
                    setMotorEnabledSafe(false);
                    enterMenuScreen();
                }

            } else if (currentScreen == SCREEN_SETTINGS) {
                if (settingsEditing) {
                    settingsEditing = false;
                    drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled,
                                              powerPauseWarnSeconds, displayUnits, true, false);
                    drawPowerPauseSettingsFooter("Press to edit / select", COLOR_SUCCESS);
                } else {
                    if (settingsIndex <= 3) {
                        settingsEditing = true;
                        drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled,
                                                  powerPauseWarnSeconds, displayUnits, true, true);
                        drawPowerPauseSettingsFooter("Rotate to adjust, press to exit", COLOR_MENU_EDIT);
                    } else {  // Exit row (index 4)
                        if (settingsDirty) syncPowerPauseSettings(true);
                        enterMenuScreen();
                    }
                }

            } else if (currentScreen == SCREEN_SUPPORT) {
                switch (supportMenuIndex) {
                    case 0: enterSupportFaqScreen();     break;
                    case 1: enterSupportTechScreen();    break;
                    case 2: enterSupportContactScreen(); break;
                    case 3: enterMenuScreen();           break;
                }

            } else if (currentScreen == SCREEN_SUPPORT_FAQ ||
                       currentScreen == SCREEN_SUPPORT_TECH ||
                       currentScreen == SCREEN_SUPPORT_CONTACT) {
                enterSupportScreen();

            } else if (currentScreen == SCREEN_TIMERS) {
                if (timersIndex == 0) {
                    // Reset job timer
                    resetJobTimeTenths();
                    saveJobTimer();
                    drawTimersScreen(totalRuntimeTenths, totalJobTimeTenths, timersIndex);
                    delay(1000);
                } else if (timersIndex == 1) {
                    // Reset filter timer
                    totalRuntimeTenths = 0;
                    saveHourMeter();
                    drawTimersScreen(totalRuntimeTenths, totalJobTimeTenths, timersIndex);
                    delay(1000);
                } else {
                    enterMenuScreen();
                }

            } else if (currentScreen == SCREEN_ABOUT) {
                enterMenuScreen();
            }
        }
    }

    unsigned long now = millis();

    // ------------------------------------------------------------------
    // Serial debug output
    // ------------------------------------------------------------------
#if DEBUG_SERIAL_OUTPUT
    static unsigned long lastSerialDebug = 0;
    if (now - lastSerialDebug >= SERIAL_DEBUG_INTERVAL_MS) {
        lastSerialDebug = now;
        float currentPsi;
        uint32_t pauseTicks;
        portENTER_CRITICAL(&motorShared.mutex);
        currentPsi  = motorShared.smoothedPsi;
        pauseTicks  = motorShared.idleSecondsRemaining;
        portEXIT_CRITICAL(&motorShared.mutex);
        Serial.printf("[DEBUG] PSI: %.2f | Power Pause: ", currentPsi);
        if (pauseTicks == UINT32_MAX) Serial.println("N/A");
        else                          Serial.printf("%lu sec\n", (unsigned long)pauseTicks);
    }
#endif

    // ------------------------------------------------------------------
    // Temperature polling
    // ------------------------------------------------------------------
    if (now - lastTempRead >= TEMP_READ_INTERVAL_MS) {
        lastTempRead = now;

        // Rolling 3-sample average to smooth noisy ADC readings
        static float tempReadings[3] = {0.0f, 0.0f, 0.0f};
        static uint8_t tempReadIndex = 0;
        static bool tempBufferFull = false;

        tempReadings[tempReadIndex] = tempSensorReadC();
        tempReadIndex = (tempReadIndex + 1) % 3;
        if (tempReadIndex == 0) tempBufferFull = true;

        uint8_t sampleCount = tempBufferFull ? 3 : tempReadIndex;
        float tempSum = 0.0f;
        for (uint8_t i = 0; i < sampleCount; i++) tempSum += tempReadings[i];
        currentTemperatureC = tempSum / (float)sampleCount;

        // --- SHUTDOWN level (266 F / 130 C) --- once set, requires restart to clear
        if (!overTempShutdown && currentTemperatureC >= TEMP_SHUTDOWN_SETPOINT) {
            overTempShutdown = true;
            overTempWarning  = true;
            overTempActive   = true;
            setMotorEnabledSafe(false);
            requestPidReset();
            Serial.printf("TEMP SHUTDOWN: %.1f C (%.1f F) — restart required\n",
                          currentTemperatureC,
                          (currentTemperatureC * 9.0f / 5.0f) + 32.0f);
        }
        // --- WARNING level (230 F / 110 C) --- motor keeps running
        else if (!overTempShutdown && currentTemperatureC >= TEMP_WARN_SETPOINT) {
            if (!overTempWarning) {
                overTempWarning             = true;
                overTempActive              = true;
                overTempWarnBuzzStart       = millis(); // start 1-second buzz
                setBeeper(true);
                Serial.printf("TEMP WARNING: %.1f C (%.1f F) — motor continues\n",
                              currentTemperatureC,
                              (currentTemperatureC * 9.0f / 5.0f) + 32.0f);
            }
        }
        // --- Below warning --- clear warning only (shutdown flag persists until restart)
        else if (!overTempShutdown && overTempWarning) {
            overTempWarning = false;
            overTempActive  = false;
            Serial.println("Temperature normal: warning cleared");
        }
    }

    // ------------------------------------------------------------------
    // Runtime screen updates
    // ------------------------------------------------------------------
    if (currentScreen == SCREEN_RUNTIME) {
        static IdleState lastIdleState      = IDLE_STATE_OFF;
        static uint32_t  idleHoldEntryTime  = 0;

        // Track idle state transitions for job timer and overlay
        if (idleState != lastIdleState) {
            if (idleState != IDLE_STATE_OFF && lastIdleState == IDLE_STATE_OFF) {
                pauseJobTimer();
            } else if (idleState == IDLE_STATE_OFF && lastIdleState != IDLE_STATE_OFF) {
                resumeJobTimer();
            }
            if (idleState == IDLE_STATE_HOLD && lastIdleState != IDLE_STATE_HOLD) {
                idleHoldEntryTime = millis();
            }
            lastIdleState = idleState;
        }

        // Redraw base screen when returning from an overlay
        // Track overtemp state transitions so we can force-redraw the overlay
        static bool lastOverTempActive = false;
        bool overTempTransition = (overTempActive != lastOverTempActive);
        lastOverTempActive = overTempActive;

        if (idleState != lastOverlayState) {
            if (idleState == IDLE_STATE_OFF && !overTempActive) {
                drawRuntimeStatic(displayUnits);
                drawRuntimeTarget(targetPsi, smoothedPressure, displayUnits, displayValid, true, displaySpeed);
                drawRuntimeMotorPower(displaySpeed, true);
                drawRuntimeJobTime(getJobTimeSeconds(), true);
                drawRuntimeTemperature(currentTemperatureC, displayUnits, true);
            }
            lastOverlayState = idleState;
        } else if (overTempTransition && !overTempActive) {
            // Overtemp warning cleared — redraw base screen
            drawRuntimeStatic(displayUnits);
            drawRuntimeTarget(targetPsi, smoothedPressure, displayUnits, displayValid, true, displaySpeed);
            drawRuntimeMotorPower(displaySpeed, true);
            drawRuntimeJobTime(getJobTimeSeconds(), true);
            drawRuntimeTemperature(currentTemperatureC, displayUnits, true);
        }

        // 1-second overtemp warning buzz — cut off automatically
        if (overTempWarnBuzzStart != 0 && (millis() - overTempWarnBuzzStart) >= 1000UL) {
            setBeeper(false);
            overTempWarnBuzzStart = 0;
            beeperOutput = false;
            lastBeeperToggle = millis();
        }

        // ---- Overtemp WARNING flash: 3 s on, 2 s off (shutdown = always on) ----
        static unsigned long overTempFlashTimer   = 0;
        static bool          overTempOverlayShown = false;

        const unsigned long OVERTEMP_ON_MS  = 3000;
        const unsigned long OVERTEMP_OFF_MS = 2000;

        if (overTempShutdown) {
            // Hard shutdown — overlay is permanently on, no flashing
            if (!overTempOverlayShown) {
                overTempOverlayShown = true;
                drawRuntimeOverTempOverlay(currentTemperatureC, true);
            } else {
                drawRuntimeOverTempOverlay(currentTemperatureC, false);
            }
        } else if (overTempWarning) {
            if (!overTempOverlayShown && (now - overTempFlashTimer >= OVERTEMP_OFF_MS)) {
                // Off period elapsed — show overlay, beep
                overTempOverlayShown = true;
                overTempFlashTimer   = now;
                drawRuntimeOverTempOverlay(currentTemperatureC, true);
                // Beep each time the overlay reappears (same 1-second buzz)
                overTempWarnBuzzStart = now;
                setBeeper(true);
            } else if (overTempOverlayShown && (now - overTempFlashTimer >= OVERTEMP_ON_MS)) {
                // On period elapsed — hide overlay, redraw runtime
                overTempOverlayShown = false;
                overTempFlashTimer   = now;
                drawRuntimeStatic(displayUnits);
                drawRuntimeTarget(targetPsi, smoothedPressure, displayUnits, displayValid, true, displaySpeed);
                drawRuntimeMotorPower(displaySpeed, true);
                drawRuntimeJobTime(getJobTimeSeconds(), true);
                drawRuntimeTemperature(currentTemperatureC, displayUnits, true);
            } else if (overTempOverlayShown) {
                // Still within on period — keep overlay fresh
                drawRuntimeOverTempOverlay(currentTemperatureC, false);
            }
        } else {
            // Warning cleared — ensure overlay state is reset
            if (overTempOverlayShown) {
                overTempOverlayShown = false;
                overTempFlashTimer   = 0;
            }
        }

        bool showOverTempOverlay = overTempShutdown || (overTempWarning && overTempOverlayShown);

        if (!overTempShutdown && !overTempWarning && idleState != IDLE_STATE_OFF) {
            uint32_t timeoutRemaining = UINT32_MAX;
            if (idleState == IDLE_STATE_HOLD) {
                uint32_t idleDuration = (millis() - idleHoldEntryTime) / 1000;
                uint32_t maxIdleTime  = 900;  // 15 minutes
                timeoutRemaining = (idleDuration < maxIdleTime) ? (maxIdleTime - idleDuration) : 0;
            }
            drawRuntimePowerPauseOverlay(idleState, timeoutRemaining, false);
        }

        // Power pause hard timeout (15 minutes) -> return to menu
        if (idleState == IDLE_STATE_HOLD) {
            uint32_t idleDuration = (millis() - idleHoldEntryTime) / 1000;
            if (idleDuration >= 900) {
                pauseJobTimer();
                setMotorEnabledSafe(false);
                requestPidReset();
                enterMenuScreen();
                Serial.println("Power pause timeout — returned to menu");
                return;
            }
        }

        // Periodic display refresh — suppressed while any overlay is visible
        // showOverTempOverlay2 mirrors the flash state so we never draw over the overlay
        bool showOverTempOverlay2 = overTempShutdown || (overTempWarning && overTempOverlayShown);

        // ---- Filter maintenance flash (>= 10 hours) ----
        // Show the filter warning overlay for 4s every 10s until the timer is reset.
        static unsigned long filterFlashTimer   = 0;
        static bool          filterOverlayShown = false;
        const unsigned long  FILTER_ON_MS       = 4000;
        const unsigned long  FILTER_OFF_MS      = 10000;

        bool filterCondition = (totalRuntimeTenths >= 100) &&
                               !showOverTempOverlay2 &&
                               (idleState == IDLE_STATE_OFF);

        if (filterCondition) {
            if (!filterOverlayShown && (now - filterFlashTimer >= FILTER_OFF_MS)) {
                filterOverlayShown = true;
                filterFlashTimer   = now;
                drawRuntimeFilterWarningOverlay();
            } else if (filterOverlayShown && (now - filterFlashTimer >= FILTER_ON_MS)) {
                filterOverlayShown = false;
                filterFlashTimer   = now;
                // Restore base runtime screen after overlay hides
                drawRuntimeStatic(displayUnits);
                drawRuntimeTarget(targetPsi, smoothedPressure, displayUnits, displayValid, true, displaySpeed);
                drawRuntimeMotorPower(displaySpeed, true);
                drawRuntimeJobTime(getJobTimeSeconds(), true);
                drawRuntimeTemperature(currentTemperatureC, displayUnits, true);
            }
        } else if (filterOverlayShown) {
            // Condition cleared (timer reset or overlay preempted) — restore base screen
            filterOverlayShown = false;
            filterFlashTimer   = 0;
            if (!showOverTempOverlay2 && idleState == IDLE_STATE_OFF) {
                drawRuntimeStatic(displayUnits);
                drawRuntimeTarget(targetPsi, smoothedPressure, displayUnits, displayValid, true, displaySpeed);
                drawRuntimeMotorPower(displaySpeed, true);
                drawRuntimeJobTime(getJobTimeSeconds(), true);
                drawRuntimeTemperature(currentTemperatureC, displayUnits, true);
            }
        }

        bool anyOverlayShowing = showOverTempOverlay2 || filterOverlayShown || (idleState != IDLE_STATE_OFF);

        if (now - lastDisplayUpdate >= DISPLAY_PRESSURE_INTERVAL_MS) {
            lastDisplayUpdate = now;
            if (!anyOverlayShowing) {
                drawRuntimeTarget(targetPsi, smoothedPressure, displayUnits, displayValid, false, displaySpeed);
                drawRuntimeMotorPower(displaySpeed);
                drawRuntimeJobTime(getJobTimeSeconds());
                drawRuntimeTemperature(currentTemperatureC, displayUnits);
            }
        }

        // Beeper: power-pause warning countdown
        // Skip power-pause beeper while the overtemp warning buzz is still running
        bool warningActive = false;
        if (overTempWarnBuzzStart == 0 &&   // don't fight the 1-second overtemp buzz
            powerPauseBeeperEnabled && idleState == IDLE_STATE_OFF &&
            idleSecondsRemaining != UINT32_MAX && idleSecondsRemaining <= powerPauseWarnSeconds) {
            warningActive = true;
        }
        if (warningActive) {
            uint32_t span       = (powerPauseWarnSeconds > 1) ? (powerPauseWarnSeconds - 1) : 1;
            uint32_t remaining  = constrain(idleSecondsRemaining > 1 ? idleSecondsRemaining - 1 : 0, (uint32_t)0, span);
            uint32_t intervalMs = 200 + (1000 * remaining) / span;
            if (now - lastBeeperToggle >= intervalMs) {
                lastBeeperToggle = now;
                beeperOutput = !beeperOutput;
                setBeeper(beeperOutput);
            }
        } else {
            if (beeperOutput) { beeperOutput = false; setBeeper(false); }
            lastBeeperToggle = now;
        }
    } else {
        // Not on runtime screen — silence beeper
        if (beeperOutput) { beeperOutput = false; setBeeper(false); }
    }

    // ------------------------------------------------------------------
    // Hour meter & job timer (every 6 minutes = 0.1 hr)
    // ------------------------------------------------------------------
    if (millis() - lastHourMeterUpdate >= 360000UL) {
        if (currentScreen == SCREEN_RUNTIME && targetPsi > TARGET_PSI_MIN && getMotorSpeedSafe() > 0) {
            lastHourMeterUpdate = millis();
            totalRuntimeTenths++;
            saveHourMeter();
            totalJobTimeTenths++;
            saveJobTimer();
            totalSystemTimeTenths++;
            saveSystemTimer();
        }
    }

#if DEBUG_OVERLAY_PREVIEW
    // ------------------------------------------------------------------
    // Debug overlay preview carousel
    // When sitting on the main menu, cycle through every warning overlay
    // on a 3-second rotation so the UI can be verified without fault conditions.
    // ------------------------------------------------------------------
    if (currentScreen == SCREEN_MENU) {
        static unsigned long lastOverlayPreviewTime = 0;
        static uint8_t       overlayPreviewStage    = 0;
        const uint8_t        OVERLAY_STAGE_COUNT    = 5;
        const unsigned long  OVERLAY_INTERVAL_MS    = 3000;

        if (lastOverlayPreviewTime == 0 || (now - lastOverlayPreviewTime) >= OVERLAY_INTERVAL_MS) {
            lastOverlayPreviewTime = now;
            drawDebugOverlayPreview(overlayPreviewStage);
            overlayPreviewStage = (overlayPreviewStage + 1) % OVERLAY_STAGE_COUNT;
        }
    }
#endif
}
