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

// Runtime target pressure
float       targetPsi               = TARGET_PSI_DEFAULT;

// Encoder tracking
int64_t     lastEncoderCount        = 0;

// Temperature / overtemp
float       currentTemperatureC     = -999.0f;
bool        overTempActive          = false;

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
    setMotorEnabledSafe(true);
    if (overTempActive) {
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
    drawRuntimeJobTime(0, true);
    drawRuntimeTemperature(currentTemperatureC, displayUnits, true);

    // Filter maintenance warning (>= 10 hours)
    if (totalRuntimeTenths >= 100) {
        drawRuntimeFilterWarningOverlay();
        delay(5000);
        drawRuntimeStatic(displayUnits);
        drawRuntimeTarget(targetPsi, currentPsi, displayUnits, pressureValid, true);
        drawRuntimeMotorPower(0, true);
        drawRuntimeJobTime(0, true);
        drawRuntimeTemperature(currentTemperatureC, displayUnits, true);
    }
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
            if (idleState != IDLE_STATE_OFF) {
                requestIdleExitSafe();
                resumeJobTimer();
            }
            float newTarget = targetPsi + (delta * TARGET_PSI_STEP);
            targetPsi = constrain(newTarget, TARGET_PSI_MIN, TARGET_PSI_MAX);
            setTargetPressureSafe(targetPsi);
            requestPidReset();  // Clear integral so stale buildup doesn't carry over to the new setpoint
            drawRuntimeTarget(targetPsi, smoothedPressure, displayUnits, displayValid);

        } else if (currentScreen == SCREEN_SETTINGS) {
            if (settingsEditing) {
                if (settingsIndex == 0) {
                    int32_t next = (int32_t)powerPauseSeconds + (int32_t)delta * POWER_PAUSE_SEC_STEP;
                    powerPauseSeconds = (uint16_t)constrain(next, POWER_PAUSE_SEC_MIN, POWER_PAUSE_SEC_MAX);
                    settingsDirty = true;
                    syncPowerPauseSettings(false);
                    drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled,
                                              powerPauseWarnSeconds, displayUnits, true, true);
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
                pauseJobTimer();
                setMotorEnabledSafe(false);
                enterMenuScreen();

            } else if (currentScreen == SCREEN_SETTINGS) {
                if (settingsEditing) {
                    settingsEditing = false;
                    drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled,
                                              powerPauseWarnSeconds, displayUnits, true, false);
                    drawPowerPauseSettingsFooter("Press to edit / select", COLOR_SUCCESS);
                } else {
                    if (settingsIndex <= 2) {
                        settingsEditing = true;
                        drawPowerPauseSettingsRow(settingsIndex, powerPauseSeconds, powerPauseBeeperEnabled,
                                                  powerPauseWarnSeconds, displayUnits, true, true);
                        drawPowerPauseSettingsFooter("Rotate to adjust, press to exit", COLOR_MENU_EDIT);
                    } else {  // Exit row
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
        currentTemperatureC = tempSensorReadC();

        if (currentTemperatureC > TEMP_OVERHEAT_SETPOINT) {
            if (!overTempActive) {
                overTempActive = true;
                setMotorEnabledSafe(false);
                requestPidReset();
                Serial.println("Overtemperature: motor disabled");
            }
        } else if (overTempActive && currentTemperatureC < TEMP_OVERHEAT_CLEAR) {
            overTempActive = false;
            if (currentScreen == SCREEN_RUNTIME) setMotorEnabledSafe(true);
            Serial.println("Temperature normal: motor enabled");
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
        if (idleState != lastOverlayState) {
            if (idleState == IDLE_STATE_OFF) {
                drawRuntimeStatic(displayUnits);
                drawRuntimeTarget(targetPsi, smoothedPressure, displayUnits, displayValid, true);
                drawRuntimeMotorPower(motorSpeed, true);
                drawRuntimeJobTime(getJobTimeSeconds(), true);
                drawRuntimeTemperature(currentTemperatureC, displayUnits, true);
            }
            lastOverlayState = idleState;
        }

        // Overlays
        if (overTempActive) {
            drawRuntimeOverTempOverlay(currentTemperatureC, false);
        } else if (idleState != IDLE_STATE_OFF) {
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

        // Periodic display refresh
        if (now - lastDisplayUpdate >= DISPLAY_PRESSURE_INTERVAL_MS) {
            lastDisplayUpdate = now;
            if (idleState == IDLE_STATE_OFF && !overTempActive) {
                drawRuntimeTarget(targetPsi, smoothedPressure, displayUnits, displayValid);
                drawRuntimeMotorPower(motorSpeed);
                drawRuntimeJobTime(getJobTimeSeconds());
                drawRuntimeTemperature(currentTemperatureC, displayUnits);
            }
        }

        // Beeper: power-pause warning countdown
        bool warningActive = false;
        if (powerPauseBeeperEnabled && idleState == IDLE_STATE_OFF &&
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
        lastHourMeterUpdate = millis();
        if (currentScreen == SCREEN_RUNTIME && getMotorSpeedSafe() > 0) {
            totalRuntimeTenths++;
            saveHourMeter();
            totalJobTimeTenths++;
            saveJobTimer();
            totalSystemTimeTenths++;
            saveSystemTimer();
        }
    }
}
