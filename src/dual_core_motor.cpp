/**
 * @file dual_core_motor.cpp
 * @brief Dual-core motor control implementation
 * 
 * Runs time-critical motor control loop on Core 0 at high frequency
 * while display/UI runs on Core 1 without blocking motor control.
 */

#include "dual_core_motor.h"
#include "pressure_sensor.h"
#include "motor_control.h"
#include "pid_controller.h"
#include <math.h>

// Global shared data instance
MotorSharedData motorShared = {
    .targetPsi = 6.0f,
    .motorEnabled = false,
    .pidResetRequest = false,
    .pidGainsChanged = true,  // Start as true to load initial gains
    .pidKp = PID_KP_DEFAULT,
    .pidKi = PID_KI_DEFAULT,
    .pidKd = PID_KD_DEFAULT,
    .idleEntryDeviationPsi = IDLE_ENTRY_DEVIATION_PSI,
        .idleEntrySeconds = IDLE_ENTRY_SECONDS,
    .currentPsi = 0.0f,
    .rawPressure = 0,
    .smoothedPsi = 0.0f,
    .motorSpeed = 0,
    .pidOutput = 0.0f,
    .pressureValid = false,
    .idleSecondsRemaining = UINT32_MAX,
    .idleState = IDLE_STATE_OFF,
    .idleExitRequest = false,
    .isMax = false,
    .loopCount = 0,
    .loopTimeUs = 0,
    .maxLoopTimeUs = 0,
    .mutex = portMUX_INITIALIZER_UNLOCKED
};

// Task handle for motor control task
static TaskHandle_t motorTaskHandle = NULL;

// Local PID controller for motor task
static PidController motorPid;

static uint16_t calculateMaxSpeedFromTarget(float targetPsi) {
    if (targetPsi <= 3.0f) {
        return 500;
    }
    if (targetPsi >= 6.0f) {
        return 1000;
    }

    float normalized = (targetPsi - 3.0f) / 3.0f;
    float maxSpeed = 500.0f + (normalized * 500.0f);
    return (uint16_t)lroundf(maxSpeed);
}

/**
 * @brief Motor control task - runs on Core 0
 * 
 * High-frequency loop that handles:
 * - Pressure sensor reading
 * - PID calculation
 * - Motor speed updates
 */
void motorControlTask(void *parameter) {
    Serial.printf("Motor Control Task started on Core %d\n", xPortGetCoreID());
    
    // Initialize pressure sensor on Core 0 (Wire/I2C must be used from the core it was initialized on)
    Serial.println("Initializing pressure sensor on Core 0...");
    if (!pressureSensor.begin()) {
        Serial.println("ERROR: Pressure sensor initialization failed on Core 0!");
    } else {
        Serial.println("Pressure sensor initialized successfully on Core 0");
    }
    
    // Initialize local PID controller
    pidInit(&motorPid, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
    
    // Timing variables
    uint32_t lastLoopTime = micros();
    uint32_t loopStartTime;
    float smoothedPressure = 0.0f;
    const float smoothingAlpha = 1.0f;  // EMA smoothing factor
    uint16_t lastSpeed = 0;

    IdleState idleState = IDLE_STATE_OFF;
    uint32_t idleCounter = 0;
    uint32_t idleStableCounter = 0;
    uint16_t idleHoldSpeed = 0;
    float maxPressureRecorded = 0.0f;  // tracks peak pressure for MAX-mode power pause

    const uint32_t idleStableLoops = (IDLE_STABLE_SECONDS * 1000000UL) / MOTOR_LOOP_INTERVAL_US;
    
    // Main motor control loop
    while (true) {
        loopStartTime = micros();
        
        // Check for PID reset request or gains change
        bool resetRequested = false;
        bool gainsChanged = false;
        bool idleExitRequested = false;
        float newKp, newKi, newKd;
        float idleEntryDeviation = IDLE_ENTRY_DEVIATION_PSI;
        float target = 0.0f;
        bool enabled = false;
        uint16_t idleEntrySeconds = IDLE_ENTRY_SECONDS;
        portENTER_CRITICAL(&motorShared.mutex);
        if (motorShared.pidResetRequest) {
            resetRequested = true;
            motorShared.pidResetRequest = false;
        }
        if (motorShared.pidGainsChanged) {
            gainsChanged = true;
            newKp = motorShared.pidKp;
            newKi = motorShared.pidKi;
            newKd = motorShared.pidKd;
            motorShared.pidGainsChanged = false;
        }
        if (motorShared.idleExitRequest) {
            idleExitRequested = true;
            motorShared.idleExitRequest = false;
        }
        target = motorShared.targetPsi;
        enabled = motorShared.motorEnabled;
        idleEntryDeviation = motorShared.idleEntryDeviationPsi;
        idleEntrySeconds = motorShared.idleEntrySeconds;
        portEXIT_CRITICAL(&motorShared.mutex);
        uint32_t idleEntryLoops = (uint32_t)idleEntrySeconds * (1000000UL / MOTOR_LOOP_INTERVAL_US);
        
        if (resetRequested) {
            pidReset(&motorPid);
        }
        
        if (gainsChanged) {
            pidSetGains(&motorPid, newKp, newKi, newKd);
            // Note: No Serial output in motor loop - can cause timing issues
        }
        
        // Read pressure sensor
        PressureReading reading = pressureSensor.readPressure();
        
        uint16_t speed = 0;
        bool valid = reading.valid;
        if (valid && enabled && target > 0.0f) {
            // Exponential moving average smoothing
            smoothedPressure = (smoothingAlpha * reading.pressurePsi) + 
                               ((1.0f - smoothingAlpha) * smoothedPressure);

            if (idleExitRequested && idleState != IDLE_STATE_OFF) {
                idleState = IDLE_STATE_OFF;
                idleCounter = 0;
                idleStableCounter = 0;
                maxPressureRecorded = 0.0f;
                pidReset(&motorPid);
            }
            
            bool inActiveBand = fabsf(smoothedPressure - target) <= idleEntryDeviation;
            bool isMaxMode = (target >= MAX_PSI_THRESHOLD);
            if (idleState == IDLE_STATE_OFF && !isMaxMode) {
                // Only count idle ticks when target PSI is above idle PSI (power pause is pointless below it)
                if (inActiveBand && target > IDLE_TARGET_PSI) {
                    if (idleCounter < idleEntryLoops) {
                        idleCounter += IDLE_LOOP_INCREMENT;
                    }
                } else if (idleCounter > 0) {
                    if (idleCounter > IDLE_ENTRY_DECREASE) {
                        idleCounter -= IDLE_ENTRY_DECREASE;
                    } else {
                        idleCounter = 0;
                    }
                }

                if (idleCounter >= idleEntryLoops && idleEntryLoops > 0) {
                    idleState = IDLE_STATE_PID_RAMP;
                    idleCounter = 0;
                    idleStableCounter = 0;
                    idleHoldSpeed = 0;
                    pidReset(&motorPid);
                }
            }

            float pidOut = 0.0f;

            uint32_t idleSecondsRemaining = 0;
            if (idleState == IDLE_STATE_OFF && idleEntryLoops > 0) {
                uint32_t remainingLoops = (idleCounter < idleEntryLoops) ? (idleEntryLoops - idleCounter) : 0;
                idleSecondsRemaining = (remainingLoops * MOTOR_LOOP_INTERVAL_US + 999999UL) / 1000000UL;
            } else if (idleState == IDLE_STATE_OFF && idleEntryLoops == 0) {
                idleSecondsRemaining = 0;
            } else {
                idleSecondsRemaining = 0;
            }

            if (idleState == IDLE_STATE_PID_RAMP) {
                motorPid.setpoint = IDLE_TARGET_PSI;
                pidOut = pidCalculate(&motorPid, smoothedPressure);
                int adjustedSpeed = (int)lastSpeed + (int)lroundf(pidOut);
                adjustedSpeed = constrain(adjustedSpeed, 200, 1000);
                speed = (uint16_t)adjustedSpeed;
                setMotorSpeed(speed);
                lastSpeed = speed;

                if (fabsf(smoothedPressure - IDLE_TARGET_PSI) <= IDLE_STABLE_BAND_PSI) {
                    if (idleStableCounter < idleStableLoops) {
                        idleStableCounter++;
                    }
                } else {
                    idleStableCounter = 0;
                }

                if (idleStableCounter >= idleStableLoops && idleStableLoops > 0) {
                    idleHoldSpeed = speed;
                    if (idleHoldSpeed < IDLE_MIN_HOLD_SPEED) {
                        idleHoldSpeed = IDLE_MIN_HOLD_SPEED;
                    }
                    idleState = IDLE_STATE_HOLD;
                    pidReset(&motorPid);
                }
            } else if (idleState == IDLE_STATE_HOLD) {
                setMotorSpeed(idleHoldSpeed);
                speed = idleHoldSpeed;
                pidOut = (float)idleHoldSpeed;
                lastSpeed = speed;

                if (smoothedPressure < (IDLE_TARGET_PSI - IDLE_EXIT_DROP_PSI)) {
                    idleState = IDLE_STATE_OFF;
                    idleCounter = 0;
                    idleStableCounter = 0;
                    pidReset(&motorPid);
                }
            } else {
                if (isMaxMode) {
                    // MAX mode: run turbine at full power
                    speed = 1000;
                    setMotorSpeed(speed);
                    lastSpeed = speed;
                    pidOut = 1000.0f;

                    // Track peak pressure for deviation-based idle entry
                    if (smoothedPressure > maxPressureRecorded) {
                        maxPressureRecorded = smoothedPressure;
                    }

                    // Idle entry: pressure stays within MAX_PRESSURE_DEVIATION_PSI of the recorded peak
                    if (smoothedPressure >= maxPressureRecorded - MAX_PRESSURE_DEVIATION_PSI) {
                        if (idleCounter < idleEntryLoops) {
                            idleCounter++;
                        }
                    } else if (idleCounter >= IDLE_ENTRY_DECREASE) {
                        // Pressure deviated significantly — decrement counter slowly
                        idleCounter -= IDLE_ENTRY_DECREASE;
                    } else {
                        // Large drop: reset peak and counter
                        idleCounter = 0;
                        maxPressureRecorded = smoothedPressure;
                    }

                    if (idleCounter >= idleEntryLoops && idleEntryLoops > 0) {
                        idleState = IDLE_STATE_PID_RAMP;
                        idleCounter = 0;
                        idleStableCounter = 0;
                        idleHoldSpeed = 0;
                        maxPressureRecorded = 0.0f;
                        pidReset(&motorPid);
                    }
                } else {
                    // Normal PID mode
                    maxPressureRecorded = 0.0f;  // reset when leaving MAX mode
                    motorPid.setpoint = target;
                    pidOut = pidCalculate(&motorPid, smoothedPressure);
                    int adjustedSpeed = (int)lastSpeed + (int)lroundf(pidOut);
                    // Cap the upper bound to a setpoint-proportional ceiling.
                    // Below 3 PSI this prevents the motor from over-driving and
                    // building pressure past the setpoint before PID can react.
                    // int speedCeil = (int)calculateMaxSpeedFromTarget(target);
                    // adjustedSpeed = constrain(adjustedSpeed, 50, speedCeil);
                    
                    //no constraint 
                    adjustedSpeed = constrain(adjustedSpeed, 50, 1000);

                    speed = (uint16_t)adjustedSpeed;
                    setMotorSpeed(speed);
                    lastSpeed = speed;
                }
            }

            portENTER_CRITICAL(&motorShared.mutex);
            motorShared.currentPsi = reading.pressurePsi;
            motorShared.rawPressure = reading.rawValue;
            motorShared.smoothedPsi = smoothedPressure;
            motorShared.motorSpeed = speed;
            motorShared.pidOutput = pidOut;
            motorShared.pressureValid = true;
            motorShared.idleSecondsRemaining = idleSecondsRemaining;
            motorShared.idleState = idleState;
            motorShared.isMax = (target >= MAX_PSI_THRESHOLD);
            portEXIT_CRITICAL(&motorShared.mutex);
        } else if (!enabled || target == 0.0f) {
            // Motor disabled or target set to zero — keep motor off
            setMotorSpeed(0);
            lastSpeed = 0;
            pidReset(&motorPid);
            idleState = IDLE_STATE_OFF;
            idleCounter = 0;
            idleStableCounter = 0;
            maxPressureRecorded = 0.0f;
            
            portENTER_CRITICAL(&motorShared.mutex);
            motorShared.motorSpeed = 0;
            motorShared.pressureValid = valid;
            motorShared.idleSecondsRemaining = UINT32_MAX;
            motorShared.idleState = IDLE_STATE_OFF;
            if (valid) {
                motorShared.currentPsi = reading.pressurePsi;
                motorShared.rawPressure = reading.rawValue;
                motorShared.smoothedPsi = smoothedPressure;
            } else {
                motorShared.rawPressure = 0;
            }
            portEXIT_CRITICAL(&motorShared.mutex);
        } else {
            // Sensor error - disable motor for safety
            setMotorSpeed(0);
            lastSpeed = 0;
            pidReset(&motorPid);
            idleState = IDLE_STATE_OFF;
            idleCounter = 0;
            idleStableCounter = 0;
            
            portENTER_CRITICAL(&motorShared.mutex);
            motorShared.motorSpeed = 0;
            motorShared.pressureValid = false;
            motorShared.rawPressure = 0;
            motorShared.idleSecondsRemaining = UINT32_MAX;
            motorShared.idleState = IDLE_STATE_OFF;
            portEXIT_CRITICAL(&motorShared.mutex);
        }
        
        // Calculate loop timing
        uint32_t loopTime = micros() - loopStartTime;
        
        portENTER_CRITICAL(&motorShared.mutex);
        motorShared.loopCount++;
        motorShared.loopTimeUs = loopTime;
        if (loopTime > motorShared.maxLoopTimeUs) {
            motorShared.maxLoopTimeUs = loopTime;
        }
        portEXIT_CRITICAL(&motorShared.mutex);
        
        // Calculate sleep time to maintain target loop interval
        uint32_t elapsed = micros() - loopStartTime;
        if (elapsed < MOTOR_LOOP_INTERVAL_US) {
            delayMicroseconds(MOTOR_LOOP_INTERVAL_US - elapsed);
        }
        
        lastLoopTime = loopStartTime;
    }
}

bool dualCoreMotorInit(void) {
    Serial.println("Dual-Core Motor Control: Initializing...");
    
    // Create motor control task on Core 0
    BaseType_t result = xTaskCreatePinnedToCore(
        motorControlTask,           // Task function
        "MotorControl",             // Task name
        MOTOR_TASK_STACK_SIZE,      // Stack size
        NULL,                       // Task parameter
        MOTOR_TASK_PRIORITY,        // Priority (higher = more important)
        &motorTaskHandle,           // Task handle
        MOTOR_CONTROL_CORE          // Core to run on (Core 0)
    );
    
    if (result != pdPASS) {
        Serial.println("ERROR: Failed to create motor control task!");
        return false;
    }
    
    Serial.printf("Motor control task created on Core %d\n", MOTOR_CONTROL_CORE);
    Serial.printf("  Stack size: %d bytes\n", MOTOR_TASK_STACK_SIZE);
    Serial.printf("  Priority: %d\n", MOTOR_TASK_PRIORITY);
    Serial.printf("  Target loop interval: %d us (%d Hz)\n", 
                  MOTOR_LOOP_INTERVAL_US, 1000000 / MOTOR_LOOP_INTERVAL_US);
    
    return true;
}

void setTargetPressureSafe(float psi) {
    portENTER_CRITICAL(&motorShared.mutex);
    motorShared.targetPsi = psi;
    portEXIT_CRITICAL(&motorShared.mutex);
}

float getCurrentPressureSafe(void) {
    float psi;
    portENTER_CRITICAL(&motorShared.mutex);
    psi = motorShared.smoothedPsi;
    portEXIT_CRITICAL(&motorShared.mutex);
    return psi;
}

void getRawPressureSafe(float *pressurePsi, int32_t *rawValue, bool *valid) {
    if (pressurePsi == NULL || rawValue == NULL || valid == NULL) {
        return;
    }
    portENTER_CRITICAL(&motorShared.mutex);
    *pressurePsi = motorShared.currentPsi;
    *rawValue = motorShared.rawPressure;
    *valid = motorShared.pressureValid;
    portEXIT_CRITICAL(&motorShared.mutex);
}

uint16_t getMotorSpeedSafe(void) {
    uint16_t speed;
    portENTER_CRITICAL(&motorShared.mutex);
    speed = motorShared.motorSpeed;
    portEXIT_CRITICAL(&motorShared.mutex);
    return speed;
}

void getMotorLoopStats(uint32_t *avgUs, uint32_t *maxUs) {
    portENTER_CRITICAL(&motorShared.mutex);
    *avgUs = motorShared.loopTimeUs;
    *maxUs = motorShared.maxLoopTimeUs;
    portEXIT_CRITICAL(&motorShared.mutex);
}

void requestPidReset(void) {
    portENTER_CRITICAL(&motorShared.mutex);
    motorShared.pidResetRequest = true;
    portEXIT_CRITICAL(&motorShared.mutex);
}

void setMotorEnabledSafe(bool enable) {
    portENTER_CRITICAL(&motorShared.mutex);
    motorShared.motorEnabled = enable;
    portEXIT_CRITICAL(&motorShared.mutex);
    enableMotor(enable);
}

void setPidGainsSafe(float kp, float ki, float kd) {
    portENTER_CRITICAL(&motorShared.mutex);
    motorShared.pidKp = kp;
    motorShared.pidKi = ki;
    motorShared.pidKd = kd;
    motorShared.pidGainsChanged = true;
    portEXIT_CRITICAL(&motorShared.mutex);
}

void getPidGainsSafe(float *kp, float *ki, float *kd) {
    portENTER_CRITICAL(&motorShared.mutex);
    *kp = motorShared.pidKp;
    *ki = motorShared.pidKi;
    *kd = motorShared.pidKd;
    portEXIT_CRITICAL(&motorShared.mutex);
}

void setIdleEntryDeviationSafe(float deviationPsi) {
    portENTER_CRITICAL(&motorShared.mutex);
    motorShared.idleEntryDeviationPsi = deviationPsi;
    portEXIT_CRITICAL(&motorShared.mutex);
}

float getIdleEntryDeviationSafe(void) {
    float deviationPsi;
    portENTER_CRITICAL(&motorShared.mutex);
    deviationPsi = motorShared.idleEntryDeviationPsi;
    portEXIT_CRITICAL(&motorShared.mutex);
    return deviationPsi;
}

void setIdleEntrySecondsSafe(uint16_t seconds) {
    portENTER_CRITICAL(&motorShared.mutex);
    motorShared.idleEntrySeconds = seconds;
    portEXIT_CRITICAL(&motorShared.mutex);
}

void requestIdleExitSafe(void) {
    portENTER_CRITICAL(&motorShared.mutex);
    motorShared.idleExitRequest = true;
    portEXIT_CRITICAL(&motorShared.mutex);
}

uint16_t getIdleEntrySecondsSafe(void) {
    uint16_t seconds;
    portENTER_CRITICAL(&motorShared.mutex);
    seconds = motorShared.idleEntrySeconds;
    portEXIT_CRITICAL(&motorShared.mutex);
    return seconds;
}
