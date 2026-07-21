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
    .targetPsi = TARGET_PSI_DEFAULT,
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
    .spikeMultiplier = 1.0f,
    .ppHoldSpeed = PP_HOLD_SPEED_DEFAULT,
    .ppSpeedSaveRequest = false,
    .mutex = portMUX_INITIALIZER_UNLOCKED
};

// Task handle for motor control task
static TaskHandle_t motorTaskHandle = NULL;

TaskHandle_t getMotorTaskHandle(void) { return motorTaskHandle; }

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

    // Read initial ppHoldSpeed from shared data (self-adjusting PowerPause speed)
    portENTER_CRITICAL(&motorShared.mutex);
    uint16_t ppHoldSpeed = motorShared.ppHoldSpeed;
    portEXIT_CRITICAL(&motorShared.mutex);

    IdleState idleState = IDLE_STATE_OFF;
    uint32_t idleCounter = 0;
    uint32_t idleStableCounter = 0;
    float maxPressureRecorded = 0.0f;  // tracks peak pressure for MAX-mode power pause

    // --- Motor-speed steady-state detection ---
    // Rolling ring buffer: track the last N motor speed samples to detect
    // when the output has settled (max-min spread within IDLE_SPEED_STABLE_SPREAD).
    uint16_t speedBuf[IDLE_SPEED_STABLE_WINDOW] = {};
    uint8_t  speedBufIdx  = 0;
    bool     speedBufFull = false;
    bool     motorSteadyState = false;          // true once ring buffer declares stable
    bool     steadyStateLogged = false;         // guard: only log once per steady-state entry
    bool     pidSaturatedLogged = false;        // guard: only log once per PID-saturated entry

    // --- Pressure ring buffer for PowerPause ramp stability detection ---
    float    pressBuf[PP_PRESSURE_STABLE_WINDOW] = {};
    uint8_t  pressBufIdx  = 0;
    bool     pressBufFull = false;
    float    settledPsi   = 0.0f;    // pressure recorded at stability (for self-tuning on HOLD exit)
    const uint32_t ppPressureStableLoops = (uint32_t)(PP_PRESSURE_STABLE_SECONDS * (1000000.0f / MOTOR_LOOP_INTERVAL_US));

    // --- Ramp-phase timeout and trigger-pull exit ---
    uint32_t       idleRampLoopCount    = 0;
    const uint32_t idleRampLockoutLoops = (uint32_t)(PP_RAMP_LOCKOUT_SECONDS * (1000000.0f / MOTOR_LOOP_INTERVAL_US));
    const uint32_t idleRampTimeoutLoops = (IDLE_RAMP_TIMEOUT_SECONDS * 1000000UL) / MOTOR_LOOP_INTERVAL_US;

    // --- HOLD-phase lockout (avoid instant bounce on entry) ---
    uint32_t       holdLoopCount        = 0;
    const uint32_t ppHoldLockoutLoops   = (uint32_t)(PP_HOLD_LOCKOUT_SECONDS * (1000000.0f / MOTOR_LOOP_INTERVAL_US));
    
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
        float spikeMultiplier = 1.0f;
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
        spikeMultiplier = motorShared.spikeMultiplier;
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
                idleState           = IDLE_STATE_OFF;
                idleCounter         = 0;
                idleStableCounter   = 0;
                idleRampLoopCount   = 0;
                holdLoopCount       = 0;
                maxPressureRecorded = 0.0f;
                motorSteadyState    = false;
                steadyStateLogged   = false;
                speedBufFull        = false;
                speedBufIdx         = 0;
                pressBufFull        = false;
                pressBufIdx         = 0;
                settledPsi          = 0.0f;
                pidReset(&motorPid);
            }
            
            bool isMaxMode = (target >= MAX_PSI_THRESHOLD);

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
                idleRampLoopCount++;

                // Apply fixed PowerPause hold speed — no PID
                setMotorSpeed(ppHoldSpeed);
                speed = ppHoldSpeed;
                pidOut = (float)ppHoldSpeed;
                lastSpeed = speed;

                // Push pressure into ring buffer for stability detection
                pressBuf[pressBufIdx] = smoothedPressure;
                pressBufIdx = (pressBufIdx + 1) % PP_PRESSURE_STABLE_WINDOW;
                if (pressBufIdx == 0) pressBufFull = true;

                // --- Trigger-pull exit: after lockout, check for pressure drop ---
                bool rampLoadExit = false;
                if (idleRampLoopCount > idleRampLockoutLoops) {
                    if (smoothedPressure < (IDLE_TARGET_PSI - IDLE_EXIT_DROP_PSI)) {
                        rampLoadExit = true;
                        Serial.printf("[PowerPause] RAMP exit by trigger: psi=%.2f\n", smoothedPressure);
                    }
                }

                if (rampLoadExit) {
                    idleState           = IDLE_STATE_OFF;
                    idleCounter         = 0;
                    idleStableCounter   = 0;
                    idleRampLoopCount   = 0;
                    maxPressureRecorded = 0.0f;
                    motorSteadyState    = false;
                    steadyStateLogged   = false;
                    pidSaturatedLogged  = false;
                    speedBufFull        = false;
                    speedBufIdx         = 0;
                    pressBufFull        = false;
                    pressBufIdx         = 0;
                    settledPsi          = 0.0f;
                    pidReset(&motorPid);
                } else {
                    // --- Pressure stability detection ---
                    bool pressureNowStable = false;
                    float pressBufMean = 0.0f;
                    if (pressBufFull) {
                        float minP = pressBuf[0], maxP = pressBuf[0];
                        float sum  = 0.0f;
                        for (uint8_t i = 0; i < PP_PRESSURE_STABLE_WINDOW; i++) {
                            if (pressBuf[i] < minP) minP = pressBuf[i];
                            if (pressBuf[i] > maxP) maxP = pressBuf[i];
                            sum += pressBuf[i];
                        }
                        pressBufMean      = sum / PP_PRESSURE_STABLE_WINDOW;
                        pressureNowStable = ((maxP - minP) <= PP_PRESSURE_STABLE_BAND_PSI);
                    }

                    if (pressureNowStable) {
                        if (idleStableCounter < ppPressureStableLoops) {
                            idleStableCounter++;
                        }
                    } else {
                        idleStableCounter = 0;
                    }

                    // Transition to HOLD when stable long enough, or on timeout
                    bool transitionToHold = false;
                    if (idleStableCounter >= ppPressureStableLoops && ppPressureStableLoops > 0) {
                        settledPsi = pressBufMean;
                        transitionToHold = true;
                        Serial.printf("[PowerPause] RAMP stable: psi=%.2f (speed=%u)\n",
                                      settledPsi, ppHoldSpeed);
                    } else if (idleRampLoopCount >= idleRampTimeoutLoops) {
                        settledPsi = smoothedPressure;
                        transitionToHold = true;
                        Serial.printf("[PowerPause] RAMP timeout: psi=%.2f (speed=%u)\n",
                                      settledPsi, ppHoldSpeed);
                    }

                    if (transitionToHold) {
                        idleState         = IDLE_STATE_HOLD;
                        idleRampLoopCount = 0;
                        idleStableCounter = 0;
                        holdLoopCount     = 0;
                        pressBufFull      = false;
                        pressBufIdx       = 0;
                    }
                }
            } else if (idleState == IDLE_STATE_HOLD) {
                setMotorSpeed(ppHoldSpeed);
                speed = ppHoldSpeed;
                pidOut = (float)ppHoldSpeed;
                lastSpeed = speed;
                holdLoopCount++;

                // Only check for exit after the hold lockout period to avoid
                // an instant bounce while pressure is still settling on entry.
                // Exit threshold is relative to settledPsi so the system stays
                // in HOLD even when settled pressure is below IDLE_TARGET_PSI.
                if (holdLoopCount > ppHoldLockoutLoops) {
                    bool holdExitByPressure = (smoothedPressure < (settledPsi - IDLE_EXIT_DROP_PSI));
                    if (holdExitByPressure) {
                        Serial.printf("[PowerPause] HOLD exit: psi=%.2f, settled=%.2f\n",
                                      smoothedPressure, settledPsi);

                        // Adjust ppHoldSpeed based on settled pressure vs. target band
                        uint16_t newSpeed = ppHoldSpeed;
                        if (settledPsi < PP_SETTLE_LOW_PSI &&
                            ppHoldSpeed + PP_HOLD_SPEED_STEP <= PP_HOLD_SPEED_MAX) {
                            // Pressure settled too low → need more power next time
                            newSpeed = ppHoldSpeed + PP_HOLD_SPEED_STEP;
                            Serial.printf("[PowerPause] Speed up: %u → %u (settled=%.2f < %.2f PSI)\n",
                                          ppHoldSpeed, newSpeed, settledPsi, PP_SETTLE_LOW_PSI);
                        } else if (settledPsi > PP_SETTLE_HIGH_PSI &&
                                   ppHoldSpeed >= (uint16_t)(PP_HOLD_SPEED_MIN + PP_HOLD_SPEED_STEP)) {
                            // Pressure settled too high → need less power next time
                            newSpeed = ppHoldSpeed - PP_HOLD_SPEED_STEP;
                            Serial.printf("[PowerPause] Speed down: %u → %u (settled=%.2f > %.2f PSI)\n",
                                          ppHoldSpeed, newSpeed, settledPsi, PP_SETTLE_HIGH_PSI);
                        }

                        if (newSpeed != ppHoldSpeed) {
                            ppHoldSpeed = newSpeed;
                            portENTER_CRITICAL(&motorShared.mutex);
                            motorShared.ppHoldSpeed        = ppHoldSpeed;
                            motorShared.ppSpeedSaveRequest = true;
                            portEXIT_CRITICAL(&motorShared.mutex);
                        }

                        // Reset to normal operation
                        idleState           = IDLE_STATE_OFF;
                        idleCounter         = 0;
                        idleStableCounter   = 0;
                        holdLoopCount       = 0;
                        motorSteadyState    = false;
                        steadyStateLogged   = false;
                        pidSaturatedLogged  = false;
                        speedBufFull        = false;
                        speedBufIdx         = 0;
                        pressBufFull        = false;
                        pressBufIdx         = 0;
                        settledPsi          = 0.0f;
                        pidReset(&motorPid);
                    }
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
                        idleState           = IDLE_STATE_PID_RAMP;
                        idleCounter         = 0;
                        idleStableCounter   = 0;
                        idleRampLoopCount   = 0;
                        maxPressureRecorded = 0.0f;
                        pressBufFull        = false;
                        pressBufIdx         = 0;
                        settledPsi          = 0.0f;
                        pidReset(&motorPid);
                    }
                } else {
                    // Normal PID mode
                    // Note: maxPressureRecorded is managed by the inner saturated-speed
                    // path below; do NOT reset it here so the peak accumulates correctly
                    // when the PID is saturated at 100%.  It is zeroed out by the inner
                    // logic whenever the motor is NOT in the saturated path.
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

                    // -----------------------------------------------------------------
                    // Motor-speed steady-state detection for power pause entry.
                    //
                    // Push the freshly-computed speed into a ring buffer and examine
                    // the max-min spread across the window.  When the spread is within
                    // IDLE_SPEED_STABLE_SPREAD the PID output has settled and we start
                    // counting toward power-pause entry.
                    //
                    // Spike detection uses the rolling mean of the buffer (not a stale
                    // one-time snapshot) so it tracks any gradual shift in the setpoint
                    // level without producing false positives.
                    //
                    // Spike threshold is linearly interpolated from target PSI:
                    //   3.0 PSI  →  IDLE_SPIKE_UNITS_AT_3PSI  (1 %)
                    //   8.5 PSI  →  IDLE_SPIKE_UNITS_AT_MAX   (3 %)
                    // -----------------------------------------------------------------
                    if (target > IDLE_TARGET_PSI) {
                        // --- Push into ring buffer ---
                        speedBuf[speedBufIdx] = speed;
                        speedBufIdx = (speedBufIdx + 1) % IDLE_SPEED_STABLE_WINDOW;
                        if (speedBufIdx == 0) speedBufFull = true;

                        // --- Evaluate buffer once it is full ---
                        bool nowSteady = false;
                        float bufMean  = 0.0f;
                        if (speedBufFull) {
                            uint16_t minS = speedBuf[0], maxS = speedBuf[0];
                            uint32_t sum  = 0;
                            for (uint8_t i = 0; i < IDLE_SPEED_STABLE_WINDOW; i++) {
                                if (speedBuf[i] < minS) minS = speedBuf[i];
                                if (speedBuf[i] > maxS) maxS = speedBuf[i];
                                sum += speedBuf[i];
                            }
                            bufMean   = (float)sum / IDLE_SPEED_STABLE_WINDOW;
                            nowSteady = ((maxS - minS) <= IDLE_SPEED_STABLE_SPREAD);
                        }

                        // --- PSI-scaled spike threshold ---
                        // Computed before the state-transition check so it can be used
                        // as the exit criterion once steady state is already established.
                        float spikeThreshold;
                        {
                            float ct = target;
                            if (ct < 3.0f)               ct = 3.0f;
                            if (ct > MAX_PSI_THRESHOLD)  ct = MAX_PSI_THRESHOLD;
                            float t = (ct - 3.0f) / (MAX_PSI_THRESHOLD - 3.0f);
                            spikeThreshold = IDLE_SPIKE_UNITS_AT_3PSI +
                                             t * (IDLE_SPIKE_UNITS_AT_MAX - IDLE_SPIKE_UNITS_AT_3PSI);
                            spikeThreshold *= spikeMultiplier;
                        }

                        // --- Steady-state transition logic ---
                        // Entry: buffer spread must be within IDLE_SPEED_STABLE_SPREAD.
                        // Exit:  once steady, only a spike exceeding spikeThreshold
                        //        (deviation of the live speed from the rolling mean)
                        //        breaks steady state — prevents normal PID micro-jitter
                        //        from repeatedly toggling the flag.
                        if (nowSteady && !motorSteadyState) {
                            motorSteadyState = true;
                            if (!steadyStateLogged) {
                                Serial.printf("[PowerPause] Steady state detected: speed=%u (mean=%.1f), target=%.1f PSI\n",
                                              speed, bufMean, target);
                                steadyStateLogged = true;
                            }
                        } else if (motorSteadyState && speedBufFull) {
                            // Already in steady state — only exit on a real spike
                            float deviation = fabsf((float)speed - bufMean);
                            if (deviation > spikeThreshold) {
                                Serial.printf("[PowerPause] Steady state lost: speed=%u, mean=%.1f, dev=%.1f (threshold=%.1f), target=%.1f PSI\n",
                                              speed, bufMean, deviation, spikeThreshold, target);
                                motorSteadyState     = false;
                                steadyStateLogged    = false;
                                pidSaturatedLogged   = false;
                            }
                        } else if (!motorSteadyState && !nowSteady) {
                            // Not yet in steady state and buffer is still unsettled — no-op,
                            // but clear logs so entry can fire cleanly next time.
                            steadyStateLogged  = false;
                            pidSaturatedLogged = false;
                        }

                        // --- Idle counter update ---
                        if (motorSteadyState) {
                            // Edge case: motor running near or at full power (mean close to 1000).
                            // When bufMean is within spikeThreshold of the ceiling, a real
                            // demand spike cannot be observed — the speed would need to exceed
                            // 1000 to register. Switch to the pressure-stability algorithm in
                            // this region so we don't falsely declare the system settled.
                            bool pidSaturated = (bufMean >= (1000.0f - spikeThreshold));

                            if (pidSaturated) {
                                // --- Saturated-speed path: mirror MAX-mode pressure logic ---
                                if (!pidSaturatedLogged) {
                                    Serial.printf("[PowerPause] Near/at 100%% speed (mean=%.1f, threshold=%.1f) — using pressure-stability algorithm (peak=%.2f PSI)\n",
                                                  bufMean, spikeThreshold, maxPressureRecorded);
                                    pidSaturatedLogged = true;
                                }

                                if (smoothedPressure > maxPressureRecorded) {
                                    maxPressureRecorded = smoothedPressure;
                                }

                                if (smoothedPressure >= maxPressureRecorded - MAX_PRESSURE_DEVIATION_PSI) {
                                    if (idleCounter < idleEntryLoops) {
                                        idleCounter += IDLE_LOOP_INCREMENT;
                                    }
                                } else if (idleCounter >= IDLE_ENTRY_DECREASE) {
                                    idleCounter -= IDLE_ENTRY_DECREASE;
                                } else {
                                    idleCounter = 0;
                                    maxPressureRecorded = smoothedPressure;
                                }
                            } else {
                                // Normal path: compare current speed against the rolling mean
                                maxPressureRecorded  = 0.0f;
                                pidSaturatedLogged   = false;
                                float deviation = fabsf((float)speed - bufMean);
                                if (deviation > spikeThreshold) {
                                    // Spike detected — penalise counter
                                    if (idleCounter > IDLE_ENTRY_DECREASE) {
                                        idleCounter -= IDLE_ENTRY_DECREASE;
                                    } else {
                                        idleCounter = 0;
                                    }
                                } else {
                                    // Settled — count toward power pause
                                    if (idleCounter < idleEntryLoops) {
                                        idleCounter += IDLE_LOOP_INCREMENT;
                                    }
                                }
                            }
                        } else {
                            // Not steady — drain the counter
                            maxPressureRecorded = 0.0f;
                            if (idleCounter > IDLE_ENTRY_DECREASE) {
                                idleCounter -= IDLE_ENTRY_DECREASE;
                            } else {
                                idleCounter = 0;
                            }
                        }

                        if (idleCounter >= idleEntryLoops && idleEntryLoops > 0) {
                            idleState           = IDLE_STATE_PID_RAMP;
                            idleCounter         = 0;
                            idleStableCounter   = 0;
                            idleRampLoopCount   = 0;
                            motorSteadyState    = false;
                            steadyStateLogged   = false;
                            pidSaturatedLogged  = false;
                            speedBufFull        = false;
                            speedBufIdx         = 0;
                            pressBufFull        = false;
                            pressBufIdx         = 0;
                            settledPsi          = 0.0f;
                            pidReset(&motorPid);
                        }
                    }
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
            idleState           = IDLE_STATE_OFF;
            idleCounter         = 0;
            idleStableCounter   = 0;
            idleRampLoopCount   = 0;
            holdLoopCount       = 0;
            maxPressureRecorded = 0.0f;
            motorSteadyState    = false;
            steadyStateLogged   = false;
            speedBufFull        = false;
            speedBufIdx         = 0;
            pressBufFull        = false;
            pressBufIdx         = 0;
            settledPsi          = 0.0f;

            portENTER_CRITICAL(&motorShared.mutex);
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
            idleState           = IDLE_STATE_OFF;
            idleCounter         = 0;
            idleStableCounter   = 0;
            idleRampLoopCount   = 0;
            holdLoopCount       = 0;
            motorSteadyState    = false;
            steadyStateLogged   = false;
            speedBufFull        = false;
            speedBufIdx         = 0;
            pressBufFull        = false;
            pressBufIdx         = 0;
            settledPsi          = 0.0f;

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

void setSpikeMultiplierSafe(float multiplier) {
    portENTER_CRITICAL(&motorShared.mutex);
    motorShared.spikeMultiplier = multiplier;
    portEXIT_CRITICAL(&motorShared.mutex);
}

float getSpikeMultiplierSafe(void) {
    float m;
    portENTER_CRITICAL(&motorShared.mutex);
    m = motorShared.spikeMultiplier;
    portEXIT_CRITICAL(&motorShared.mutex);
    return m;
}

void setPpHoldSpeedSafe(uint16_t speed) {
    portENTER_CRITICAL(&motorShared.mutex);
    motorShared.ppHoldSpeed = speed;
    portEXIT_CRITICAL(&motorShared.mutex);
}

uint16_t getPpHoldSpeedSafe(void) {
    uint16_t s;
    portENTER_CRITICAL(&motorShared.mutex);
    s = motorShared.ppHoldSpeed;
    portEXIT_CRITICAL(&motorShared.mutex);
    return s;
}
