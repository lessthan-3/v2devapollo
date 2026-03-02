/**
 * @file motor_control.cpp
 * @brief Motor control implementation for triac-based AC motor speed control
 */

#include "motor_control.h"
#include <math.h>

// Global motor control state
MotorControlState motorState = {
    .zeroCrossingCount = 0,
    .lastZcTime = 0,
    .zcDetected = false,
    .motorSpeed = 0,
    .motorPwm = 0,
    .maxDelay = MAXDELAY_60HZ,
    .acFrequency = AC_FREQ_UNKNOWN,
    .motorEnabled = false
};

// Hardware timer handle
static hw_timer_t *triacTimer = NULL;

#ifdef SIMULATE_AC_60HZ
static hw_timer_t *zcSimTimer = NULL;
#endif

// Debug reporting variables
static unsigned long lastDebugReport = 0;
static uint32_t lastReportedCount = 0;

#ifdef TRIAC_DEBUG_SERIAL
static volatile uint32_t triacFireCount = 0;
static volatile uint32_t triacTimerArmCount = 0;
static volatile uint32_t triacTimerSkipCount = 0;
static uint32_t lastReportedTriacFireCount = 0;
static uint32_t lastReportedTriacArmCount = 0;
static uint32_t lastReportedTriacSkipCount = 0;
#endif

// Frequency detection variables
static volatile uint32_t zcTimestamps[3] = {0, 0, 0};
static volatile uint8_t zcTimestampIndex = 0;

/**
 * @brief Zero crossing interrupt service routine
 * 
 * Called on rising edge of zero crossing signal
 */
void IRAM_ATTR zeroCrossingISR() {
    uint32_t now = micros();
    
    // Store timestamp for frequency detection
    if (zcTimestampIndex < 3) {
        zcTimestamps[zcTimestampIndex++] = now;
    }
    
    // Increment zero crossing counter
    motorState.zeroCrossingCount++;
    motorState.lastZcTime = now;
    motorState.zcDetected = true;
    
    // If motor is enabled and speed > 0, start the delay timer
    if (motorState.motorEnabled && motorState.motorSpeed > 0) {
        if (triacTimer != NULL && motorState.motorPwm < motorState.maxDelay) {
            // Reset and start timer with calculated delay
            timerWrite(triacTimer, 0);
            timerAlarmWrite(triacTimer, motorState.motorPwm, false);
            timerAlarmEnable(triacTimer);
#ifdef TRIAC_DEBUG_SERIAL
            triacTimerArmCount++;
#endif
        } else {
#ifdef TRIAC_DEBUG_SERIAL
            triacTimerSkipCount++;
#endif
        }
    }
}

/**
 * @brief Timer interrupt service routine for triac firing
 * 
 * Called when delay timer expires - time to fire the triac
 */
void IRAM_ATTR triacTimerISR() {
    // if (!motorState.motorEnabled || motorState.motorSpeed == 0) {
    //     timerAlarmDisable(triacTimer);
    //     return;
    // }
    // Fire the triac with a short pulse
    digitalWrite(TRIAC_GATE_PIN, HIGH);
    delayMicroseconds(TRIAC_PULSE_US);
    digitalWrite(TRIAC_GATE_PIN, LOW);
#ifdef TRIAC_DEBUG_SERIAL
    triacFireCount++;
#endif
    
    // Disable the alarm until next zero crossing
    timerAlarmDisable(triacTimer);
}

#ifdef SIMULATE_AC_60HZ
/**
 * @brief Simulated zero crossing timer ISR (60Hz)
 */
void IRAM_ATTR zcSimTimerISR() {
    zeroCrossingISR();
}
#endif

bool motorControlInit() {
    Serial.println("Motor Control: Initializing...");
    
    // Configure triac gate pin as output
    pinMode(TRIAC_GATE_PIN, OUTPUT);
    digitalWrite(TRIAC_GATE_PIN, LOW);
    Serial.printf("  Triac gate pin: IO%d (output)\n", TRIAC_GATE_PIN);
    
#ifdef SIMULATE_AC_60HZ
    Serial.println("  Simulating AC zero crossings at 60Hz");
    motorState.acFrequency = AC_FREQ_60HZ;
    motorState.maxDelay = MAXDELAY_60HZ;

    // Configure simulated zero crossing timer (1us resolution)
    zcSimTimer = timerBegin(1, 80, true);
    if (zcSimTimer == NULL) {
        Serial.println("  ERROR: Failed to initialize ZC simulation timer!");
        return false;
    }
    timerAttachInterrupt(zcSimTimer, &zcSimTimerISR, true);
    timerAlarmWrite(zcSimTimer, SIMULATED_ZC_HALF_CYCLE_US, true);
    timerAlarmEnable(zcSimTimer);
    Serial.printf("  ZC simulation period: %u us\n", SIMULATED_ZC_HALF_CYCLE_US);
#else
    // Configure zero crossing detection pin with internal pullup
    pinMode(ZERO_CROSSING_PIN, INPUT_PULLUP);
    Serial.printf("  Zero crossing pin: IO%d (input)\n", ZERO_CROSSING_PIN);
    
    // Attach interrupt for zero crossing detection (falling edge)
    attachInterrupt(digitalPinToInterrupt(ZERO_CROSSING_PIN), zeroCrossingISR, FALLING);
    Serial.println("  Zero crossing interrupt attached (FALLING edge)");
#endif
    
    // Configure hardware timer for triac firing delay
    // Timer 0, prescaler 80 (1 tick = 1 microsecond at 80MHz APB clock)
    triacTimer = timerBegin(0, 80, true);  // Timer 0, prescaler 80, count up
    if (triacTimer == NULL) {
        Serial.println("  ERROR: Failed to initialize timer!");
        return false;
    }
    
    // Attach timer interrupt (edge-triggered)
    timerAttachInterrupt(triacTimer, &triacTimerISR, true);
    Serial.println("  Triac timer initialized (1us resolution)");
    
    // Initialize state
    motorState.zeroCrossingCount = 0;
    motorState.motorSpeed = 0;
    motorState.motorPwm = motorState.maxDelay;
    motorState.motorEnabled = false;
    
    // Reset debug reporting
    lastDebugReport = millis();
    lastReportedCount = 0;
    
    Serial.println("Motor Control: Initialization complete");
    return true;
     
}

void setMotorSpeed(uint8_t speed) {
    // Clamp speed to 0-100
    if (speed > 100) {
        speed = 100;
    }
    
    motorState.motorSpeed = speed;
    
    // Convert speed to delay value
    // Speed 0 = max delay (no firing)
    // Speed 100 = min delay (full power)
    if (speed == 0) {
        motorState.motorPwm = motorState.maxDelay;
    } else {
        uint16_t speedDelay = (uint16_t)speed * motorState.maxDelay / 100;
        motorState.motorPwm = motorState.maxDelay - speedDelay;
        
        // Ensure minimum delay
        if (motorState.motorPwm < MINDELAY) {
            motorState.motorPwm = MINDELAY;
        }
    }
}

void enableMotor(bool enable) {
    motorState.motorEnabled = enable;
    
    if (!enable) {
        // Ensure triac is off when motor is disabled
        digitalWrite(TRIAC_GATE_PIN, LOW);
        if (triacTimer != NULL) {
            timerStop(triacTimer);
        }
    } else if (triacTimer != NULL) {
        timerStart(triacTimer);
    }
    
    Serial.printf("Motor Control: Motor %s\n", enable ? "ENABLED" : "DISABLED");
}

uint32_t getZeroCrossingCount() {
    return motorState.zeroCrossingCount;
}

void resetZeroCrossingCount() {
    noInterrupts();
    motorState.zeroCrossingCount = 0;
    interrupts();
    lastReportedCount = 0;
    Serial.println("Motor Control: Zero crossing count reset");
}

AcFrequency detectAcFrequency() {
    Serial.println("Motor Control: Detecting AC frequency...");

#ifdef SIMULATE_AC_60HZ
    motorState.acFrequency = AC_FREQ_60HZ;
    motorState.maxDelay = MAXDELAY_60HZ;
    Serial.println("  Simulated: 60Hz AC");
    return motorState.acFrequency;
#endif
    
    // Reset timestamp collection
    noInterrupts();
    zcTimestampIndex = 0;
    zcTimestamps[0] = 0;
    zcTimestamps[1] = 0;
    zcTimestamps[2] = 0;
    interrupts();
    
    // Wait for 3 zero crossings (up to 500ms timeout)
    unsigned long startTime = millis();
    while (zcTimestampIndex < 3 && (millis() - startTime) < 500) {
        delay(1);
    }
    
    if (zcTimestampIndex < 3) {
        Serial.println("  ERROR: Timeout waiting for zero crossings");
        Serial.println("  (Is the AC power connected?)");
        motorState.acFrequency = AC_FREQ_UNKNOWN;
        return AC_FREQ_UNKNOWN;
    }
    
    // Calculate average period between zero crossings
    uint32_t period1 = zcTimestamps[1] - zcTimestamps[0];
    uint32_t period2 = zcTimestamps[2] - zcTimestamps[1];
    uint32_t avgPeriod = (period1 + period2) / 2;
    
    Serial.printf("  ZC periods: %lu us, %lu us (avg: %lu us)\n", period1, period2, avgPeriod);
    
    // 60Hz half-cycle: ~8333us
    // 50Hz half-cycle: ~10000us
    // Use midpoint ~9166us as threshold
    
    if (avgPeriod > 9200) {
        motorState.acFrequency = AC_FREQ_50HZ;
        motorState.maxDelay = MAXDELAY_50HZ;
        Serial.println("  Detected: 50Hz AC");
    } else if (avgPeriod > 7500) {
        motorState.acFrequency = AC_FREQ_60HZ;
        motorState.maxDelay = MAXDELAY_60HZ;
        Serial.println("  Detected: 60Hz AC");
    } else {
        motorState.acFrequency = AC_FREQ_UNKNOWN;
        Serial.printf("  WARNING: Unusual frequency (period: %lu us)\n", avgPeriod);
    }
    
    return motorState.acFrequency;
}

void motorControlDebugReport() {
    unsigned long now = millis();
    
    if (now - lastDebugReport >= DEBUG_REPORT_INTERVAL_MS) {
        uint32_t currentCount = motorState.zeroCrossingCount;
        uint32_t countDelta = currentCount - lastReportedCount;
        
        // Calculate frequency from count delta
        float measuredFreq = (float)countDelta / (DEBUG_REPORT_INTERVAL_MS / 1000.0f) / 2.0f;  // Divide by 2 because 2 ZC per cycle
        
        Serial.println("--- Motor Control Debug ---");
        Serial.printf("  Zero Crossings: %lu (total), +%lu (last %d sec)\n", 
                      currentCount, countDelta, DEBUG_REPORT_INTERVAL_MS / 1000);
        Serial.printf("  Measured AC Freq: %.1f Hz\n", measuredFreq);
        Serial.printf("  Motor Speed: %u%%, PWM Delay: %u us\n", 
                      motorState.motorSpeed, motorState.motorPwm);
        Serial.printf("  Motor Enabled: %s, Max Delay: %u us\n",
                      motorState.motorEnabled ? "YES" : "NO", motorState.maxDelay);
    #ifdef TRIAC_DEBUG_SERIAL
        uint32_t fireCount = triacFireCount;
        uint32_t armCount = triacTimerArmCount;
        uint32_t skipCount = triacTimerSkipCount;
        Serial.printf("  Triac Fires: %lu (total), +%lu\n",
                  fireCount, fireCount - lastReportedTriacFireCount);
        Serial.printf("  Triac Arms: %lu (total), +%lu\n",
                  armCount, armCount - lastReportedTriacArmCount);
        Serial.printf("  Triac Skips: %lu (total), +%lu\n",
                  skipCount, skipCount - lastReportedTriacSkipCount);
        lastReportedTriacFireCount = fireCount;
        lastReportedTriacArmCount = armCount;
        lastReportedTriacSkipCount = skipCount;
    #endif
        Serial.println("---------------------------");
        
        lastReportedCount = currentCount;
        lastDebugReport = now;
    }
}

void fireTriac() {
    // Manual triac firing (for testing)
    digitalWrite(TRIAC_GATE_PIN, HIGH);
    delayMicroseconds(TRIAC_PULSE_US);
    digitalWrite(TRIAC_GATE_PIN, LOW);
}

// ============================================================================
// PID-Based Pressure Control Implementation
// ============================================================================

// Global PID controller instance
static PidController pressurePid;

void pressurePidInit() {
    Serial.println("Pressure PID: Initializing...");
    
    // Initialize PID with default gains
    // The pidInit function will load saved params from NVS if available
    pidInit(&pressurePid, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
    
    Serial.println("Pressure PID: Initialization complete");
}

uint8_t updateMotorFromPid(float currentPressurePsi) {
    // Don't run PID if motor is disabled
    if (!motorState.motorEnabled) {
        return 0;
    }
    
    // Calculate PID output (motor speed adjustment)
    float pidAdjustment = pidCalculate(&pressurePid, currentPressurePsi);

    // Apply adjustment to current speed and clamp to 20-100
    int targetSpeed = (int)motorState.motorSpeed + (int)lroundf(pidAdjustment);
    targetSpeed = constrain(targetSpeed, 20, 100);
    uint8_t speed = (uint8_t)targetSpeed;

    // Set motor speed
    setMotorSpeed(speed);
    
    return speed;
}

void setTargetPressure(float targetPsi) {
    pidSetSetpoint(&pressurePid, targetPsi);
}

float getTargetPressure() {
    return pidGetSetpoint(&pressurePid);
}

void resetPressurePid() {
    pidReset(&pressurePid);
    Serial.println("Pressure PID: Reset");
}

PidController* getPressurePid() {
    return &pressurePid;
}

void setPidLearning(bool enable) {
    pidEnableLearning(&pressurePid, enable);
}

bool isPidLearningEnabled() {
    return pidIsLearningEnabled(&pressurePid);
}
