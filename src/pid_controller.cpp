/**
 * @file pid_controller.cpp
 * @brief PID Controller Implementation for Manual Tuning
 */

#include "pid_controller.h"

// NVS Preferences instance for storing parameters
static Preferences pidPrefs;

/**
 * @brief Constrain a float value between min and max
 */
static float constrainFloat(float value, float minVal, float maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

void pidInit(PidController *pid, float kp, float ki, float kd) {
    Serial.println("PID Controller: Initializing...");
    
    // Set initial gains
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    // Initialize state
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->prevOutput = 0.0f;
    pid->dt = 0.1f;  // Default 100ms update rate
    pid->lastUpdateTime = 0;
    
    // Set limits
    pid->outputMin = PID_OUTPUT_MIN;
    pid->outputMax = PID_OUTPUT_MAX;
    pid->integralMax = PID_INTEGRAL_MAX;
    pid->deadband = 0.1f;  // 0.1 PSI deadband
    
    pid->hasStoredParams = false;
    
    // Try to load saved parameters from NVS
    if (pidLoadGains(pid)) {
        Serial.printf("  Loaded saved gains: Kp=%.1f, Ki=%.1f, Kd=%.1f\n",
                      pid->kp, pid->ki, pid->kd);
    } else {
        Serial.printf("  Using default gains: Kp=%.1f, Ki=%.1f, Kd=%.1f\n",
                      pid->kp, pid->ki, pid->kd);
    }
    
    Serial.println("PID Controller: Initialization complete");
}

void pidReset(PidController *pid) {
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->prevOutput = 0.0f;
    pid->lastUpdateTime = 0;
    // Note: No Serial output - this function is called from Core 0
}

float pidCalculate(PidController *pid, float currentValue) {
    unsigned long now = millis();
    
    // Calculate time delta
    if (pid->lastUpdateTime > 0) {
        pid->dt = (now - pid->lastUpdateTime) / 1000.0f;  // Convert to seconds
        
        // Sanity check on dt
        if (pid->dt <= 0.0f || pid->dt > 1.0f) {
            pid->dt = 0.1f;  // Default to 100ms if something went wrong
        }
    }
    pid->lastUpdateTime = now;
    
    // Calculate error
    float error = pid->setpoint - currentValue;
    
    // Apply deadband for stability
    if (fabsf(error) < pid->deadband) {
        // Within deadband - maintain previous output
        return pid->prevOutput;
    }
    
    // Calculate proportional term
    float proportional = pid->kp * error;
    
    // Calculate derivative term (before updating integral to check rate limiting)
    float derivative = 0.0f;
    if (pid->dt > 0.0f) {
        derivative = pid->kd * (error - pid->prevError) / pid->dt;
    }
    
    // Calculate what output would be without integral
    float rawOutputNoIntegral = proportional + derivative;
    
    // Anti-windup: Only accumulate integral if output isn't saturated
    // or if error is reducing (opposite sign to integral)
    bool atLimit = (rawOutputNoIntegral >= pid->outputMax) || 
                   (rawOutputNoIntegral <= pid->outputMin);
    bool errorReducingIntegral = (error * pid->integral < 0);
    
    if (!atLimit || errorReducingIntegral) {
        pid->integral += error * pid->dt;
        
        // Apply integral limits
        pid->integral = constrainFloat(pid->integral, -pid->integralMax, pid->integralMax);
    }
    
    // Calculate integral term
    float integral = pid->ki * pid->integral;
    
    // Store error for next derivative calculation
    pid->prevError = error;
    
    // Calculate raw output with base offset to ensure minimum motor speed
    float output = proportional + integral + derivative + 30.0f;
    
    // Apply output limits
    output = constrainFloat(output, pid->outputMin, pid->outputMax);
    
    // Store for next iteration
    pid->prevOutput = output;
    
    return output;
}

void pidSetSetpoint(PidController *pid, float setpoint) {
    // Check if setpoint changed significantly
    if (fabsf(setpoint - pid->setpoint) > 0.5f) {
        Serial.printf("PID: Setpoint changed from %.2f to %.2f\n", pid->setpoint, setpoint);
        // Optionally reset integral on large setpoint changes to prevent windup
        // pid->integral = 0.0f;
    }
    
    pid->setpoint = setpoint;
}

float pidGetSetpoint(PidController *pid) {
    return pid->setpoint;
}

bool pidSaveGains(PidController *pid) {
    if (pidPrefs.begin(NVS_NAMESPACE_PID, false)) {  // Read-write
        pidPrefs.putFloat(NVS_KEY_KP, pid->kp);
        pidPrefs.putFloat(NVS_KEY_KI, pid->ki);
        pidPrefs.putFloat(NVS_KEY_KD, pid->kd);
        pidPrefs.putBool(NVS_KEY_VALID, true);
        pidPrefs.end();
        
        pid->hasStoredParams = true;
        Serial.printf("PID: Saved gains to NVS (Kp=%.1f, Ki=%.1f, Kd=%.1f)\n",
                      pid->kp, pid->ki, pid->kd);
        return true;
    }
    
    Serial.println("PID: ERROR - Failed to save gains to NVS");
    return false;
}

bool pidLoadGains(PidController *pid) {
    if (pidPrefs.begin(NVS_NAMESPACE_PID, true)) {  // Read-only
        bool valid = pidPrefs.getBool(NVS_KEY_VALID, false);
        
        if (valid) {
            float kp = pidPrefs.getFloat(NVS_KEY_KP, PID_KP_DEFAULT);
            float ki = pidPrefs.getFloat(NVS_KEY_KI, PID_KI_DEFAULT);
            float kd = pidPrefs.getFloat(NVS_KEY_KD, PID_KD_DEFAULT);
            pidPrefs.end();
            
            // Validate loaded values are within limits
            if (kp >= PID_KP_MIN && kp <= PID_KP_MAX &&
                ki >= PID_KI_MIN && ki <= PID_KI_MAX &&
                kd >= PID_KD_MIN && kd <= PID_KD_MAX) {
                pid->kp = kp;
                pid->ki = ki;
                pid->kd = kd;
                pid->hasStoredParams = true;
                return true;
            }
        }
        pidPrefs.end();
    }
    
    return false;
}

void pidMarkAsLearned(PidController *pid, bool saveNow) {
    pid->hasStoredParams = true;
    Serial.println("PID: Parameters marked for storage");
    
    if (saveNow) {
        pidSaveGains(pid);
    }
}

void pidResetToDefaults(PidController *pid) {
    // Reset gains to defaults
    pid->kp = PID_KP_DEFAULT;
    pid->ki = PID_KI_DEFAULT;
    pid->kd = PID_KD_DEFAULT;
    
    // Clear NVS
    if (pidPrefs.begin(NVS_NAMESPACE_PID, false)) {
        pidPrefs.clear();
        pidPrefs.end();
    }
    
    pid->hasStoredParams = false;
    
    // Reset controller state
    pidReset(pid);
    
    Serial.printf("PID: Reset to defaults (Kp=%.1f, Ki=%.1f, Kd=%.1f)\n",
                  pid->kp, pid->ki, pid->kd);
}

void pidGetGains(PidController *pid, float *kp, float *ki, float *kd) {
    if (kp) *kp = pid->kp;
    if (ki) *ki = pid->ki;
    if (kd) *kd = pid->kd;
}

void pidSetGains(PidController *pid, float kp, float ki, float kd) {
    pid->kp = constrainFloat(kp, PID_KP_MIN, PID_KP_MAX);
    pid->ki = constrainFloat(ki, PID_KI_MIN, PID_KI_MAX);
    pid->kd = constrainFloat(kd, PID_KD_MIN, PID_KD_MAX);
    // Note: No Serial output - this function is called from Core 0
}

void pidDebugPrint(PidController *pid) {
    Serial.println("--- PID Controller Status ---");
    Serial.printf("  Setpoint: %.2f, Output: %.2f\n", pid->setpoint, pid->prevOutput);
    Serial.printf("  Gains: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", pid->kp, pid->ki, pid->kd);
    Serial.printf("  Integral: %.3f, Prev Error: %.3f\n", pid->integral, pid->prevError);
    Serial.printf("  Stored in NVS: %s\n", pid->hasStoredParams ? "YES" : "NO");
    Serial.println("-----------------------------");
}

// Legacy compatibility stubs
void pidEnableLearning(PidController *pid, bool enable) {
    (void)pid;
    (void)enable;
    // No-op - learning removed
}

bool pidIsLearningEnabled(PidController *pid) {
    (void)pid;
    return false;  // Learning removed
}
