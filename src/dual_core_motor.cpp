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

// Global shared data instance
MotorSharedData motorShared = {
    .targetPsi = 6.0f,
    .motorEnabled = false,
    .pidResetRequest = false,
    .pidGainsChanged = true,  // Start as true to load initial gains
    .pidKp = PID_KP_DEFAULT,
    .pidKi = PID_KI_DEFAULT,
    .pidKd = PID_KD_DEFAULT,
    .currentPsi = 0.0f,
    .smoothedPsi = 0.0f,
    .motorSpeed = 0,
    .pidOutput = 0.0f,
    .pressureValid = false,
    .loopCount = 0,
    .loopTimeUs = 0,
    .maxLoopTimeUs = 0,
    .mutex = portMUX_INITIALIZER_UNLOCKED
};

// Task handle for motor control task
static TaskHandle_t motorTaskHandle = NULL;

// Local PID controller for motor task
static PidController motorPid;

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
    const float smoothingAlpha = 0.3f;  // EMA smoothing factor
    
    // Main motor control loop
    while (true) {
        loopStartTime = micros();
        
        // Check for PID reset request or gains change
        bool resetRequested = false;
        bool gainsChanged = false;
        float newKp, newKi, newKd;
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
        portEXIT_CRITICAL(&motorShared.mutex);
        
        if (resetRequested) {
            pidReset(&motorPid);
        }
        
        if (gainsChanged) {
            pidSetGains(&motorPid, newKp, newKi, newKd);
            // Note: No Serial output in motor loop - can cause timing issues
        }
        
        // Get target pressure (atomic read)
        portENTER_CRITICAL(&motorShared.mutex);
        float target = motorShared.targetPsi;
        bool enabled = motorShared.motorEnabled;
        portEXIT_CRITICAL(&motorShared.mutex);
        
        // Update PID setpoint
        motorPid.setpoint = target;
        
        // Read pressure sensor
        PressureReading reading = pressureSensor.readPressure();
        
        uint8_t speed = 0;
        bool valid = reading.valid;
        
        if (valid && enabled) {
            // Exponential moving average smoothing
            smoothedPressure = (smoothingAlpha * reading.pressurePsi) + 
                               ((1.0f - smoothingAlpha) * smoothedPressure);
            
            // Calculate PID output
            float pidOut = pidCalculate(&motorPid, smoothedPressure);
            
            // Convert PID output (0-100) to motor speed
            speed = (uint8_t)constrain(pidOut, 0.0f, 100.0f);
            
            // Update motor speed
            setMotorSpeed(speed);
            //setMotorSpeed(50);  // TESTING - REMOVE
            
            // Store results (atomic write)
            portENTER_CRITICAL(&motorShared.mutex);
            motorShared.currentPsi = reading.pressurePsi;
            motorShared.smoothedPsi = smoothedPressure;
            motorShared.motorSpeed = speed;
            motorShared.pidOutput = pidOut;
            motorShared.pressureValid = true;
            portEXIT_CRITICAL(&motorShared.mutex);
        } else if (!enabled) {
            // Motor disabled - set speed to 0
            setMotorSpeed(0);
            
            portENTER_CRITICAL(&motorShared.mutex);
            motorShared.motorSpeed = 0;
            motorShared.pressureValid = valid;
            if (valid) {
                motorShared.currentPsi = reading.pressurePsi;
                motorShared.smoothedPsi = smoothedPressure;
            }
            portEXIT_CRITICAL(&motorShared.mutex);
        } else {
            // Sensor error - disable motor for safety
            setMotorSpeed(0);
            pidReset(&motorPid);
            
            portENTER_CRITICAL(&motorShared.mutex);
            motorShared.motorSpeed = 0;
            motorShared.pressureValid = false;
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

uint8_t getMotorSpeedSafe(void) {
    uint8_t speed;
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
