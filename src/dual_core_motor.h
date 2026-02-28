/**
 * @file dual_core_motor.h
 * @brief Dual-core architecture for separating motor control from display
 * 
 * Motor control (time-critical) runs on Core 0
 * Display/UI runs on Core 1 (Arduino default)
 */

#ifndef DUAL_CORE_MOTOR_H
#define DUAL_CORE_MOTOR_H

#include <Arduino.h>

// Core assignments
#define MOTOR_CONTROL_CORE  0   // Core 0 for time-critical motor control
#define DISPLAY_CORE        1   // Core 1 for display (Arduino default)

// Motor control task parameters
#define MOTOR_TASK_STACK_SIZE   4096    // Stack size in bytes
#define MOTOR_TASK_PRIORITY     2       // Higher priority than display
#define MOTOR_LOOP_INTERVAL_US  5000    // 5ms loop interval (200Hz)

// Shared data structure (thread-safe access)
typedef struct {
    // Inputs (written by display task, read by motor task)
    volatile float targetPsi;           // Target pressure setpoint
    volatile bool motorEnabled;         // Enable/disable motor
    volatile bool pidResetRequest;      // Request PID reset
    volatile bool pidGainsChanged;      // Flag to indicate gains changed
    volatile float pidKp;               // PID proportional gain
    volatile float pidKi;               // PID integral gain
    volatile float pidKd;               // PID derivative gain
    
    // Outputs (written by motor task, read by display task)
    volatile float currentPsi;          // Current pressure reading
    volatile float smoothedPsi;         // Smoothed pressure
    volatile uint8_t motorSpeed;        // Current motor speed 0-100%
    volatile float pidOutput;           // Raw PID output
    volatile bool pressureValid;        // Pressure sensor status
    volatile uint32_t loopCount;        // Motor loop iteration count
    volatile uint32_t loopTimeUs;       // Actual loop time in microseconds
    volatile uint32_t maxLoopTimeUs;    // Maximum loop time observed
    
    // Synchronization
    portMUX_TYPE mutex;                 // Spinlock for atomic access
} MotorSharedData;

// Global shared data instance
extern MotorSharedData motorShared;

/**
 * @brief Initialize the dual-core motor control system
 * 
 * Creates a high-priority task on Core 0 for motor control
 * 
 * @return true if initialization successful
 */
bool dualCoreMotorInit(void);

/**
 * @brief Set target pressure (thread-safe)
 * @param psi Target pressure in PSI
 */
void setTargetPressureSafe(float psi);

/**
 * @brief Get current pressure (thread-safe)
 * @return Current smoothed pressure in PSI
 */
float getCurrentPressureSafe(void);

/**
 * @brief Get motor speed (thread-safe)
 * @return Motor speed 0-100%
 */
uint8_t getMotorSpeedSafe(void);

/**
 * @brief Get motor loop timing stats (thread-safe)
 * @param avgUs Pointer to store average loop time
 * @param maxUs Pointer to store maximum loop time
 */
void getMotorLoopStats(uint32_t *avgUs, uint32_t *maxUs);

/**
 * @brief Request PID reset (thread-safe)
 */
void requestPidReset(void);

/**
 * @brief Enable/disable motor (thread-safe)
 * @param enable true to enable motor
 */
void setMotorEnabledSafe(bool enable);

/**
 * @brief Set PID gains (thread-safe) - syncs to motor task
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void setPidGainsSafe(float kp, float ki, float kd);

/**
 * @brief Get PID gains from motor task (thread-safe)
 * @param kp Pointer to store Kp value
 * @param ki Pointer to store Ki value
 * @param kd Pointer to store Kd value
 */
void getPidGainsSafe(float *kp, float *ki, float *kd);

#endif // DUAL_CORE_MOTOR_H
