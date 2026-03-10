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

// Idle / power pause behavior
#define IDLE_ENTRY_SECONDS          20    // Seconds of stable pressure before idle
#define IDLE_ENTRY_DEVIATION_PSI    0.3f  // Allowed deviation from target to count as stable
#define IDLE_ENTRY_DECREASE         50     // Counter decrease rate when outside band
#define IDLE_TARGET_PSI             2.5f  // Idle pressure target
#define IDLE_STABLE_SECONDS         2     // Seconds at idle target before holding speed
#define IDLE_STABLE_BAND_PSI        0.15f // Allowed deviation at idle target for stability
#define IDLE_EXIT_DROP_PSI          0.3f  // Pressure drop below idle target to exit
#define IDLE_MIN_HOLD_SPEED         50     // Minimum motor speed to hold in idle (0-1000 scale)

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
    volatile float idleEntryDeviationPsi; // Allowed deviation from target to count as stable
    
    // Outputs (written by motor task, read by display task)
    volatile float currentPsi;          // Current pressure reading
    volatile int32_t rawPressure;       // Raw 24-bit pressure value
    volatile float smoothedPsi;         // Smoothed pressure
    volatile uint16_t motorSpeed;       // Current motor speed 0-1000
    volatile float pidOutput;           // Raw PID output
    volatile bool pressureValid;        // Pressure sensor status
    volatile uint32_t idleSecondsRemaining; // Seconds until power pause activates
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
 * @brief Get raw sensor pressure (thread-safe)
 * @param pressurePsi Pointer to store raw pressure in PSI
 * @param valid Pointer to store sensor validity
 */
void getRawPressureSafe(float *pressurePsi, int32_t *rawValue, bool *valid);

/**
 * @brief Get motor speed (thread-safe)
 * @return Motor speed 0-1000
 */
uint16_t getMotorSpeedSafe(void);

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

void setIdleEntryDeviationSafe(float deviationPsi);
float getIdleEntryDeviationSafe(void);

/**
 * @brief Set idle entry deviation band (thread-safe)
 * @param deviationPsi Allowed deviation from target to count as stable
 */
void setIdleEntryDeviationSafe(float deviationPsi);

/**
 * @brief Get idle entry deviation band (thread-safe)
 * @return Allowed deviation from target to count as stable
 */
float getIdleEntryDeviationSafe(void);

#endif // DUAL_CORE_MOTOR_H
