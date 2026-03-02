/**
 * @file motor_control.h
 * @brief Motor control module for triac-based AC motor speed control
 * 
 * This module handles:
 * - Zero crossing detection via external interrupt
 * - Timer-based triac firing delay
 * - Motor speed control (0-100%)
 * - AC frequency detection (50Hz/60Hz)
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "pid_controller.h"

// Pin definitions
#define TRIAC_GATE_PIN      2   // IO2 - Triac Gate Control
#define ZERO_CROSSING_PIN   20  // IO20 - Zero Crossing Detection

// Timing constants for 60Hz AC (half cycle = 8.333ms)
#define MAXDELAY_60HZ       8000    // Maximum delay in microseconds for 60Hz
#define MAXDELAY_50HZ       9600    // Maximum delay in microseconds for 50Hz
#define MINDELAY            500     // Minimum delay to ensure triac fires

// Simulated zero crossing (compile-time option)
// Enable with build flag: -DSIMULATE_AC_60HZ
#define SIMULATED_ZC_HALF_CYCLE_US  8333  // 60Hz half-cycle period

// Triac gate pulse width
#define TRIAC_PULSE_US      100     // Gate pulse duration in microseconds

// Debug reporting interval
#define DEBUG_REPORT_INTERVAL_MS  3000  // Report ZC count every 3 seconds

// Enable serial triac diagnostics (safe, non-ISR printing)
// Add -DTRIAC_DEBUG_SERIAL to build_flags to enable
// #define TRIAC_DEBUG_SERIAL

/**
 * @brief AC frequency enum
 */
typedef enum {
    AC_FREQ_UNKNOWN = 0,
    AC_FREQ_50HZ = 50,
    AC_FREQ_60HZ = 60
} AcFrequency;

/**
 * @brief Motor control state structure
 */
typedef struct {
    volatile uint32_t zeroCrossingCount;    // Total zero crossing count
    volatile uint32_t lastZcTime;           // Last zero crossing timestamp (micros)
    volatile bool zcDetected;               // Flag set when ZC detected
    uint16_t motorSpeed;                    // Motor speed 0-100%
    uint16_t motorPwm;                      // Current PWM delay value
    uint16_t maxDelay;                      // Maximum delay based on AC frequency
    AcFrequency acFrequency;                // Detected AC frequency
    bool motorEnabled;                      // Motor enabled flag
} MotorControlState;

// Global motor control state
extern MotorControlState motorState;

/**
 * @brief Initialize the motor control module
 * 
 * Sets up:
 * - Zero crossing interrupt on IO20
 * - Triac gate output on IO2
 * - Timer for triac firing delay
 * 
 * @return true if initialization successful
 */
bool motorControlInit();

/**
 * @brief Set motor speed (0-100%)
 * 
 * Converts speed percentage to triac firing delay
 * 
 * @param speed Motor speed 0-100
 */
void setMotorSpeed(uint8_t speed);

/**
 * @brief Enable or disable motor
 * 
 * @param enable true to enable, false to disable
 */
void enableMotor(bool enable);

/**
 * @brief Get current zero crossing count
 * 
 * @return Zero crossing count since initialization
 */
uint32_t getZeroCrossingCount();

/**
 * @brief Reset zero crossing counter
 */
void resetZeroCrossingCount();

/**
 * @brief Detect AC frequency (50Hz or 60Hz)
 * 
 * Measures time between zero crossings to determine AC frequency
 * 
 * @return Detected AC frequency
 */
AcFrequency detectAcFrequency();

/**
 * @brief Debug function to report zero crossing stats
 * 
 * Call this periodically from main loop for debugging
 * Reports ZC count to serial every DEBUG_REPORT_INTERVAL_MS
 */
void motorControlDebugReport();

/**
 * @brief Fire the triac gate pulse
 * 
 * Called by timer interrupt when it's time to fire the triac
 */
void fireTriac();

// ============================================================================
// PID-Based Pressure Control Functions
// ============================================================================

/**
 * @brief Initialize the pressure PID controller
 * 
 * Call during system initialization after motor control init.
 * Loads learned parameters from NVS if available.
 */
void pressurePidInit();

/**
 * @brief Update motor speed based on PID output
 * 
 * Call this function regularly (e.g., every 100ms) with the current
 * pressure reading. The PID controller will calculate the appropriate
 * motor speed to achieve the target pressure.
 * 
 * @param currentPressurePsi Current measured pressure in PSI
 * @return Motor speed 0-100 that was set
 */
uint8_t updateMotorFromPid(float currentPressurePsi);

/**
 * @brief Set target pressure for PID control
 * 
 * @param targetPsi Target pressure in PSI
 */
void setTargetPressure(float targetPsi);

/**
 * @brief Get target pressure
 * 
 * @return Current target pressure in PSI
 */
float getTargetPressure();

/**
 * @brief Reset PID controller state
 * 
 * Call when entering/exiting idle mode or when motor is disabled.
 */
void resetPressurePid();

/**
 * @brief Get pointer to the pressure PID controller
 * 
 * Use for accessing PID status, metrics, or manual gain adjustment.
 * 
 * @return Pointer to PidController structure
 */
PidController* getPressurePid();

/**
 * @brief Enable or disable PID self-learning (legacy - does nothing)
 * 
 * @param enable True to enable learning, false to disable
 */
void setPidLearning(bool enable);

/**
 * @brief Check if PID learning is enabled (legacy - always returns false)
 * 
 * @return True if learning is enabled
 */
bool isPidLearningEnabled();

#endif // MOTOR_CONTROL_H
