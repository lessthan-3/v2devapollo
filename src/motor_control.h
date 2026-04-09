/**
 * @file motor_control.h
 * @brief Motor control module for triac-based AC motor speed control
 *
 * Handles:
 * - Zero crossing detection via external interrupt
 * - Timer-based triac firing delay
 * - Motor speed control (0-1000)
 * - AC frequency detection (50 Hz / 60 Hz)
 * - PID-based pressure control
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "config.h"
#include "pid_controller.h"

/**
 * @brief AC frequency enum
 */
typedef enum {
    AC_FREQ_UNKNOWN = 0,
    AC_FREQ_50HZ    = 50,
    AC_FREQ_60HZ    = 60
} AcFrequency;

/**
 * @brief Motor control state structure
 */
typedef struct {
    volatile uint32_t zeroCrossingCount;
    volatile uint32_t lastZcTime;
    volatile bool     zcDetected;
    uint16_t          motorSpeed;
    uint16_t          motorPwm;
    uint16_t          maxDelay;
    AcFrequency       acFrequency;
    bool              motorEnabled;
} MotorControlState;

extern MotorControlState motorState;

/**
 * @brief Initialise motor control (zero crossing interrupt + triac timer).
 * @return true on success
 */
bool motorControlInit(void);

/**
 * @brief Set motor speed (0-1000).
 * @param speed 0 = off, 1000 = full power
 */
void setMotorSpeed(uint16_t speed);

/**
 * @brief Enable or disable the motor.
 * @param enable true to enable
 */
void enableMotor(bool enable);

/**
 * @brief Return the cumulative zero crossing count.
 */
uint32_t getZeroCrossingCount(void);

/**
 * @brief Reset the zero crossing counter to zero.
 */
void resetZeroCrossingCount(void);

/**
 * @brief Measure AC frequency from live zero crossings.
 * @return Detected AcFrequency (AC_FREQ_UNKNOWN on timeout)
 */
AcFrequency detectAcFrequency(void);

// ============================================================================
// PID-Based Pressure Control
// ============================================================================

/**
 * @brief Initialise the pressure PID controller.
 *        Loads saved gains from NVS if available.
 */
void pressurePidInit(void);

/**
 * @brief Set the PID target pressure.
 * @param targetPsi Target in PSI
 */
void setTargetPressure(float targetPsi);

/**
 * @brief Get the current PID target pressure.
 */
float getTargetPressure(void);

/**
 * @brief Reset PID integral and state.
 *        Call when entering/exiting idle or when motor is disabled.
 */
void resetPressurePid(void);

/**
 * @brief Return a pointer to the pressure PID controller instance.
 */
PidController* getPressurePid(void);

#endif // MOTOR_CONTROL_H
