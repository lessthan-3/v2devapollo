/**
 * @file pid_controller.h
 * @brief PID Controller for Pressure Regulation
 *
 * Implements a PID controller with:
 * - Anti-windup protection on the integral term
 * - NVS persistence for tuned gains
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

// ============================================================================
// Data Structures
// ============================================================================

typedef struct {
    float kp;
    float ki;
    float kd;

    float setpoint;
    float integral;
    float prevError;
    float prevOutput;

    float outputMin;
    float outputMax;
    float integralMax;

    float dt;
    unsigned long lastUpdateTime;

    float deadband;

    bool hasStoredParams;
} PidController;

// ============================================================================
// Function Declarations
// ============================================================================

/**
 * @brief Initialise the PID controller.
 *        Attempts to load saved gains from NVS; falls back to defaults.
 */
void pidInit(PidController *pid, float kp, float ki, float kd);

/**
 * @brief Reset controller state (integral, derivative).
 *        Safe to call from Core 0 — no Serial output.
 */
void pidReset(PidController *pid);

/**
 * @brief Calculate the PID adjustment for the given measurement.
 * @param currentValue Current measured pressure (PSI)
 * @return Motor speed adjustment
 */
float pidCalculate(PidController *pid, float currentValue);

/**
 * @brief Set the target setpoint.
 */
void pidSetSetpoint(PidController *pid, float setpoint);

/**
 * @brief Get the current setpoint.
 */
float pidGetSetpoint(PidController *pid);

/**
 * @brief Save current gains to NVS.
 * @return true on success
 */
bool pidSaveGains(PidController *pid);

/**
 * @brief Load gains from NVS.
 * @return true if valid saved gains were loaded
 */
bool pidLoadGains(PidController *pid);

/**
 * @brief Reset gains to compile-time defaults and clear NVS.
 */
void pidResetToDefaults(PidController *pid);

/**
 * @brief Read the current gains.
 */
void pidGetGains(PidController *pid, float *kp, float *ki, float *kd);

/**
 * @brief Set gains, clamped to PID_KP/KI/KD_MIN/MAX from config.h.
 *        Safe to call from Core 0 — no Serial output.
 */
void pidSetGains(PidController *pid, float kp, float ki, float kd);

/**
 * @brief Print PID status to Serial (debug only).
 */
void pidDebugPrint(PidController *pid);

#endif // PID_CONTROLLER_H
