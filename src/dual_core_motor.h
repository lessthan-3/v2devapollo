/**
 * @file dual_core_motor.h
 * @brief Dual-core architecture for separating motor control from display
 *
 * Motor control (time-critical) runs on Core 0.
 * Display / UI runs on Core 1 (Arduino default).
 */

#ifndef DUAL_CORE_MOTOR_H
#define DUAL_CORE_MOTOR_H

#include <Arduino.h>
#include "config.h"

typedef enum {
    IDLE_STATE_OFF = 0,
    IDLE_STATE_PID_RAMP,
    IDLE_STATE_HOLD
} IdleState;

typedef enum {
    UNITS_IMPERIAL = 0,
    UNITS_METRIC   = 1
} DisplayUnits;

// Shared data between Core 0 (motor task) and Core 1 (display task)
typedef struct {
    // Inputs — written by display task, read by motor task
    volatile float    targetPsi;
    volatile bool     motorEnabled;
    volatile bool     pidResetRequest;
    volatile bool     pidGainsChanged;
    volatile float    pidKp;
    volatile float    pidKi;
    volatile float    pidKd;
    volatile float    idleEntryDeviationPsi;
    volatile uint16_t idleEntrySeconds;

    // Outputs — written by motor task, read by display task
    volatile float    currentPsi;
    volatile int32_t  rawPressure;
    volatile float    smoothedPsi;
    volatile uint16_t motorSpeed;
    volatile float    pidOutput;
    volatile bool     pressureValid;
    volatile uint32_t idleSecondsRemaining;
    volatile uint8_t  idleState;
    volatile bool     idleExitRequest;
    volatile bool     isMax;            // true when running in MAX mode (target >= MAX_PSI_THRESHOLD)
    volatile uint32_t loopCount;
    volatile uint32_t loopTimeUs;
    volatile uint32_t maxLoopTimeUs;

    portMUX_TYPE mutex;
} MotorSharedData;

extern MotorSharedData motorShared;

/**
 * @brief Create the motor control task on Core 0.
 * @return true on success
 */
bool dualCoreMotorInit(void);

/** @brief Set target pressure (thread-safe). */
void setTargetPressureSafe(float psi);

/** @brief Get smoothed pressure (thread-safe). */
float getCurrentPressureSafe(void);

/** @brief Get raw sensor reading (thread-safe). */
void getRawPressureSafe(float *pressurePsi, int32_t *rawValue, bool *valid);

/** @brief Get motor speed (thread-safe). */
uint16_t getMotorSpeedSafe(void);

/** @brief Get motor loop timing stats (thread-safe). */
void getMotorLoopStats(uint32_t *avgUs, uint32_t *maxUs);

/** @brief Request a PID reset (thread-safe). */
void requestPidReset(void);

/** @brief Enable or disable the motor (thread-safe). */
void setMotorEnabledSafe(bool enable);

/** @brief Set PID gains (thread-safe). */
void setPidGainsSafe(float kp, float ki, float kd);

/** @brief Read PID gains (thread-safe). */
void getPidGainsSafe(float *kp, float *ki, float *kd);

/** @brief Set idle entry duration (thread-safe). */
void setIdleEntrySecondsSafe(uint16_t seconds);

/** @brief Get idle entry duration (thread-safe). */
uint16_t getIdleEntrySecondsSafe(void);

/** @brief Set idle entry deviation band (thread-safe). */
void setIdleEntryDeviationSafe(float deviationPsi);

/** @brief Get idle entry deviation band (thread-safe). */
float getIdleEntryDeviationSafe(void);

/** @brief Request exit from power pause (thread-safe). */
void requestIdleExitSafe(void);

#endif // DUAL_CORE_MOTOR_H
