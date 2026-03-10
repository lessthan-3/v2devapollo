/**
 * @file pid_controller.h
 * @brief PID Controller for Pressure Regulation with Manual Tuning
 * 
 * This module implements a PID controller that:
 * - Tracks a pressure setpoint using proportional, integral, and derivative control
 * - Includes anti-windup protection for the integral term
 * - Supports manual gain adjustment via encoder
 * - Persists parameters to NVS flash storage
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include <Preferences.h>

// ============================================================================
// PID Controller Default Constants
// ============================================================================
#define PID_KP_DEFAULT      4.0f    // Default proportional gain (reduced)
#define PID_KI_DEFAULT      2.0f    // Default integral gain (reduced)
#define PID_KD_DEFAULT      2.0f    // Default derivative gain (reduced)

// PID Output Limits
#define PID_OUTPUT_MIN      -1000.0f // Minimum PID output (motor speed adjustment)
#define PID_OUTPUT_MAX      1000.0f  // Maximum PID output (motor speed adjustment)
#define PID_INTEGRAL_MAX    1000.0f  // Anti-windup limit for integral term

// Gain limits
#define PID_KP_MIN          0.25f
#define PID_KP_MAX          100.0f
#define PID_KI_MIN          0.0f
#define PID_KI_MAX          100.0f
#define PID_KD_MIN          0.0f
#define PID_KD_MAX          100.0f

// NVS Storage keys
#define NVS_NAMESPACE_PID   "pid_cfg"
#define NVS_KEY_KP          "kp"
#define NVS_KEY_KI          "ki"
#define NVS_KEY_KD          "kd"
#define NVS_KEY_VALID       "valid"

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief PID Controller structure
 */
typedef struct {
    // PID Gains
    float kp;                       // Proportional gain
    float ki;                       // Integral gain
    float kd;                       // Derivative gain
    
    // Setpoint and state
    float setpoint;                 // Target value (pressure in PSI)
    float integral;                 // Accumulated integral term
    float prevError;                // Previous error for derivative
    float prevOutput;               // Previous output adjustment for rate limiting
    
    // Output limits
    float outputMin;
    float outputMax;
    float integralMax;              // Anti-windup limit
    
    // Timing
    float dt;                       // Time delta for calculations (seconds)
    unsigned long lastUpdateTime;   // Last update timestamp
    
    // Deadband for stability
    float deadband;                 // Error deadband (no adjustment if error within this)
    
    // NVS save state
    bool hasStoredParams;           // Whether params are stored in NVS
} PidController;

// ============================================================================
// Function Declarations
// ============================================================================

/**
 * @brief Initialize the PID controller
 * 
 * Attempts to load previously saved gains from NVS.
 * Falls back to default gains if no saved params exist.
 * 
 * @param pid Pointer to PID controller structure
 * @param kp Initial proportional gain (used if no saved params)
 * @param ki Initial integral gain (used if no saved params)
 * @param kd Initial derivative gain (used if no saved params)
 */
void pidInit(PidController *pid, float kp, float ki, float kd);

/**
 * @brief Reset PID controller state (integral, derivative, etc.)
 * 
 * Call when motor is stopped, setpoint changes significantly,
 * or entering/exiting idle mode.
 * 
 * @param pid Pointer to PID controller structure
 */
void pidReset(PidController *pid);

/**
 * @brief Calculate PID output based on current value
 * 
 * @param pid Pointer to PID controller structure
 * @param currentValue Current measured value (pressure in PSI)
 * @return PID output (motor speed adjustment)
 */
float pidCalculate(PidController *pid, float currentValue);

/**
 * @brief Set the target setpoint
 * 
 * @param pid Pointer to PID controller structure
 * @param setpoint New target setpoint (pressure in PSI)
 */
void pidSetSetpoint(PidController *pid, float setpoint);

/**
 * @brief Get current setpoint
 * 
 * @param pid Pointer to PID controller structure
 * @return Current setpoint value
 */
float pidGetSetpoint(PidController *pid);

/**
 * @brief Save current gains to NVS
 * 
 * @param pid Pointer to PID controller structure
 * @return True if save successful
 */
bool pidSaveGains(PidController *pid);

/**
 * @brief Load gains from NVS
 * 
 * @param pid Pointer to PID controller structure
 * @return True if valid saved gains were loaded
 */
bool pidLoadGains(PidController *pid);

/**
 * @brief Mark current parameters as learned/saved
 * 
 * This enables saving to NVS.
 * 
 * @param pid Pointer to PID controller structure
 * @param saveNow If true, immediately save to NVS after marking
 */
void pidMarkAsLearned(PidController *pid, bool saveNow);

/**
 * @brief Reset gains to defaults and clear NVS
 * 
 * @param pid Pointer to PID controller structure
 */
void pidResetToDefaults(PidController *pid);

/**
 * @brief Get current PID gains for display
 * 
 * @param pid Pointer to PID controller structure
 * @param kp Pointer to store Kp value
 * @param ki Pointer to store Ki value
 * @param kd Pointer to store Kd value
 */
void pidGetGains(PidController *pid, float *kp, float *ki, float *kd);

/**
 * @brief Set PID gains manually
 * 
 * @param pid Pointer to PID controller structure
 * @param kp New proportional gain
 * @param ki New integral gain
 * @param kd New derivative gain
 */
void pidSetGains(PidController *pid, float kp, float ki, float kd);

/**
 * @brief Print PID status to Serial (debug)
 * 
 * @param pid Pointer to PID controller structure
 */
void pidDebugPrint(PidController *pid);

// Legacy compatibility stubs (do nothing)
void pidEnableLearning(PidController *pid, bool enable);
bool pidIsLearningEnabled(PidController *pid);

#endif // PID_CONTROLLER_H
