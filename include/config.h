/**
 * @file config.h
 * @brief Central configuration for Apollo HVLP ESP32-S3 Firmware
 *
 * This file is the single source of truth for:
 *   - Hardware pin assignments
 *   - Display color theme
 *   - Run-time tunable parameters (intervals, thresholds, limits)
 *   - Firmware identity
 *
 * All other modules should include this header instead of defining
 * their own pin or configuration constants.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// Firmware Identity
// ============================================================================

#define FIRMWARE_VERSION    "2.0.0"

// ============================================================================
// Pin Assignments
// ============================================================================

// --- Motor / Triac ---
#define PIN_TRIAC_GATE      2   // IO2  - Triac gate control output
#define PIN_ZERO_CROSSING   20  // IO20 - Zero-crossing detection input

// --- Alarm ---
#define PIN_BEEPER          4   // IO4  - Alarm beeper output

// --- Temperature (ADC) ---
#define PIN_TEMP_SENSOR     3   // IO3  - Temperature sensor (ADC input)

// --- Pressure Sensor (I2C) ---
#define PIN_PRESSURE_CSB    19  // IO19 - CSB: float/HIGH for I2C, LOW for SPI
#define PIN_PRESSURE_SDA    17  // IO17 - I2C SDA
#define PIN_PRESSURE_SCL    18  // IO18 - I2C SCL

// --- Display (SPI - configured via TFT_eSPI build flags) ---
#define PIN_TFT_BL          21  // IO21 - Backlight control
// Note: MOSI=35, SCK=36, MISO=37, CS=10, DC=7, RST=6 are set in platformio.ini

// --- Rotary Encoder ---
#define PIN_ENCODER_CLK     41  // IO41 - Encoder A (Right)
#define PIN_ENCODER_DT      42  // IO42 - Encoder B (Left)
#define PIN_ENCODER_BTN     5   // IO5  - Encoder push-button

// ============================================================================
// Display Color Theme
// ============================================================================

#define COLOR_BG            TFT_BLACK
#define COLOR_TARGET        TFT_YELLOW
#define COLOR_CURRENT       TFT_CYAN
#define COLOR_LABEL         TFT_WHITE
#define COLOR_MINMAX        TFT_RED
#define COLOR_TEMP          TFT_ORANGE
#define COLOR_RUNTIME       TFT_GREEN
#define COLOR_DEBUG         TFT_MAGENTA

// ============================================================================
// Pressure Sensor
// ============================================================================

#define PRESSURE_I2C_ADDR           0x6D    // 7-bit address: 1101101
#define PRESSURE_BAR_TO_PSI         14.5038f
#define PRESSURE_MULTI              2       // Unit multiplier
#define PRESSURE_RANGE_BAR          2.0f    // Sensor range (2 Bar gauge)
#define PRESSURE_RANGE_PSI          73.5f
#define PRESSURE_ZERO_POINT         8388608   // 2^23 zero condition
#define PRESSURE_FULL_SCALE         8388608.0f
#define PRESSURE_24BIT_MAX          16777216  // 2^24
#define PRESSURE_23BIT_MAX          8388608   // 2^23
#define PRESSURE_CONVERSION_TIMEOUT_MS  100

// ============================================================================
// Temperature Sensor
// ============================================================================

#define TEMP_MULT           -87
#define TEMP_DIVISOR        200
#define TEMP_OFFSET         210

// Overtemperature protection thresholds (°C)
#define TEMP_OVERHEAT_SETPOINT  115.0f  // Shutdown motor above this
#define TEMP_OVERHEAT_CLEAR     100.0f  // Re-enable motor below this

// ============================================================================
// Motor / Triac Timing
// ============================================================================

#define TRIAC_MAXDELAY_60HZ     8000    // Maximum firing delay (µs) for 60 Hz AC
#define TRIAC_MAXDELAY_50HZ     9600    // Maximum firing delay (µs) for 50 Hz AC
#define TRIAC_MINDELAY          500     // Minimum firing delay (µs)
#define TRIAC_PULSE_US          100     // Gate pulse width (µs)

// Simulated zero-crossing (enabled with -DSIMULATE_AC_60HZ build flag)
#define SIMULATED_ZC_HALF_CYCLE_US  8333  // 60 Hz half-cycle period (µs)

// ============================================================================
// PID Controller Defaults & Limits
// ============================================================================

#define PID_KP_DEFAULT      4.0f
#define PID_KI_DEFAULT      2.0f
#define PID_KD_DEFAULT      2.0f

#define PID_OUTPUT_MIN      -1000.0f
#define PID_OUTPUT_MAX       1000.0f
#define PID_INTEGRAL_MAX     1000.0f

#define PID_KP_MIN          0.25f
#define PID_KP_MAX          100.0f
#define PID_KI_MIN          0.0f
#define PID_KI_MAX          100.0f
#define PID_KD_MIN          0.0f
#define PID_KD_MAX          100.0f

// ============================================================================
// Target Pressure Settings
// ============================================================================

#define TARGET_PSI_MIN      3.0f
#define TARGET_PSI_MAX      9.0f
#define TARGET_PSI_DEFAULT  6.0f
#define TARGET_PSI_STEP     0.1f    // Encoder step size (PSI)

// ============================================================================
// Idle / Power-Pause Behavior
// ============================================================================

#define IDLE_ENTRY_SECONDS          20      // Stable-pressure seconds before entering idle
#define IDLE_ENTRY_DEVIATION_PSI    0.3f    // Allowed PSI deviation from target for "stable"
#define IDLE_ENTRY_DECREASE         200     // Counter decrease rate when outside band
#define IDLE_TARGET_PSI             2.5f    // Pressure target while in power-pause ramp
#define IDLE_STABLE_SECONDS         2       // Seconds at idle target before hold
#define IDLE_STABLE_BAND_PSI        0.15f   // Allowed deviation at idle target for stability
#define IDLE_EXIT_DROP_PSI          0.2f    // Pressure drop below idle target to exit
#define IDLE_MIN_HOLD_SPEED         50      // Minimum hold speed in idle (0-1000)

// Power-pause setting bounds
#define POWER_PAUSE_SEC_MIN         0
#define POWER_PAUSE_SEC_MAX         600
#define POWER_PAUSE_SEC_STEP        1
#define POWER_PAUSE_WARN_SEC_MIN    0
#define POWER_PAUSE_WARN_SEC_MAX    600
#define POWER_PAUSE_WARN_SEC_STEP   1

// ============================================================================
// Settings Adjustment Step Sizes
// ============================================================================

#define IDLE_DEV_STEP       0.05f
#define IDLE_DEV_MIN        0.05f
#define IDLE_DEV_MAX        2.0f

// ============================================================================
// Polling / Update Intervals
// ============================================================================

#define TEMP_READ_INTERVAL_MS           1000    // Temperature polling (ms)
#define DISPLAY_PRESSURE_INTERVAL_MS    100     // Pressure display refresh (ms, ~10 Hz)
#define MOTOR_LOOP_INTERVAL_US          5000    // Motor control task loop period (µs, 200 Hz)

// Hour-meter update: every 6 minutes = 0.1 hour
#define HOUR_METER_UPDATE_INTERVAL_MS   360000UL

// Button debounce
#define BUTTON_DEBOUNCE_MS              30

// Debug report interval (used when TRIAC_DEBUG_SERIAL is active)
#define DEBUG_REPORT_INTERVAL_MS        3000

// ============================================================================
// Dual-Core Task Parameters
// ============================================================================

#define MOTOR_CONTROL_CORE      0   // Core 0: time-critical motor control
#define DISPLAY_CORE            1   // Core 1: display / UI (Arduino default)
#define MOTOR_TASK_STACK_SIZE   4096
#define MOTOR_TASK_PRIORITY     2

// ============================================================================
// Display Layout (480×320 landscape)
// ============================================================================

#define SCREEN_WIDTH        480
#define SCREEN_HEIGHT       320

#define PRESSURE_ZONE_HEIGHT    213
#define STATUS_ZONE_Y           220
#define LEFT_COLUMN_X           10
#define RIGHT_COLUMN_X          245
#define COLUMN_WIDTH            225

// Runtime screen grid (3×3)
#define GRID_COLS           3
#define GRID_ROWS           3
#define CELL_WIDTH          (SCREEN_WIDTH / GRID_COLS)
#define CELL_HEIGHT         (SCREEN_HEIGHT / GRID_ROWS)
#define RUNTIME_TOP_HEIGHT  (CELL_HEIGHT * 2)
#define RUNTIME_RIGHT_X     (CELL_WIDTH * 2)
#define RUNTIME_FOOTER_Y    (CELL_HEIGHT * 2)
#define RUNTIME_FOOTER_WIDTH (CELL_WIDTH * 2)

// Menu
#define MENU_OPTION_COUNT       4
#define MENU_OPTION_HEIGHT      40
#define MENU_TOP_Y              90

// Settings  (Idle dev, Start psi, Save, Back)
#define SETTINGS_OPTION_COUNT   4
#define SETTINGS_OPTION_HEIGHT  36
#define SETTINGS_TOP_Y          70

#define MINMAX_FLASH_MS         500

#endif // CONFIG_H
