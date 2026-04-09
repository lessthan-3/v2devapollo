/**
 * @file config.h
 * @brief Central configuration for Apollo HVLP ESP32-S3 firmware
 *
 * All pin assignments, timing constants, PID defaults, display layout,
 * and feature flags live here. No #define should be scattered across
 * individual module headers unless it is truly private to that module.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// Firmware Version
// ============================================================================
#define FIRMWARE_VERSION    "2.0.0"

// ============================================================================
// Pin Assignments
// ============================================================================
#define TRIAC_GATE_PIN          2   // IO2  - Triac Gate Control
#define ZERO_CROSSING_PIN       20  // IO20 - Zero Crossing Detection
#define BEEPER_PIN              4   // IO4  - Alarm Beeper
#define TEMP_SENSOR_PIN         3   // IO3  - Temperature Sensor (ADC)

// Pressure Sensor (WF100DPZ, I2C)
#define PRESSURE_CSB_PIN        19  // IO19 - CSB: HIGH/floating = I2C mode
#define PRESSURE_SDA_PIN        17  // IO17 - SDA
#define PRESSURE_SCL_PIN        18  // IO18 - SCL

// Display (ST7796, SPI) -- SPI pins are configured via TFT_eSPI library settings
#define TFT_BL                  21  // IO21 - Backlight

// Rotary Encoder
#define ENCODER_CLK             41  // IO41 - A (Right)
#define ENCODER_DT              42  // IO42 - B (Left)
#define ENCODER_BTN             5   // IO5  - Push Button

// ============================================================================
// Motor / Triac Timing
// ============================================================================
#define MAXDELAY_60HZ           8000    // Max firing delay (us) for 60 Hz AC
#define MAXDELAY_50HZ           9600    // Max firing delay (us) for 50 Hz AC
#define MINDELAY                500     // Min firing delay (us) to ensure triac fires
#define TRIAC_PULSE_US          100     // Triac gate pulse width (us)

// Simulated zero-crossing (enable via build flag -DSIMULATE_AC_60HZ)
#define SIMULATED_ZC_HALF_CYCLE_US  8333  // 60 Hz half-cycle period (us)

// ============================================================================
// Pressure Sensor (WF100DPZ)
// ============================================================================
#define PRESSURE_I2C_ADDR       0x6D    // 7-bit I2C address (1101101)

// Register map
#define REG_STATUS              0x02
#define REG_PRESSURE_MSB        0x06
#define REG_PRESSURE_CSB        0x07
#define REG_PRESSURE_LSB        0x08
#define REG_CMD                 0x30

// Command values
#define CMD_SINGLE_OUTPUT       0x0A
#define CMD_CONTINUOUS_OUTPUT   0x0B
#define CMD_INTERVAL_62_5MS     0x1B
#define CMD_INTERVAL_125MS      0x2B
#define CMD_INTERVAL_1S         0xFB

// Pressure math
#define PRESSURE_ZERO_POINT     8388608     // 2^23 zero condition
#define PRESSURE_FULL_SCALE     8388608.0f
#define PRESSURE_24BIT_MAX      16777216    // 2^24
#define PRESSURE_23BIT_MAX      8388608     // 2^23
#define PRESSURE_BAR_TO_PSI     14.5038f
#define PRESSURE_MULTI          2
#define PRESSURE_RANGE_BAR      2.0f
#define PRESSURE_RANGE_PSI      73.5f
#define CONVERSION_TIMEOUT_MS   100
#define STATUS_CONVERSION_DONE  0x01

// ============================================================================
// PID Controller
// ============================================================================
#define PID_KP_DEFAULT          5.0f
#define PID_KI_DEFAULT          3.5f
#define PID_KD_DEFAULT          1.6f

#define PID_OUTPUT_MIN          -1000.0f
#define PID_OUTPUT_MAX          1000.0f
#define PID_INTEGRAL_MAX        1000.0f

#define PID_KP_MIN              0.25f
#define PID_KP_MAX              100.0f
#define PID_KI_MIN              0.0f
#define PID_KI_MAX              100.0f
#define PID_KD_MIN              0.0f
#define PID_KD_MAX              100.0f

// Low-PSI overshoot compensation
// Below LOW_PSI_THRESHOLD the effective gains are linearly scaled down to
// LOW_PSI_GAIN_SCALE (fraction) to prevent overshoot at low setpoints.
#define LOW_PSI_THRESHOLD       3.0f    // PSI below which gain scaling is active
#define LOW_PSI_GAIN_SCALE      0.35f   // Minimum gain multiplier at 0 PSI
#define LOW_PSI_KI_GAIN_SCALE      0.0f   // Minimum gain multiplier at 0 PSI
#define LOW_PSI_KD_GAIN_SCALE      0.1f   // Minimum gain multiplier at 0 PSI



// NVS storage keys for PID gains
#define NVS_NAMESPACE_PID       "pid_cfg"
#define NVS_KEY_KP              "kp"
#define NVS_KEY_KI              "ki"
#define NVS_KEY_KD              "kd"
#define NVS_KEY_VALID           "valid"

// ============================================================================
// Dual-Core Motor Control
// ============================================================================
#define MOTOR_CONTROL_CORE          0       // Core 0: time-critical motor control
#define DISPLAY_CORE                1       // Core 1: display/UI (Arduino default)
#define MOTOR_TASK_STACK_SIZE       4096
#define MOTOR_TASK_PRIORITY         2
#define MOTOR_LOOP_INTERVAL_US      5000    // 5 ms loop (200 Hz)

// Idle / Power Pause
#define IDLE_ENTRY_SECONDS          20      // Seconds of stable pressure before idle
#define IDLE_ENTRY_DEVIATION_PSI    0.12f   // Band around setpoint counted as stable
#define IDLE_BAND_THRESHOLD_PSI     0.17f   // Band to increment power pause counter 
#define IDLE_ENTRY_DECREASE         2000     // Counter decrease when outside band
#define IDLE_TARGET_PSI             2.5f    // Pressure target while idle
#define IDLE_STABLE_SECONDS         2       // Seconds at idle target before holding
#define IDLE_STABLE_BAND_PSI        0.15f   // Stability band at idle target
#define IDLE_EXIT_DROP_PSI          0.2f    // Pressure drop below idle target to exit
#define IDLE_MIN_HOLD_SPEED         50      // Min motor speed in idle hold (0-1000)
#define MAX_PRESSURE_DEVIATION_PSI  0.30f   // Deviation from peak used for MAX-mode power pause entry
#define IDLE_LOOP_INCREMENT           3       // Ticks per loop within deviation band for idle entry

// ============================================================================
// Temperature Sensor
// ============================================================================
// Conversion formula (applied in tempSensorReadC()):
//   tempC = -1.75 * adcValue + 207.0
#define TEMP_OVERHEAT_SETPOINT      115.0f  // Shutdown above this (°C)
//#define TEMP_OVERHEAT_SETPOINT      400.0f  // Shutdown above this (°C) - set very high for initial testing without temp sensor

#define TEMP_OVERHEAT_CLEAR         100.0f  // Re-enable motor below this (°C)

// ============================================================================
// Target Pressure (user-facing)
// ============================================================================
#define TARGET_PSI_MIN          0.0f    // Minimum user-selectable pressure
#define TARGET_PSI_MAX          9.0f    // Encoder upper bound (beyond MAX_PSI_THRESHOLD = MAX mode)
#define TARGET_PSI_DEFAULT      6.0f
#define TARGET_PSI_STEP         0.1f
#define MAX_PSI_THRESHOLD       8.5f    // At or above this value: motor runs at 100% (MAX mode)

// ============================================================================
// Power Pause Settings
// ============================================================================
#define POWER_PAUSE_SEC_MIN     20
#define POWER_PAUSE_SEC_MAX     600
#define POWER_PAUSE_SEC_STEP    1
#define POWER_PAUSE_WARN_SEC    10      // Fixed warning countdown (seconds)

// Idle entry deviation tuning bounds
#define IDLE_DEV_MIN            0.05f
#define IDLE_DEV_MAX            2.0f

// ============================================================================
// Display Layout (480x320 landscape, ST7796)
// ============================================================================
#define SCREEN_WIDTH            480
#define SCREEN_HEIGHT           320

// Runtime screen layout
#define GRID_COLS               3
#define GRID_ROWS               3
#define CELL_WIDTH              (SCREEN_WIDTH / GRID_COLS)
#define CELL_HEIGHT             (SCREEN_HEIGHT / GRID_ROWS)
#define RUNTIME_TOP_HEIGHT      (CELL_HEIGHT * 2)
#define RUNTIME_RIGHT_X         (CELL_WIDTH * 2)
#define RUNTIME_FOOTER_Y        (CELL_HEIGHT * 2)
#define RUNTIME_FOOTER_WIDTH    (CELL_WIDTH * 2)

// Legacy layout constants (used by display_ui.cpp)
#define PRESSURE_ZONE_HEIGHT    213
#define STATUS_ZONE_Y           220
#define LEFT_COLUMN_X           10
#define RIGHT_COLUMN_X          245
#define COLUMN_WIDTH            225

// Menu / Settings
#define MENU_OPTION_COUNT       5
#define MENU_OPTION_HEIGHT      40
#define MENU_TOP_Y              90
#define SETTINGS_OPTION_COUNT   4
#define SETTINGS_OPTION_HEIGHT  40
#define SETTINGS_TOP_Y          85
#define SUPPORT_OPTION_COUNT    4
#define MINMAX_FLASH_MS         500

// Color theme
#define COLOR_BG                TFT_BLACK
#define COLOR_TEXT_PRIMARY      TFT_WHITE
#define COLOR_TEXT_SECONDARY    TFT_LIGHTGREY
#define COLOR_TARGET_ACTIVE     TFT_GREEN
#define COLOR_TARGET_OUTRANGE   TFT_RED
#define COLOR_TARGET_INACTIVE   TFT_YELLOW
#define COLOR_CURRENT           TFT_CYAN
#define COLOR_TEMP              TFT_ORANGE
#define COLOR_TEMP_WARNING      TFT_RED
#define COLOR_RUNTIME           TFT_GREEN
#define COLOR_WARNING           TFT_ORANGE
#define COLOR_ERROR             TFT_RED
#define COLOR_SUCCESS           TFT_GREEN
#define COLOR_LABEL             TFT_WHITE
#define COLOR_MINMAX            TFT_RED
#define COLOR_DEBUG             TFT_MAGENTA
#define COLOR_OVERLAY_BG        TFT_BLACK
#define COLOR_OVERLAY_BORDER    TFT_WHITE
#define COLOR_MENU_SELECT       TFT_DARKGREY
#define COLOR_MENU_EDIT         TFT_YELLOW

// ============================================================================
// Polling / Update Intervals (ms)
// ============================================================================
#define DISPLAY_PRESSURE_INTERVAL_MS    100     // Pressure display refresh (10 Hz)
#define SERIAL_DEBUG_INTERVAL_MS        1000    // Serial debug output
#define TEMP_READ_INTERVAL_MS           1000    // Temperature polling
#define DEBUG_REPORT_INTERVAL_MS        3000    // Motor control debug report

// ============================================================================
// Debug Flags
// ============================================================================
#define DEBUG_SERIAL_OUTPUT     1   // 1 = enable serial debug in loop()
// Add -DTRIAC_DEBUG_SERIAL to build_flags to enable triac ISR diagnostics
// Add -DSIMULATE_AC_60HZ    to build_flags to simulate zero crossings

#endif // CONFIG_H
