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
#define FIRMWARE_VERSION    "2.4.0"

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
#define IDLE_ENTRY_SECONDS          20      // Seconds of stable motor speed before idle
#define IDLE_ENTRY_DEVIATION_PSI    0.12f   // (legacy, kept for UI compatibility)
#define IDLE_BAND_THRESHOLD_PSI     0.17f   // (legacy, kept for UI compatibility)
#define IDLE_ENTRY_DECREASE         2000    // Counter decrease ticks when spike detected
#define IDLE_TARGET_PSI             2.5f    // Pressure target while idle
#define IDLE_STABLE_SECONDS         2       // Seconds at idle target before holding
#define IDLE_STABLE_BAND_PSI        0.15f   // Stability band at idle target
#define IDLE_EXIT_DROP_PSI          0.2f    // Pressure drop below idle target to exit
#define IDLE_MIN_HOLD_SPEED         50      // Min motor speed in idle hold (0-1000)
#define MAX_PRESSURE_DEVIATION_PSI  0.35f   // Deviation from peak used for MAX-mode power pause entry
#define IDLE_LOOP_INCREMENT         3       // Ticks per loop when stable motor speed detected

// Motor-speed steady-state detection for power pause entry
// The motor settles into a ~1% (10-unit on 0-1000 scale) oscillation at steady state.
// We sample a rolling window and declare steady when max-min spread is within the threshold.
#define IDLE_SPEED_STABLE_WINDOW    40      // Ring buffer depth (40 samples = 200 ms at 200 Hz)
#define IDLE_SPEED_STABLE_SPREAD    14      // Max-min spread (0-1000) that counts as stable (~1.4%)
// Spike threshold: linearly interpolated between 3 PSI and MAX_PSI_THRESHOLD
//   At 3.0 PSI  → 10 units (1.0%)
//   At 8.5 PSI  → 30 units (3.0%)
#define IDLE_SPIKE_UNITS_AT_3PSI    20.0f   // Spike threshold at 3 PSI (units 0-1000)
#define IDLE_SPIKE_UNITS_AT_MAX     60.0f   // Spike threshold at MAX_PSI_THRESHOLD

// Idle hold phase (2.5 PSI) — uses same window/spread but fixed spike threshold
// at the 3 PSI floor value since IDLE_TARGET_PSI is below 3 PSI.
#define IDLE_HOLD_STABLE_SECONDS    1       // Seconds stable at idle speed before entering HOLD
#define IDLE_HOLD_SPIKE_UNITS       10.0f   // Spike threshold during HOLD (matches 3 PSI entry floor)
#define IDLE_RAMP_TIMEOUT_SECONDS      8       // Max seconds in PID ramp phase before forcing to HOLD
#define IDLE_RAMP_LOCKOUT_SECONDS      1       // Lockout at low PSI (~3 PSI): no exit check for first N seconds
#define IDLE_RAMP_LOCKOUT_SECONDS_MAX  5       // Lockout at high PSI (MAX_PSI_THRESHOLD): scales linearly between these two

// ============================================================================
// Temperature Sensor
// ============================================================================
// Conversion formula (applied in tempSensorReadC()):
//   tempC = -1.75 * adcValue + 207.0
//
// Two-level temperature protection:
//   WARNING  : >= TEMP_WARN_SETPOINT  (230 F / 110.0 C)  — motor continues, overlay shown
//   SHUTDOWN : >= TEMP_SHUTDOWN_SETPOINT (266 F / 130.0 C) — motor stops, restart required
#define TEMP_WARN_SETPOINT          110.0f  // Filter-check warning threshold (°C) = 230 °F
#define TEMP_SHUTDOWN_SETPOINT      400.0f  // Hard-shutdown threshold (°C) = 266 °F
//#define TEMP_WARN_SETPOINT          400.0f  // Set very high to disable during testing
//#define TEMP_SHUTDOWN_SETPOINT      410.0f  // Set very high to disable during testing

// ============================================================================
// Target Pressure (user-facing)
// ============================================================================
#define TARGET_PSI_MIN          0.0f    // Minimum user-selectable pressure
#define TARGET_PSI_MAX          11.0f    // Encoder upper bound (beyond MAX_PSI_THRESHOLD = MAX mode)
#define TARGET_PSI_DEFAULT      0.0f
#define TARGET_PSI_STEP         0.1f
#define MAX_PSI_THRESHOLD       12.0f    // At or above this value: motor runs at 100% (MAX mode)

// ============================================================================
// Power Pause Settings
// 100% = POWER_PAUSE_PCT_BASE seconds; three fixed steps: 100%, 125%, 150%
// ============================================================================
#define POWER_PAUSE_PCT_BASE    60      // Seconds corresponding to 100%
#define POWER_PAUSE_SEC_MIN     15      // 100% (minimum / default)
#define POWER_PAUSE_SEC_MAX     90      // 150% (maximum)
#define POWER_PAUSE_SEC_STEP    15      // Encoder step: 15 s = 25% per detent
#define POWER_PAUSE_WARN_SEC    10      // Fixed warning countdown (seconds)

// Idle entry deviation tuning bounds
#define IDLE_DEV_MIN            0.05f
#define IDLE_DEV_MAX            2.0f

// Secret menu — Power Pause Sensitivity multiplier (applied to spike threshold)
// Stored as integer percentage; 100 = 1.0x (default, no change to threshold)
// Lower % → smaller threshold → more sensitive (exits PP sooner)
// Higher % → larger threshold → less sensitive (stays in PP longer)
#define PP_SENSITIVITY_DEFAULT  100     // 100% = 1.0x multiplier
#define PP_SENSITIVITY_MIN      10      // 10% = 0.1x multiplier (most sensitive)
#define PP_SENSITIVITY_MAX      300     // 300% = 3.0x multiplier (least sensitive)

// Self-adjusting PowerPause hold speed
// Motor goes straight to PP_HOLD_SPEED_DEFAULT (25%) on entry instead of PID ramping.
// After each HOLD exit the speed is nudged ±PP_HOLD_SPEED_STEP if the settled
// pressure was outside [PP_SETTLE_LOW_PSI, PP_SETTLE_HIGH_PSI], and saved to flash.
#define PP_HOLD_SPEED_DEFAULT       250     // 25% of 1000 (initial pause speed)
#define PP_HOLD_SPEED_MIN           50      // 5%  minimum (avoid stall)
#define PP_HOLD_SPEED_MAX           700     // 70% maximum
#define PP_HOLD_SPEED_STEP          20      // 2% nudge per pause cycle
#define PP_SETTLE_LOW_PSI           2.4f    // Lower bound of acceptable settled pressure
#define PP_SETTLE_HIGH_PSI          2.8f    // Upper bound of acceptable settled pressure

// Pressure stability detection during RAMP phase
#define PP_PRESSURE_STABLE_WINDOW   40      // Ring buffer depth (40 samples = 200 ms @ 200 Hz)
#define PP_PRESSURE_STABLE_BAND_PSI 0.2f    // Max-min spread (PSI) to declare pressure stable
#define PP_PRESSURE_STABLE_SECONDS  0.5f    // Seconds of continuous stability → enter HOLD
#define PP_RAMP_LOCKOUT_SECONDS     2.0f    // No trigger-pull exit for first N seconds of RAMP
#define PP_HOLD_LOCKOUT_SECONDS     2.0f    // No exit check for first N seconds after entering HOLD
#define PP_SENSITIVITY_STEP     10      // 10% per encoder detent

// Secret menu — max user-settable system hours
#define SECRET_HOURS_MAX        9999    // Maximum hours that can be entered via secret menu

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
#define MENU_OPTION_COUNT       6   // START MOTOR, SETTINGS, TIMERS, SUPPORT, ABOUT, FW UPDATE
#define MENU_OPTION_HEIGHT      34  // reduced from 40 to fit 6 items above footer
#define MENU_TOP_Y              72  // reduced from 90 to fit 6 items above footer
#define SETTINGS_OPTION_COUNT   5
#define SETTINGS_OPTION_HEIGHT  44
#define SETTINGS_TOP_Y          55
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
#define COLOR_MENU_SELECT       TFT_YELLOW
#define COLOR_MENU_EDIT         TFT_ORANGE

// ============================================================================
// Polling / Update Intervals (ms)
// ============================================================================
#define DISPLAY_PRESSURE_INTERVAL_MS    100     // Pressure display refresh (10 Hz)
#define SERIAL_DEBUG_INTERVAL_MS        1000    // Serial debug output
#define TEMP_READ_INTERVAL_MS           1000    // Temperature polling
#define DEBUG_REPORT_INTERVAL_MS        3000    // Motor control debug report

// ============================================================================
// PSI Display Mode
// ============================================================================
// By default the SYSTEM PRESSURE zone always shows the actual measured PSI in
// green.  Define SHOW_SET_PREVIEW (via build_flags or here) to restore the
// alternate behaviour where turning the encoder briefly shows the set-point
// value for 1 second before reverting to the actual reading.
//
// To enable: add   -DSHOW_SET_PREVIEW   to build_flags in platformio.ini
// or uncomment:
//#define SHOW_SET_PREVIEW 0

// ============================================================================
// OTA Firmware Update
// ============================================================================
#define OTA_AP_SSID             "ApolloUpdate"  // WiFi AP SSID for captive portal
#define OTA_AP_IP               "192.168.4.1"   // AP gateway IP (fallback if captive portal redirect fails)
#define OTA_AP_CHANNEL          1               // WiFi AP channel
#define OTA_PORTAL_PORT         80              // Captive portal HTTP port
#define OTA_DNS_PORT            53              // Captive portal DNS port
#define OTA_WIFI_TIMEOUT_MS     15000           // STA connection timeout (ms)

// Manifest URL — JSON file with {"version":"x.y.z","url":"http://...\/firmware.bin"}
// For local testing:  http://<your-pc-ip>:8080/manifest.json
// For production:     https://releases.yourdomain.com/apollo/manifest.json
#define OTA_MANIFEST_URL        "http://18.191.98.191:8080/manifest.json"

// Remote telemetry log endpoint (log_server.py running on the same EC2 instance)
// Set OTA_LOG_API_KEY to the same value as API_KEY in apollo-logserver.service
#define OTA_LOG_URL             "http://18.191.98.191:8081/log"
#define OTA_LOG_API_KEY         "apollo-secret-key"   // ← change before production

#define OTA_ROLLBACK_TIMEOUT_S  60             // Seconds before auto-rollback

// ============================================================================
// Voltage Sensor (Mains monitoring)
// ============================================================================
// 16V AC transformer secondary → voltage divider → 1V at full (115V) mains.
// Scale: 1V on ADC input = VOLTAGE_MAINS_SCALE V of mains.
#define VOLTAGE_SENSOR_PIN      16      // IO16 - Mains voltage divider ADC input
#define VOLTAGE_MAINS_SCALE     161.0f  // V_mains per V_adc (115 * 1.4 calibration factor)

// ============================================================================
// Debug Flags
// ============================================================================
#define DEBUG_SERIAL_OUTPUT     1   // 1 = enable serial debug in loop()
// Add -DTRIAC_DEBUG_SERIAL to build_flags to enable triac ISR diagnostics
// Add -DSIMULATE_AC_60HZ    to build_flags to simulate zero crossings

// Set to 1 to replace the HOURS display in the bottom-right info bar with the
// live mains voltage reading (useful for verifying the voltage divider circuit).
// Set to 0 (or comment out) before production builds.
#define DEBUG_MAINS_VOLTAGE     1

// Set to 1 to enable the overlay preview carousel on the main menu screen.
// Cycles through every warning overlay on a 3-second rotation so the UI can
// be verified without triggering real fault conditions.
// Set to 0 (or comment out) before production builds.
#define DEBUG_OVERLAY_PREVIEW   0

#endif // CONFIG_H
