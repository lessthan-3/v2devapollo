/**
 * @file pressure_sensor.h
 * @brief WF100DPZ Digital Pressure Sensor Driver for ESP32-S3
 * 
 * This module handles I2C communication with the WF100DPZ pressure sensor.
 * Sensor: WF100DPZ 2BG S6 DT (2 Bar Gauge range, ~0-30 PSI)
 * 
 * Pinout:
 *   IO17 - CSB (pulled low for I2C mode)
 *   IO18 - SDA (I2C Data)
 *   IO19 - SCL (I2C Clock)
 */

#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <Arduino.h>
#include <Wire.h>

// Pin definitions
#define PRESSURE_CSB_PIN    17  // Chip select - pull low for I2C mode
#define PRESSURE_SDA_PIN    18  // I2C Data
#define PRESSURE_SCL_PIN    19  // I2C Clock

// I2C Configuration
#define PRESSURE_I2C_ADDR   0x6D  // 7-bit address: 1101101

// Register definitions
#define REG_STATUS          0x02  // Status register (bit0 = conversion complete)
#define REG_PRESSURE_MSB    0x06  // Pressure output MSB
#define REG_PRESSURE_CSB    0x07  // Pressure output CSB
#define REG_PRESSURE_LSB    0x08  // Pressure output LSB
#define REG_CMD             0x30  // Command register

// Command definitions
#define CMD_SINGLE_OUTPUT       0x0A  // Perform one output
#define CMD_CONTINUOUS_OUTPUT   0x0B  // Continuous output
#define CMD_INTERVAL_62_5MS     0x1B  // 62.5ms interval output
#define CMD_INTERVAL_125MS      0x2B  // 125ms interval output
#define CMD_INTERVAL_1S         0xFB  // 1 second interval output

// Pressure calculation constants
#define PRESSURE_ZERO_POINT     8388608   // 2^23 - zero condition
#define PRESSURE_FULL_SCALE     8388608.0f // Divisor for normalization
#define PRESSURE_24BIT_MAX      16777216  // 2^24

// Sensor range: 2BG = 2 Bar Gauge
// 2 Bar = ~29.0075 PSI
#define PRESSURE_RANGE_BAR      2.0f
#define PRESSURE_RANGE_PSI      29.0075f

// Conversion timeout
#define CONVERSION_TIMEOUT_MS   100

// Status flags
#define STATUS_CONVERSION_DONE  0x01

/**
 * @brief Pressure sensor reading result
 */
typedef struct {
    float pressureBar;      // Pressure in Bar
    float pressurePsi;      // Pressure in PSI
    float normalizedValue;  // Normalized value (-1.0 to 1.0)
    int32_t rawValue;       // Raw 24-bit signed value
    bool valid;             // True if reading is valid
    uint8_t errorCode;      // Error code if invalid
} PressureReading;

/**
 * @brief Error codes for pressure sensor
 */
enum PressureSensorError {
    PRESSURE_OK = 0,
    PRESSURE_ERR_I2C_BEGIN = 1,
    PRESSURE_ERR_TIMEOUT = 2,
    PRESSURE_ERR_I2C_WRITE = 3,
    PRESSURE_ERR_I2C_READ = 4,
    PRESSURE_ERR_NOT_INITIALIZED = 5
};

/**
 * @brief Pressure sensor class for WF100DPZ
 */
class PressureSensor {
public:
    /**
     * @brief Constructor
     */
    PressureSensor();

    /**
     * @brief Initialize the pressure sensor
     * @return true if initialization successful, false otherwise
     */
    bool begin();

    /**
     * @brief Read pressure from sensor (blocking, single conversion)
     * @return PressureReading structure with pressure data
     */
    PressureReading readPressure();

    /**
     * @brief Start continuous conversion mode
     * @param intervalCmd Interval command (CMD_CONTINUOUS_OUTPUT, CMD_INTERVAL_62_5MS, etc.)
     * @return true if command sent successfully
     */
    bool startContinuousMode(uint8_t intervalCmd = CMD_INTERVAL_125MS);

    /**
     * @brief Stop continuous mode (sensor will need re-initialization)
     * @return true if successful
     */
    bool stopContinuousMode();

    /**
     * @brief Check if conversion is complete (for non-blocking reads)
     * @return true if data is ready
     */
    bool isDataReady();

    /**
     * @brief Read pressure data without triggering new conversion
     * @return PressureReading structure with pressure data
     */
    PressureReading readPressureData();

    /**
     * @brief Trigger a single conversion (non-blocking)
     * @return true if command sent successfully
     */
    bool triggerConversion();

    /**
     * @brief Get the last error code
     * @return Error code
     */
    uint8_t getLastError() const { return lastError; }

    /**
     * @brief Check if sensor is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return initialized; }

    /**
     * @brief Get raw 24-bit pressure value as signed integer
     * @param msb Most significant byte
     * @param csb Center significant byte  
     * @param lsb Least significant byte
     * @return Signed 24-bit value
     */
    static int32_t convertRawToSigned(uint8_t msb, uint8_t csb, uint8_t lsb);

    /**
     * @brief Convert normalized value to pressure in Bar
     * @param normalized Normalized value from sensor
     * @return Pressure in Bar
     */
    static float normalizedToBar(float normalized);

    /**
     * @brief Convert normalized value to pressure in PSI
     * @param normalized Normalized value from sensor
     * @return Pressure in PSI
     */
    static float normalizedToPsi(float normalized);

private:
    bool initialized;
    uint8_t lastError;
    bool continuousMode;

    /**
     * @brief Write a command to the sensor
     * @param reg Register address
     * @param value Command value
     * @return true if successful
     */
    bool writeRegister(uint8_t reg, uint8_t value);

    /**
     * @brief Read a single register
     * @param reg Register address
     * @param value Pointer to store value
     * @return true if successful
     */
    bool readRegister(uint8_t reg, uint8_t* value);

    /**
     * @brief Read multiple consecutive registers
     * @param startReg Starting register address
     * @param buffer Buffer to store values
     * @param length Number of bytes to read
     * @return true if successful
     */
    bool readRegisters(uint8_t startReg, uint8_t* buffer, uint8_t length);
};

// Global pressure sensor instance
extern PressureSensor pressureSensor;

#endif // PRESSURE_SENSOR_H
