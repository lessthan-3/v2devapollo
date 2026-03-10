/**
 * @file pressure_sensor.cpp
 * @brief WF100DPZ Digital Pressure Sensor Driver Implementation
 */

#include "pressure_sensor.h"
#include "driver/gpio.h"  // For gpio_set_drive_capability

// Global instance
PressureSensor pressureSensor;

PressureSensor::PressureSensor() 
    : initialized(false)
    , lastError(PRESSURE_OK)
    , continuousMode(false)
    , zeroPointRaw(PRESSURE_ZERO_POINT)
    , pressure24bitMax(PRESSURE_24BIT_MAX) {
}

bool PressureSensor::begin() {
    Serial.println("Pressure sensor: Starting initialization...");
    
    // Debug: Check pin states before I2C initialization to detect stuck bus or missing pullups
    // Reset pins first
    pinMode(PRESSURE_SDA_PIN, INPUT);
    pinMode(PRESSURE_SCL_PIN, INPUT);
    delay(10);
    
    int sda = digitalRead(PRESSURE_SDA_PIN);
    int scl = digitalRead(PRESSURE_SCL_PIN);
    
    Serial.printf("DEBUG: Pre-init Pin States - SDA: %s, SCL: %s\n", 
                  sda == HIGH ? "HIGH" : "LOW", 
                  scl == HIGH ? "HIGH" : "LOW");

    // If bus is stuck low, attempt recovery
    if (sda == LOW || scl == LOW) {
        Serial.println("DEBUG: I2C Bus stuck LOW. Attempting recovery sequence...");
        
        // Try to toggle SCL to release stuck SDA
        pinMode(PRESSURE_SDA_PIN, INPUT);
        pinMode(PRESSURE_SCL_PIN, OUTPUT);
        
        // 9 clocks to flush out any stuck slave
        for(int i = 0; i < 9; i++) {
            digitalWrite(PRESSURE_SCL_PIN, LOW);
            delayMicroseconds(10);
            digitalWrite(PRESSURE_SCL_PIN, HIGH);
            delayMicroseconds(10);
        }
        
        // Generate STOP condition
        pinMode(PRESSURE_SDA_PIN, OUTPUT);
        digitalWrite(PRESSURE_SDA_PIN, LOW);
        delayMicroseconds(10);
        digitalWrite(PRESSURE_SCL_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(PRESSURE_SDA_PIN, HIGH);
        delayMicroseconds(10);
        
        // Check again
        pinMode(PRESSURE_SCL_PIN, INPUT_PULLUP);
        pinMode(PRESSURE_SDA_PIN, INPUT_PULLUP);
        delay(10);
        sda = digitalRead(PRESSURE_SDA_PIN);
        scl = digitalRead(PRESSURE_SCL_PIN);
        Serial.printf("DEBUG: Post-recovery Pin States - SDA: %s, SCL: %s\n", 
                      sda == HIGH ? "HIGH" : "LOW", 
                      scl == HIGH ? "HIGH" : "LOW");
    }

    // CSB pin: HIGH or floating = I2C mode, LOW = SPI mode
    // Try pulling HIGH first since floating may not be reliable on all boards
    // pinMode(PRESSURE_CSB_PIN, OUTPUT);
    // digitalWrite(PRESSURE_CSB_PIN, HIGH);  // Explicitly set HIGH for I2C mode
    
    // Serial.printf("Pressure sensor: CSB pin %d configured as floating input for I2C mode\n", 
    //               PRESSURE_CSB_PIN);

    //csb physically disonnected on PCB, no longer need to pull high
    
    // Allow sensor time to recognize I2C mode and stabilize
    delay(100);
    
    // End any existing Wire instance before reinitializing
    Wire.end();
    delay(10);
    
    // Initialize I2C with custom pins
    // ESP32-S3 requires explicit pin configuration
    if (!Wire.begin(PRESSURE_SDA_PIN, PRESSURE_SCL_PIN)) {
        lastError = PRESSURE_ERR_I2C_BEGIN;
        Serial.println("Pressure sensor: Wire.begin() failed");
        return false;
    }
    
    Wire.setClock(10000);  // 10kHz I2C clock (standard mode)
    Wire.setTimeOut(1000);  // Increase timeout to 1 second
    
    // Give I2C bus time to stabilize
    delay(50);
    uint8_t error = 1;
    uint32_t scanStart = millis();
    
    // Scan for the device on the I2C bus (with timeout)
    while (error != 0 && (millis() - scanStart) < 500) {
        Wire.beginTransmission(PRESSURE_I2C_ADDR);
        // pinMode(PRESSURE_SDA_PIN, INPUT);
        // pinMode(PRESSURE_SCL_PIN, INPUT);
        // delay(50); // Removed delay between begin and end transmission
        error = Wire.endTransmission();
        if (error == 0) {
            break;  // Device found
        }
        delay(100);
    }

    if (error != 0) {
        lastError = PRESSURE_ERR_I2C_BEGIN;
        Serial.println("Pressure sensor: I2C scan timeout");
        return false;
    }
        
    // Serial.printf("DEBUG: Wire.endTransmission() returned %d at address 0x%02X\n", error, PRESSURE_I2C_ADDR);
    
    
    // Serial.printf("Pressure sensor: Found at address 0x%02X\n", PRESSURE_I2C_ADDR);
    
    // Test communication by reading status register
    uint8_t status;
    if (!readRegister(REG_STATUS, &status)) {
        lastError = PRESSURE_ERR_I2C_BEGIN;
        Serial.println("Pressure sensor: Failed to read status register");
        return false;
    }
    
    Serial.printf("Pressure sensor: Status register = 0x%02X\n", status);
    
    initialized = true;
    continuousMode = false;
    lastError = PRESSURE_OK;

    if (!calibrateZeroPoint()) {
        Serial.println("Pressure sensor: Zero calibration failed, using defaults");
    } else {
        Serial.printf("Pressure sensor: Zero calibration raw=%ld, max=%ld\n",
                      (long)zeroPointRaw, (long)pressure24bitMax);
    }

    Serial.println("Pressure sensor: Initialized successfully");
    return true;
}

bool PressureSensor::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(PRESSURE_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    uint8_t result = Wire.endTransmission();
    
    if (result != 0) {
        lastError = PRESSURE_ERR_I2C_WRITE;
        Serial.printf("Pressure sensor: I2C write error %d\n", result);
        return false;
    }
    return true;
}

bool PressureSensor::readRegister(uint8_t reg, uint8_t* value) {
    // Write register address with full stop (not repeated start)
    // Some I2C devices and ESP32 have issues with repeated start
    Wire.beginTransmission(PRESSURE_I2C_ADDR);
    Wire.write(reg);
    uint8_t result = Wire.endTransmission(true);  // Full stop
    
    if (result != 0) {
        lastError = PRESSURE_ERR_I2C_WRITE;
        return false;
    }
    
    // Small delay between write and read
    delayMicroseconds(100);
    
    // Read value
    uint8_t bytesReceived = Wire.requestFrom((uint8_t)PRESSURE_I2C_ADDR, (uint8_t)1, (uint8_t)true);
    if (bytesReceived != 1) {
        lastError = PRESSURE_ERR_I2C_READ;
        return false;
    }
    
    *value = Wire.read();
    return true;
}

bool PressureSensor::readRegisters(uint8_t startReg, uint8_t* buffer, uint8_t length) {
    // Write starting register address with full stop
    Wire.beginTransmission(PRESSURE_I2C_ADDR);
    Wire.write(startReg);
    uint8_t result = Wire.endTransmission(true);  // Full stop
    
    if (result != 0) {
        lastError = PRESSURE_ERR_I2C_WRITE;
        return false;
    }
    
    // Small delay between write and read
    delayMicroseconds(100);
    
    // Read multiple bytes
    uint8_t bytesReceived = Wire.requestFrom((uint8_t)PRESSURE_I2C_ADDR, length, (uint8_t)true);
    if (bytesReceived != length) {
        lastError = PRESSURE_ERR_I2C_READ;
        Serial.printf("Pressure sensor: Expected %d bytes, got %d\n", length, bytesReceived);
        return false;
    }
    
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }
    
    return true;
}

bool PressureSensor::triggerConversion() {
    if (!initialized) {
        lastError = PRESSURE_ERR_NOT_INITIALIZED;
        return false;
    }
    
    return writeRegister(REG_CMD, CMD_SINGLE_OUTPUT);
}

bool PressureSensor::isDataReady() {
    if (!initialized) {
        return false;
    }
    
    uint8_t status;
    if (!readRegister(REG_STATUS, &status)) {
        return false;
    }
    
    // Bit 0 indicates conversion complete
    // Note: Status is auto-cleared after read
    return (status & STATUS_CONVERSION_DONE) != 0;
}

int32_t PressureSensor::convertRawToSigned(uint8_t msb, uint8_t csb, uint8_t lsb) {
    // Combine bytes into 24-bit UNSIGNED value
    // Per WF100DPZ datasheet, the raw value is unsigned with zero point at 8388608 (2^23)
    // Values below 8388608 = below zero pressure
    // Values above 8388608 = above zero pressure
    uint32_t rawValue = ((uint32_t)msb << 16) | ((uint32_t)csb << 8) | (uint32_t)lsb;
    
    // Return as signed for easier arithmetic with zero point subtraction
    return (int32_t)rawValue;
}

float PressureSensor::normalizedToBar(float normalized) {
    // Convert normalized value to Bar (sensor range: 0 to 2 Bar gauge)
    // Normalized value of 0 = 0 Bar, 1.0 = 2 Bar
    return normalized * PRESSURE_RANGE_BAR;
}

float PressureSensor::normalizedToPsi(float normalized) {
    // Convert normalized value to PSI
    return normalized * PRESSURE_RANGE_PSI;
}

PressureReading PressureSensor::readPressureData() {
    PressureReading reading;
    reading.valid = false;
    reading.errorCode = PRESSURE_OK;
    reading.rawValue = 0;
    reading.normalizedValue = 0.0f;
    reading.pressureBar = 0.0f;
    reading.pressurePsi = 0.0f;
    
    if (!initialized) {
        reading.errorCode = PRESSURE_ERR_NOT_INITIALIZED;
        return reading;
    }
    
    // Read 3 bytes starting from pressure MSB register
    uint8_t buffer[3];
    if (!readRegisters(REG_PRESSURE_MSB, buffer, 3)) {
        reading.errorCode = lastError;
        return reading;
    }
    
    // Convert raw bytes to signed 24-bit value
    reading.rawValue = convertRawToSigned(buffer[0], buffer[1], buffer[2]);

    // Serial.printf("Pressure sensor: Raw value = %ld (0x%06lX), Zero point = %ld\n", 
    //               (long)reading.rawValue, (unsigned long)(reading.rawValue & 0xFFFFFF),
    //               (long)PRESSURE_24BIT_MAX);
    
    // Calculate normalized value
    // Based on datasheet:
    // - Zero condition = 8388608 (2^23)
    // - Values below 8388608 are below zero pressure
    // - Values above 8388608 are above zero pressure
    // 
    // normalized = (rawValue - ZERO_POINT) / FULL_SCALE
    int32_t fullScale = pressure24bitMax > 0 ? pressure24bitMax : PRESSURE_24BIT_MAX;
    // if (reading.rawValue < PRESSURE_24BIT_MAX/2) {
    //     reading.normalizedValue = (float)(reading.rawValue - zeroPointRaw) / (fullScale) * 1.26f;
    // } else {
    //     reading.normalizedValue = (float)-(zeroPointRaw - reading.rawValue) / (fullScale) * 1.26f;
    // }

    //from zero point to rollover point 
    if (reading.rawValue >= zeroPointRaw-PRESSURE_23BIT_MAX) {
        reading.normalizedValue = (float)(reading.rawValue - zeroPointRaw) / (PRESSURE_24BIT_MAX);
    } else { //rolled over starting from zero 
        reading.normalizedValue = (float)(reading.rawValue  + PRESSURE_24BIT_MAX - zeroPointRaw) / (PRESSURE_24BIT_MAX);
    }

    
    // Convert to pressure units
    reading.pressureBar = normalizedToBar(reading.normalizedValue);
    reading.pressurePsi = normalizedToPsi(reading.normalizedValue);
    
    // Clamp negative values for gauge sensor (can't have negative gauge pressure in normal operation)
    if (reading.pressurePsi < 0.0f) {
        reading.pressurePsi = 0.0f;
    }
    if (reading.pressureBar < 0.0f) {
        reading.pressureBar = 0.0f;
    }
    
    reading.valid = true;
    return reading;
}

PressureReading PressureSensor::readPressure() {
    PressureReading reading;
    reading.valid = false;
    reading.errorCode = PRESSURE_OK;
    
    if (!initialized) {
        reading.errorCode = PRESSURE_ERR_NOT_INITIALIZED;
        return reading;
    }
    
    // Send single conversion command
    if (!writeRegister(REG_CMD, CMD_SINGLE_OUTPUT)) {
        reading.errorCode = lastError;
        return reading;
    }
    
    // Wait for conversion to complete with timeout
    uint32_t startTime = millis();
    while (!isDataReady()) {
        if (millis() - startTime > CONVERSION_TIMEOUT_MS) {
            reading.errorCode = PRESSURE_ERR_TIMEOUT;
            Serial.println("Pressure sensor: Conversion timeout");
            return reading;
        }
        delay(1);  // Small delay to avoid hammering the I2C bus
    }
    
    // Read the pressure data
    return readPressureData();
}

bool PressureSensor::startContinuousMode(uint8_t intervalCmd) {
    if (!initialized) {
        lastError = PRESSURE_ERR_NOT_INITIALIZED;
        return false;
    }
    
    if (writeRegister(REG_CMD, intervalCmd)) {
        continuousMode = true;
        return true;
    }
    return false;
}

bool PressureSensor::stopContinuousMode() {
    // To stop continuous mode, we need to reinitialize the sensor
    // For now, just clear the flag - a full reset would require power cycling
    continuousMode = false;
    return true;
}

bool PressureSensor::calibrateZeroPoint() {
    if (!initialized) {
        lastError = PRESSURE_ERR_NOT_INITIALIZED;
        return false;
    }

    if (!writeRegister(REG_CMD, CMD_SINGLE_OUTPUT)) {
        return false;
    }

    uint32_t startTime = millis();
    while (!isDataReady()) {
        if (millis() - startTime > CONVERSION_TIMEOUT_MS) {
            lastError = PRESSURE_ERR_TIMEOUT;
            return false;
        }
        delay(1);
    }

    uint8_t buffer[3];
    if (!readRegisters(REG_PRESSURE_MSB, buffer, 3)) {
        return false;
    }

    int32_t raw = convertRawToSigned(buffer[0], buffer[1], buffer[2]);
    if (raw <= 0) {
        return false;
    }

    zeroPointRaw = raw;
    pressure24bitMax = raw * 2;
    return true;
}
