/**
 * @file pressure_sensor.cpp
 * @brief WF100DPZ Digital Pressure Sensor Driver Implementation
 */

#include "pressure_sensor.h"

// Global instance
PressureSensor pressureSensor;

PressureSensor::PressureSensor() 
    : initialized(false)
    , lastError(PRESSURE_OK)
    , continuousMode(false) {
}

bool PressureSensor::begin() {
    // Configure CSB pin as output and pull low for I2C mode
    pinMode(PRESSURE_CSB_PIN, OUTPUT);
    digitalWrite(PRESSURE_CSB_PIN, LOW);
    
    // Small delay to let the sensor recognize I2C mode
    delay(10);
    
    // Initialize I2C with custom pins
    Wire.begin(PRESSURE_SDA_PIN, PRESSURE_SCL_PIN);
    Wire.setClock(100000);  // 100kHz I2C clock (standard mode)
    
    // Test communication by reading status register
    uint8_t status;
    if (!readRegister(REG_STATUS, &status)) {
        lastError = PRESSURE_ERR_I2C_BEGIN;
        Serial.println("Pressure sensor: Failed to communicate");
        return false;
    }
    
    initialized = true;
    continuousMode = false;
    lastError = PRESSURE_OK;
    
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
    // Write register address
    Wire.beginTransmission(PRESSURE_I2C_ADDR);
    Wire.write(reg);
    uint8_t result = Wire.endTransmission(false);  // Repeated start
    
    if (result != 0) {
        lastError = PRESSURE_ERR_I2C_WRITE;
        return false;
    }
    
    // Read value
    uint8_t bytesReceived = Wire.requestFrom(PRESSURE_I2C_ADDR, (uint8_t)1);
    if (bytesReceived != 1) {
        lastError = PRESSURE_ERR_I2C_READ;
        return false;
    }
    
    *value = Wire.read();
    return true;
}

bool PressureSensor::readRegisters(uint8_t startReg, uint8_t* buffer, uint8_t length) {
    // Write starting register address
    Wire.beginTransmission(PRESSURE_I2C_ADDR);
    Wire.write(startReg);
    uint8_t result = Wire.endTransmission(false);  // Repeated start
    
    if (result != 0) {
        lastError = PRESSURE_ERR_I2C_WRITE;
        return false;
    }
    
    // Read multiple bytes
    uint8_t bytesReceived = Wire.requestFrom(PRESSURE_I2C_ADDR, length);
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
    // Combine bytes into 24-bit value
    int32_t rawValue = ((int32_t)msb << 16) | ((int32_t)csb << 8) | (int32_t)lsb;
    
    // Handle sign extension for 24-bit signed value
    // If the MSB bit (bit 23) is set, the value is negative
    if (rawValue & 0x800000) {
        // Negative value: sign extend to 32-bit
        rawValue |= 0xFF000000;
    }
    
    return rawValue;
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
    
    // Calculate normalized value
    // Based on datasheet:
    // - Zero condition = 8388608 (2^23)
    // - Values below 8388608 are below zero pressure
    // - Values above 8388608 are above zero pressure
    // 
    // Formula from datasheet (corrected):
    // For values >= 0 (rawValue >= 8388608): normalized = (rawValue - 8388608) / 8388608
    // For values < 0 (rawValue < 8388608): normalized = (rawValue - 8388608) / 8388608
    // 
    // Simplified: normalized = (rawValue - ZERO_POINT) / FULL_SCALE
    
    reading.normalizedValue = (float)(reading.rawValue - PRESSURE_ZERO_POINT) / PRESSURE_FULL_SCALE;
    
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
