/**
 * @file temp_sensor.h
 * @brief Temperature sensor interface (ADC-based)
 */

#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include <stdint.h>

// Temperature sensor pin
#define TEMP_SENSOR_PIN 38  // IO38 - Temperature Sensor (1-Wire treated as ADC)

// Temperature conversion constants
#define TEMP_MULT    -87
#define TEMP_DIVISOR 100
#define TEMP_OFFSET  210

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize temperature sensor (ADC)
 */
void tempSensorInit(void);

/**
 * @brief Read raw ADC value from temperature sensor
 *
 * @return Raw ADC reading
 */
uint16_t tempSensorReadAdc(void);

/**
 * @brief Read temperature in Celsius using linear conversion
 *
 * tempC = (adc - TEMP_OFFSET) * TEMP_MULT / TEMP_DIVISOR
 *
 * @return Temperature in Celsius
 */
float tempSensorReadC(void);

#ifdef __cplusplus
}
#endif

#endif // TEMP_SENSOR_H
