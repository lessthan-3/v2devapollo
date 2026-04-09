/**
 * @file temp_sensor.c
 * @brief Temperature sensor implementation (ADC-based)
 */

#include "temp_sensor.h"

int analogRead(unsigned char pin);

void tempSensorInit(void) {
  (void)TEMP_SENSOR_PIN;
}

uint16_t tempSensorReadAdc(void) {
  int value = analogRead((unsigned char)TEMP_SENSOR_PIN) / 10;
  if (value < 0) {
    value = 0;
  }
  return (uint16_t)value;
}

float tempSensorReadC(void) {
  uint16_t adcValue = tempSensorReadAdc();
  // Linear calibration: tempC = -1.75 * adcValue + 207.0
  return -1.75f * (float)adcValue + 207.0f;
}
