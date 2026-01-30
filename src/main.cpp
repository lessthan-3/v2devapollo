#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <ESP32Encoder.h>
#include "pressure_sensor.h"

// Create TFT display instance
TFT_eSPI tft = TFT_eSPI();

// Rotary encoder instance
ESP32Encoder encoder;

// Encoder pins
#define ENCODER_CLK 41  // A (Right)
#define ENCODER_DT  42  // B (Left)
#define ENCODER_BTN 5   // Button

// Backlight pin
#define TFT_BL 21

// Pressure reading interval
#define PRESSURE_READ_INTERVAL_MS 100

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Apollo Sprayers HVLP - ESP32-S3");
  Serial.println("Initializing...");

  // Initialize backlight pin
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);  // Turn on backlight

  // Initialize the display
  tft.init();
  tft.setRotation(1);  // 0=Portrait, 1=Landscape, 2=Portrait inverted, 3=Landscape inverted
  
  // Fill screen with black
  tft.fillScreen(TFT_BLACK);
  
  // Display startup message
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Apollo Sprayers");
  tft.println("HVLP System");
  tft.println("ESP32-S3");
  tft.println("");
  tft.println("Initializing...");
  
  Serial.println("Display initialized successfully!");
  
  // Initialize rotary encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachHalfQuad(ENCODER_DT, ENCODER_CLK);
  encoder.setCount(0);
  
  // Initialize encoder button
  pinMode(ENCODER_BTN, INPUT_PULLUP);
  
  Serial.println("Encoder initialized");
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.println("Encoder: OK");
  
  // Initialize pressure sensor
  delay(100);  // Allow sensor to stabilize
  if (pressureSensor.begin()) {
    Serial.println("Pressure sensor initialized");
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Pressure: OK");
  } else {
    Serial.println("Pressure sensor FAILED");
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("Pressure: FAIL");
  }
  
  delay(2000);
  
  // Clear screen for main display
  tft.fillScreen(TFT_BLACK);
  
  // Draw static labels
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("PRESSURE");
  
  tft.setTextSize(1);
  tft.setCursor(10, 120);
  tft.println("Raw Value:");
  tft.setCursor(10, 140);
  tft.println("Normalized:");
  
  // Encoder labels
  tft.setTextSize(2);
  tft.setCursor(10, 180);
  tft.println("ENCODER");
  tft.setTextSize(1);
  tft.setCursor(10, 210);
  tft.println("Value:");
  tft.setCursor(10, 230);
  tft.println("Button:");
}

void loop() {
  // Read pressure at regular intervals
  static unsigned long lastPressureRead = 0;
  static unsigned long lastUpdate = 0;
  static int counter = 0;
  static float smoothedPressure = 0.0f;
  static int64_t lastEncoderValue = 0;
  static bool lastButtonState = true;  // HIGH when not pressed (INPUT_PULLUP)
  
  // Read encoder
  int64_t encoderValue = encoder.getCount();
  bool buttonPressed = (digitalRead(ENCODER_BTN) == LOW);  // Active low
  
  // Update encoder display if value changed or button state changed
  if (encoderValue != lastEncoderValue || buttonPressed != lastButtonState) {
    lastEncoderValue = encoderValue;
    lastButtonState = buttonPressed;
    
    // Display encoder value
    tft.setTextSize(2);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(80, 210);
    char encStr[16];
    snprintf(encStr, sizeof(encStr), "%6lld  ", (long long)encoderValue);
    tft.print(encStr);
    
    // Display button state
    tft.setCursor(80, 230);
    if (buttonPressed) {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.print("PRESSED ");
    } else {
      tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
      tft.print("Released");
    }
    
    Serial.printf("Encoder: %lld | Button: %s\n", 
                  (long long)encoderValue, 
                  buttonPressed ? "PRESSED" : "Released");
  }
  
  // Read pressure sensor
  if (millis() - lastPressureRead >= PRESSURE_READ_INTERVAL_MS) {
    lastPressureRead = millis();
    
    PressureReading reading = pressureSensor.readPressure();
    
    if (reading.valid) {
      // Simple smoothing (moving average)
      smoothedPressure = (smoothedPressure * 2.0f + reading.pressurePsi) / 3.0f;
      
      // Update pressure display
      tft.setTextSize(4);
      tft.setTextColor(TFT_CYAN, TFT_BLACK);
      tft.setCursor(10, 50);
      
      // Format: XX.X PSI
      char pressureStr[16];
      snprintf(pressureStr, sizeof(pressureStr), "%5.1f PSI  ", smoothedPressure);
      tft.print(pressureStr);
      
      // Display raw and normalized values (smaller text)
      tft.setTextSize(1);
      tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
      
      tft.setCursor(80, 120);
      char rawStr[16];
      snprintf(rawStr, sizeof(rawStr), "%8ld    ", reading.rawValue);
      tft.print(rawStr);
      
      tft.setCursor(80, 140);
      char normStr[16];
      snprintf(normStr, sizeof(normStr), "%7.4f    ", reading.normalizedValue);
      tft.print(normStr);
      
      // Serial output for debugging
      Serial.printf("Pressure: %.2f PSI (%.3f Bar) | Raw: %ld | Norm: %.4f\n",
                    reading.pressurePsi, reading.pressureBar, 
                    reading.rawValue, reading.normalizedValue);
    } else {
      // Display error
      tft.setTextSize(2);
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setCursor(10, 50);
      tft.print("ERROR: ");
      tft.print(reading.errorCode);
      tft.print("  ");
      
      Serial.printf("Pressure read error: %d\n", reading.errorCode);
    }
  }
  
  // Update counter and uptime display every second
  if (millis() - lastUpdate >= 1000) {
    lastUpdate = millis();
    counter++;
    
    // Display uptime
    tft.setTextSize(1);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(10, 460);
    tft.print("Uptime: ");
    tft.print(millis() / 1000);
    tft.print("s   ");
    
    // Toggle indicator to show loop is running
    static bool toggle = false;
    toggle = !toggle;
    tft.fillRect(300, 10, 10, 10, toggle ? TFT_GREEN : TFT_BLACK);
  }
}