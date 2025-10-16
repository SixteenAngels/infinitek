/*
  Adafruit_SHT4x.h - Stub implementation for Adafruit SHT4x library
  
  This is a stub implementation to prevent compilation errors when SHT4x
  functionality is enabled but the actual Adafruit SHT4x library is not available.
  
  Copyright (C) 2021  Theo Arends
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#pragma once

#include <Arduino.h>

// Stub sensor event structure
struct sensors_event_t {
  float temperature;
  float relative_humidity;
  uint32_t timestamp;
};

class Adafruit_SHT4x {
public:
  Adafruit_SHT4x() {}
  
  bool begin(uint8_t i2caddr = 0x44) {
    // Stub implementation - always returns false
    return false;
  }
  
  void getEvent(sensors_event_t *humidity, sensors_event_t *temp) {
    // Stub implementation - sets values to 0
    if (humidity) {
      humidity->relative_humidity = 0.0;
      humidity->temperature = 0.0;
      humidity->timestamp = millis();
    }
    if (temp) {
      temp->temperature = 0.0;
      temp->relative_humidity = 0.0;
      temp->timestamp = millis();
    }
  }
  
  float readTemperature() {
    // Stub implementation - returns 0
    return 0.0;
  }
  
  float readHumidity() {
    // Stub implementation - returns 0
    return 0.0;
  }
};