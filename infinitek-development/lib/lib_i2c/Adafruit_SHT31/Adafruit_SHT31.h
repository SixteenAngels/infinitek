/*
  Adafruit_SHT31.h - Stub implementation for Adafruit SHT31 library
  
  This is a stub implementation to prevent compilation errors when SHT3x
  functionality is enabled but the actual Adafruit SHT31 library is not available.
  
  Copyright (C) 2021  Theo Arends
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#pragma once

#include <Arduino.h>

class Adafruit_SHT31 {
public:
  Adafruit_SHT31() {}
  
  bool begin(uint8_t i2caddr = 0x44) {
    // Stub implementation - always returns false
    return false;
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