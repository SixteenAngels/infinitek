#pragma once

#include <Adafruit_SHT31.h>

class SHT3x {
public:
  bool begin(uint8_t i2caddr = 0x44) { return sensor.begin(i2caddr); }
  float getTemperature() { return sensor.readTemperature(); }
  float getHumidity() { return sensor.readHumidity(); }

private:
  Adafruit_SHT31 sensor;
};

