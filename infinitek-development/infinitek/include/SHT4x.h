#pragma once

#include <Adafruit_SHT4x.h>

class SHT4x {
public:
  bool begin() { return sensor.begin(); }
  float getTemperature() {
    sensors_event_t humidity, temp;
    sensor.getEvent(&humidity, &temp);
    return temp.temperature;
  }
  float getHumidity() {
    sensors_event_t humidity, temp;
    sensor.getEvent(&humidity, &temp);
    return humidity.relative_humidity;
  }

private:
  Adafruit_SHT4x sensor;
};

