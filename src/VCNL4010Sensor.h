/**
 * VCNL4010Sensor.h
 * 
 * Example implementation for VCNL4010 proximity sensor
 * Shows how to add support for additional sensors
 * 
 * Library: Adafruit VCNL4010
 * Add to platformio.ini:
 * lib_deps = 
 *     adafruit/Adafruit VCNL4010
 */

#ifndef VCNL4010_SENSOR_H
#define VCNL4010_SENSOR_H

#include "proximitySensor.h"
#include <Adafruit_VCNL4010.h>

class VCNL4010Sensor : public ProximitySensor {
public:
  VCNL4010Sensor() : vcnl() {}
  
  bool init() override {
    return vcnl.begin();
  }
  
  bool readProximity(uint16_t& value) override {
    value = vcnl.readProximity();
    // VCNL4010 returns 0 on error, but this is also a valid reading
    // In practice, you might want additional error checking
    return true;
  }
  
  const char* getSensorName() const override {
    return "VCNL4010";
  }
  
private:
  Adafruit_VCNL4010 vcnl;
};

#endif // VCNL4010_SENSOR_H
