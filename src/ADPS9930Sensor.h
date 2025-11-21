/**
 * APDS9930Sensor.h
 * 
 * APDS-9930 specific implementation of ProximitySensor interface
 * 
 * Library: depau/APDS-9930 Ambient Light and Proximity Sensor
 * Add to platformio.ini:
 * lib_deps = 
 *     https://github.com/depau/APDS9930
 */

#ifndef APDS9930_SENSOR_H
#define APDS9930_SENSOR_H

#include "ProximitySensor.h"
#include <APDS9930.h>

class APDS9930Sensor : public ProximitySensor {
public:
  APDS9930Sensor() : apds() {}
  
  bool init() override {
    if (!apds.init()) {
      return false;
    }
    
    // Enable proximity sensor without interrupts
    return apds.enableProximitySensor(false);
  }
  
  bool readProximity(uint16_t& value) override {
    return apds.readProximity(value);
  }
  
  const char* getSensorName() const override {
    return "APDS-9930";
  }
  
private:
  APDS9930 apds;
};

#endif // APDS9930_SENSOR_H
