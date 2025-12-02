/**
 * proximitySensor.h
 * 
 * Abstract base class for proximity sensors
 * Defines the interface that all sensor implementations must provide
 */

#ifndef PROXIMITY_SENSOR_H
#define PROXIMITY_SENSOR_H

#include <Arduino.h>

class ProximitySensor {
public:
  virtual ~ProximitySensor() {}
  
  /**
   * Initialize the sensor hardware
   * Returns true on success, false on failure
   */
  virtual bool init() = 0;
  
  /**
   * Read current proximity value from sensor
   * @param value Reference to store the reading
   * @return true on successful read, false on error
   */
  virtual bool readProximity(uint16_t& value) = 0;

  virtual uint16_t getRaw() const = 0;
  
  /**
   * Get sensor name for debugging
   */
  virtual const char* getSensorName() const = 0;
};

#endif // PROXIMITY_SENSOR_H
