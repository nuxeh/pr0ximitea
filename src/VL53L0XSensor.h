/**
 * VL53L0XSensor.h
 * 
 * VL53L0X Time-of-Flight distance sensor implementation
 * Measures actual distance and inverts it to proximity value
 * (closer object = higher proximity value, like other sensors)
 * 
 * Library: Pololu VL53L0X
 * Add to platformio.ini:
 * lib_deps = 
 *     pololu/VL53L0X
 */

#ifndef VL53L0X_SENSOR_H
#define VL53L0X_SENSOR_H

#include "proximitySensor.h"
#include <VL53L0X.h>

class VL53L0XSensor : public ProximitySensor {
public:
  /**
   * Constructor
   * @param maxDistance Maximum distance in mm (default 1000mm = 1m)
   *                    Distances beyond this will be clamped
   *                    This helps normalize the proximity range
   */
  VL53L0XSensor(uint16_t maxDistance = 1000) 
    : maxDistance(maxDistance)
    , lastDistance(0)
  {
  }
  
  bool init() override {
    vl53.setTimeout(500);
    
    if (!vl53.init()) {
      return false;
    }
    
    // Optional: Set timing budget for faster/more accurate readings
    // Options (microseconds): 20000, 33000, 50000, 100000, 200000, 500000
    // Lower = faster but less accurate, higher = slower but more accurate
    // Default is typically 33000 (33ms)
    // vl53.setMeasurementTimingBudget(33000);
    
    // Start continuous ranging mode
    vl53.startContinuous();
    
    return true;
  }
  
  bool readProximity(uint16_t& value) override {
    // Read distance in continuous mode
    uint16_t distance = vl53.readRangeContinuousMillimeters();
    
    // Check for timeout or invalid reading
    if (vl53.timeoutOccurred()) {
      return false;
    }
    
    // Store for debugging
    lastDistance = distance;
    
    // Clamp to max distance
    if (distance > maxDistance) {
      distance = maxDistance;
    }
    
    // Invert: closer = higher value
    // Map [0, maxDistance] to [maxDistance, 0]
    value = maxDistance - distance;
    return true;
  }
  
  const char* getSensorName() const override {
    return "VL53L0X";
  }
  
  /**
   * Get the configured maximum distance
   */
  uint16_t getMaxDistance() const { return maxDistance; }
  
  /**
   * Get the last measured distance in mm (before inversion)
   * Useful for debugging - call after readProximity()
   */
  uint16_t getLastDistanceMM() const { return lastDistance; }
  
  /**
   * Stop continuous mode (call before changing settings)
   */
  void stopContinuous() {
    vl53.stopContinuous();
  }
  
  /**
   * Restart continuous mode (call after changing settings)
   */
  void startContinuous() {
    vl53.startContinuous();
  }
  
  /**
   * Access the underlying sensor object for advanced configuration
   */
  VL53L0X& getSensor() { return vl53; }
  
private:
  VL53L0X vl53;
  uint16_t maxDistance;
  uint16_t lastDistance;
};

#warning Using VL52L0X proximity sensor, ensure lib_deps includes pololu/VL53L0X

#endif // VL53L0X_SENSOR_H
