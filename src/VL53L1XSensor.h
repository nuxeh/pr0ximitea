/**
 * VL53L1XSensor.h
 * 
 * VL53L1X Time-of-Flight distance sensor implementation
 * Measures actual distance and inverts it to proximity value
 * (closer object = higher proximity value, like other sensors)
 * 
 * Library: Pololu VL53L1X
 * Add to platformio.ini:
 * lib_deps = 
 *     pololu/VL53L1X
 */

#ifndef VL53L1X_SENSOR_H
#define VL53L1X_SENSOR_H

#include "proximitySensor.h"
#include <VL53L1X.h>

class VL53L1XSensor : public ProximitySensor {
public:
  /**
   * Distance mode options for VL53L1X
   */
  enum DistanceMode {
    Short = VL53L1X::Short,    // Up to 1.3m, best ambient immunity
    Medium = VL53L1X::Medium,  // Up to 3m
    Long = VL53L1X::Long       // Up to 4m
  };
  
  /**
   * Constructor
   * @param maxDistance Maximum distance in mm (default 1000mm = 1m)
   *                    Distances beyond this will be clamped
   *                    This helps normalize the proximity range
   * @param mode Distance mode (Short, Medium, or Long)
   */
  VL53L1XSensor(uint16_t maxDistance = 1000, DistanceMode mode = Short) 
    : maxDistance(maxDistance)
    , distanceMode(mode)
    , lastDistance(0)
  {
  }
  
  bool init() override {
    vl53.setTimeout(500);
    
    if (!vl53.init()) {
      return false;
    }
    
    // Set distance mode
    vl53.setDistanceMode((VL53L1X::DistanceMode)distanceMode);
    
    // Optional: Set timing budget (microseconds)
    // Options: 15000, 20000, 33000, 50000, 100000, 200000, 500000
    // Lower = faster but less accurate, higher = slower but more accurate
    // Default is typically 50000 (50ms)
    // vl53.setMeasurementTimingBudget(33000);
    
    // Start continuous mode with 50ms between readings
    vl53.startContinuous(50);
    
    return true;
  }
  
  bool readProximity(uint16_t& value) override {
    // Check if data is ready
    if (!vl53.dataReady()) {
      return false;
    }
    
    // Read distance (false = don't block)
    uint16_t distance = vl53.read(false);
    
    // Check for timeout
    if (vl53.timeoutOccurred()) {
      return false;
    }
    
    // Check for valid range status
    // 0 = good, 1-2 = warnings (still usable), 3+ = errors
    if (vl53.ranging_data.range_status > 2) {
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
    return "VL53L1X";
  }
  
  /**
   * Get the configured maximum distance
   */
  uint16_t getMaxDistance() const { return maxDistance; }
  
  /**
   * Get the last measured distance in mm (before inversion)
   * Useful for debugging - call after readProximity()
   */
  uint16_t getRaw() const { return lastDistance; }
  
  /**
   * Get current distance mode
   */
  DistanceMode getDistanceMode() const { return distanceMode; }
  
  /**
   * Stop continuous mode (call before changing settings)
   */
  void stopContinuous() {
    vl53.stopContinuous();
  }
  
  /**
   * Restart continuous mode (call after changing settings)
   * @param period_ms Period between measurements in milliseconds
   */
  void startContinuous(uint16_t period_ms = 50) {
    vl53.startContinuous(period_ms);
  }
  
  /**
   * Change distance mode (requires stop/start continuous)
   */
  void setDistanceMode(DistanceMode mode) {
    stopContinuous();
    distanceMode = mode;
    vl53.setDistanceMode((VL53L1X::DistanceMode)mode);
    startContinuous();
  }
  
  /**
   * Access the underlying sensor object for advanced configuration
   */
  VL53L1X& getSensor() { return vl53; }
  
private:
  VL53L1X vl53;
  uint16_t maxDistance;
  DistanceMode distanceMode;
  uint16_t lastDistance;
};

#endif // VL53L1X_SENSOR_H
