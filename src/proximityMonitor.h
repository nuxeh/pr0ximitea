/**
 * proximityMonitor.h
 * 
 * Handles baseline tracking, debouncing, and detection logic
 * Works with any ProximitySensor implementation
 */

#ifndef PROXIMITY_MONITOR_H
#define PROXIMITY_MONITOR_H

#include <Arduino.h>
#include "proximitySensor.h"

// Configuration defaults (can be overridden in constructor)
#define DEFAULT_DETECTION_SIGMA 3.0f
#define DEFAULT_MIN_DETECTION_DURATION 200
#define DEFAULT_EWMA_ALPHA 0.01f
#define DEFAULT_UPDATE_INTERVAL 50
#define DEFAULT_DEBUG_INTERVAL 500

#define BASELINE_LOCKOUT_MS 400
#define BASELINE_STABLE_REQUIRED 5

#ifndef MAD_WINDOW_SIZE
 #define MAD_WINDOW_SIZE 31 // choose 15–61 depending on MCU
#endif

enum BaselineMode {
    BASELINE_VARIANCE = 0,
    BASELINE_MAD = 1
};

class ProximityMonitor {
public:
  /**
   * Callback function type for detection events
   * @param detected true when object detected, false when removed
   */
  typedef void (*DetectionCallback)(bool detected);
  
  /**
   * Constructor
   * @param sensor Pointer to a ProximitySensor implementation
   * @param outputPin Pin to control when object is detected (optional, -1 to disable)
   */
  ProximityMonitor(ProximitySensor* sensor, int outputPin = -1);
  
  /**
   * Initialize the monitor and sensor
   * @return true on success
   */
  bool begin();
  
  /**
   * Update function - call repeatedly from loop()
   * Non-blocking, handles timing internally
   */
  void update();
  
  /**
   * Set callback for detection events
   */
  void setDetectionCallback(DetectionCallback callback);
  
  /**
   * Configuration setters (call before begin())
   */
  void setDetectionSigma(float sigma) { detectionSigma = sigma; }
  void setMinDetectionDuration(unsigned long ms) { minDetectionDuration = ms; }
  void setEwmaAlpha(float alpha) { ewmaAlpha = alpha; }
  void setUpdateInterval(unsigned long ms) { updateInterval = ms; }
  void setDebugInterval(unsigned long ms) { debugInterval = ms; }
  void enableDebug(bool enable) { debugEnabled = enable; }
  void setBaselineMode(BaselineMode mode) { baselineMode = mode; }
  
  /**
   * Get current state
   */
  bool isObjectPresent() const { return objectPresent; }
  float getBaseline() const { return baseline; }
  float getStdDev() const { return sqrt(variance); }
  float getThreshold() const { return baseline + (detectionSigma * sqrt(variance)); }
  uint16_t getLastReading() const { return lastReading; }
  
private:
  // Sensor interface
  ProximitySensor* sensor;
  int outputPin;
  
  // Configuration
  float detectionSigma;
  unsigned long minDetectionDuration;
  float ewmaAlpha;
  unsigned long updateInterval;
  unsigned long debugInterval;
  bool debugEnabled;
  
  // State tracking
  float baseline;
  float variance;
  bool objectPresent;
  unsigned long detectionStartTime;
  uint16_t lastReading;
  
  // Timing
  unsigned long lastUpdate;
  unsigned long lastDebugPrint;
  
  // Callback
  DetectionCallback callback;
  
  // Internal methods
  bool checkProximity();
  void printDebugInfo(bool readSuccess, float reading);
  void updateBaseline(float reading, float threshold);
  float computeMADBaselineAndStd(float& outStd);

  // Baseline update control
  unsigned long baselineLockoutUntil;
  int stableBaselineSamples;

  // Hysteresis
  float detectionSigmaOn = DEFAULT_DETECTION_SIGMA;
  float detectionSigmaOff = DEFAULT_DETECTION_SIGMA * 0.66f; // e.g. 3.0 on, 2.0 off

  BaselineMode baselineMode;

  // MAD window
  float madWindow[MAD_WINDOW_SIZE];
  int madIndex;
  bool madFilled;
};

#endif // PROXIMITY_MONITOR_H
