/**
 * proximityMonitor.cpp
 * 
 * Implementation of proximity monitoring with EWMA baseline tracking
 */

#include "proximityMonitor.h"

ProximityMonitor::ProximityMonitor(ProximitySensor* sensor, int outputPin)
  : sensor(sensor)
  , outputPin(outputPin)
  , detectionSigma(DEFAULT_DETECTION_SIGMA)
  , minDetectionDuration(DEFAULT_MIN_DETECTION_DURATION)
  , ewmaAlpha(DEFAULT_EWMA_ALPHA)
  , updateInterval(DEFAULT_UPDATE_INTERVAL)
  , debugInterval(DEFAULT_DEBUG_INTERVAL)
  , debugEnabled(true)
  , baseline(0.0f)
  , variance(100.0f)
  , objectPresent(false)
  , detectionStartTime(0)
  , lastReading(0)
  , lastUpdate(0)
  , lastDebugPrint(0)
  , callback(nullptr)
{
}

bool ProximityMonitor::begin() {
  // Initialize sensor
  if (!sensor->init()) {
    Serial.print("Failed to initialize sensor: ");
    Serial.println(sensor->getSensorName());
    return false;
  }
  
  Serial.print("Initialized sensor: ");
  Serial.println(sensor->getSensorName());
  
  // Setup output pin if provided
  if (outputPin >= 0) {
    pinMode(outputPin, OUTPUT);
    digitalWrite(outputPin, LOW);
  }
  
  // Get initial baseline reading
  delay(100);
  uint16_t initialReading;
  if (sensor->readProximity(initialReading)) {
    baseline = (float)initialReading;
    variance = 100.0f; // Starting variance for initial detection capability
    lastReading = initialReading;
    
    Serial.print("Initial baseline set to: ");
    Serial.println(baseline, 2);
    return true;
  } else {
    Serial.println("Failed to get initial sensor reading!");
    return false;
  }
}

void ProximityMonitor::update() {
  unsigned long currentTime = millis();
  
  // Non-blocking timing check
  if (currentTime - lastUpdate < updateInterval) {
    return;
  }
  lastUpdate = currentTime;
  
  // Check proximity and handle detection
  bool proximityDetected = checkProximity();
  
  // Debouncing logic: require sustained detection
  if (proximityDetected && !objectPresent) {
    if (detectionStartTime == 0) {
      detectionStartTime = currentTime;
    } else if (currentTime - detectionStartTime >= minDetectionDuration) {
      objectPresent = true;
      
      // Set output pin
      if (outputPin >= 0) {
        digitalWrite(outputPin, HIGH);
      }
      
      // Call callback
      if (callback) {
        callback(true);
      }
      
      Serial.println("Object DETECTED");
    }
  } else if (!proximityDetected && objectPresent) {
    objectPresent = false;
    detectionStartTime = 0;
    
    // Clear output pin
    if (outputPin >= 0) {
      digitalWrite(outputPin, LOW);
    }
    
    // Call callback
    if (callback) {
      callback(false);
    }
    
    Serial.println("Object REMOVED");
  } else if (!proximityDetected) {
    detectionStartTime = 0;
  }
}

bool ProximityMonitor::checkProximity() {
  uint16_t proximityData = 0;
  unsigned long currentTime = millis();
  
  // Read sensor
  bool readSuccess = sensor->readProximity(proximityData);
  
  // Debug output
  if (debugEnabled && (currentTime - lastDebugPrint >= debugInterval)) {
    printDebugInfo(readSuccess, (float)proximityData);
    lastDebugPrint = currentTime;
  }
  
  if (!readSuccess) {
    return false;
  }
  
  lastReading = proximityData;
  float reading = (float)proximityData;

  // Update baseline
  float stdDev = sqrt(variance);
  float threshold = baseline + (detectionSigma * stdDev);

  updateBaseline(reading, threshold);

  // Calculate final threshold after potential baseline update
  stdDev = sqrt(variance);
  threshold = baseline + (detectionSigma * stdDev);

  // Detection check
  return reading > threshold;
}

void ProximityMonitor::updateBaseline(float reading, float threshold) {
  // Only update baseline when no object present and reading is below threshold
  if (!objectPresent && (reading < threshold)) {
    // EWMA baseline update
    float delta = reading - baseline;
    baseline += ewmaAlpha * delta;
    
    // EWMA variance update
    float absDelta = abs(delta);
    variance = (1.0f - ewmaAlpha) * variance + ewmaAlpha * absDelta * absDelta;
    
    // Ensure minimum variance (prevents threshold from becoming too sensitive)
    if (variance < 1.0f) {
      variance = 1.0f;
    }
  }
}

void ProximityMonitor::printDebugInfo(bool readSuccess, float reading) {
  if (!readSuccess) {
    Serial.print(sensor->getSensorName());
    Serial.println(" Read Failed! Check wiring/power.");
    return;
  }
  
  float stdDev = sqrt(variance);
  float threshold = baseline + (detectionSigma * stdDev);
  
  Serial.print("[");
  Serial.print(sensor->getSensorName());
  Serial.print("] Prox: ");
  Serial.print(reading, 1);
  Serial.print(" | Base: ");
  Serial.print(baseline, 2);
  Serial.print(" | StdDev: ");
  Serial.print(stdDev, 2);
  Serial.print(" | Thresh: ");
  Serial.print(threshold, 2);
  Serial.print(" | Detected: ");
  Serial.println(reading > threshold ? "YES" : "NO");
}

void ProximityMonitor::setDetectionCallback(DetectionCallback callback) {
  this->callback = callback;
}
