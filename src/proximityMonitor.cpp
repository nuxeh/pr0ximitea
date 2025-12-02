/**
 * proximityMonitor.cpp
 *
 * Implementation of proximity monitoring with EWMA baseline tracking
 */

#include "proximityMonitor.h"

#ifdef PROXIMITEA_PLOTMSG_DEBUG
#include "plotMsg.hpp"

PlotMsg::PlotMsg PlotMsgBuilder;
#endif

ProximityMonitor::ProximityMonitor(ProximitySensor* sensor, int outputPin)
  : sensor(sensor)
  , outputPin(outputPin)
  , detectionSigma(DEFAULT_DETECTION_SIGMA)
  , minDetectionDuration(DEFAULT_MIN_DETECTION_DURATION)
  , ewmaAlpha(DEFAULT_EWMA_ALPHA)
  , updateInterval(DEFAULT_UPDATE_INTERVAL)
  , debugInterval(DEFAULT_DEBUG_INTERVAL)
  , debugEnabled(false)
  , baseline(0.0f)
  , variance(100.0f)
  , objectPresent(false)
  , detectionStartTime(0)
  , lastReading(0)
  , lastUpdate(0)
  , lastDebugPrint(0)
  , callback(nullptr)
  , baselineLockoutUntil(0)
  , stableBaselineSamples(0)
  , baselineMode(BASELINE_VARIANCE)
  , madIndex(0)
  , madFilled(false)
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
  if (currentTime - lastUpdate < updateInterval)
    return;
  lastUpdate = currentTime;

  bool proximityDetected = checkProximity();

  // ENTER detection
  if (proximityDetected && !objectPresent) {
    if (detectionStartTime == 0) {
      detectionStartTime = currentTime;
    } else if (currentTime - detectionStartTime >= minDetectionDuration) {
      objectPresent = true;

      // NEW: freeze baseline/variance until object is removed
      baselineLockoutUntil = millis() + BASELINE_LOCKOUT_MS;

      if (outputPin >= 0) digitalWrite(outputPin, HIGH);
      if (callback) callback(true);
      Serial.println("Object DETECTED");
    }

  // EXIT detection
  } else if (!proximityDetected && objectPresent) {
    objectPresent = false;
    detectionStartTime = 0;

    // NEW: lockout after removal
    baselineLockoutUntil = millis() + BASELINE_LOCKOUT_MS;

    if (outputPin >= 0) digitalWrite(outputPin, LOW);
    if (callback) callback(false);
    Serial.println("Object REMOVED");

  } else if (!proximityDetected) {
    detectionStartTime = 0;
  }
}

bool ProximityMonitor::checkProximity() {
  uint16_t proximityData = 0;
  unsigned long currentTime = millis();

  bool readSuccess = sensor->readProximity(proximityData);

  if (debugEnabled && (currentTime - lastDebugPrint >= debugInterval)) {
    printDebugInfo(readSuccess, (float)proximityData);
    lastDebugPrint = currentTime;
  }
  if (!readSuccess) return false;

  lastReading = proximityData;
  float reading = (float)proximityData;

  float stdDev = sqrt(variance);

  // NEW: hysteresis thresholds
  float thresholdOn  = baseline + detectionSigmaOn  * stdDev;
  float thresholdOff = baseline + detectionSigmaOff * stdDev;

  // NEW: only try to update baseline when it’s safe
  updateBaseline(reading, thresholdOff);

  // Recompute after update
  stdDev = sqrt(variance);
  thresholdOn  = baseline + detectionSigmaOn  * stdDev;
  thresholdOff = baseline + detectionSigmaOff * stdDev;

  // NEW: use correct threshold depending on state
  if (objectPresent)
    return (reading > thresholdOff);
  else
    return (reading > thresholdOn);
}

void ProximityMonitor::updateBaseline(float reading, float threshold) {
    unsigned long now = millis();

    // Freeze during presence or lockout
    if (objectPresent) return;
    if (now < baselineLockoutUntil) return;

    // Only update when safely below threshold
    if (reading >= threshold) {
        stableBaselineSamples = 0;
        return;
    }

    // Multiple stable samples required
    if (++stableBaselineSamples < BASELINE_STABLE_REQUIRED) {
        return;
    }

    // -----------------------
    // MODE 1: EWMA + VARIANCE
    // -----------------------
    if (baselineMode == BASELINE_VARIANCE) {
        float delta = reading - baseline;
        baseline += ewmaAlpha * delta;

        const float MAX_DELTA = 8.0f;
        float d = fabs(delta);
        if (d > MAX_DELTA) d = MAX_DELTA;

        variance = (1.0f - ewmaAlpha) * variance + ewmaAlpha * (d * d);

        const float MAX_VARIANCE = 600.0f;
        if (variance > MAX_VARIANCE) variance = MAX_VARIANCE;

        return;
    }

    // ---------------------------
    // MODE 2: MAD SLIDING WINDOW
    // ---------------------------
    if (baselineMode == BASELINE_MAD) {
        // Insert reading into ring buffer
        madWindow[madIndex] = reading;
        madIndex = (madIndex + 1) % MAD_WINDOW_SIZE;
        if (madIndex == 0) madFilled = true;

        // Compute median + MAD-based std
        float stdEst;
        baseline = computeMADBaselineAndStd(stdEst);

        // Reuse existing variance member to avoid breaking code
        variance = stdEst * stdEst;

        return;
    }
}

#ifdef PROXIMITEA_PLOTMSG_DEBUG
#warning Pr0ximitea using plotMsg formatted debugging output
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
  Serial.print("] ");

  PlotMsgBuilder.init();
  PlotMsgBuilder.add("raw", sensor->getRaw());
  PlotMsgBuilder.add("prox", reading);
  PlotMsgBuilder.add("base", baseline);
  PlotMsgBuilder.add("std", stdDev);
  PlotMsgBuilder.add("thresh", threshold);
  PlotMsgBuilder.add("detect", reading > threshold ? 1 : 0);
  Serial.println(PlotMsgBuilder.get());
}
#else
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
#endif

void ProximityMonitor::setDetectionCallback(DetectionCallback callback) {
  this->callback = callback;
}

static float quickSelect(float* arr, int n, int k) {
    int left = 0, right = n - 1;

    while (true) {
        if (left == right) return arr[left];
        int pivotIndex = (left + right) / 2;
        float pivot = arr[pivotIndex];

        // Partition
        int i = left, j = right;
        while (i <= j) {
            while (arr[i] < pivot) i++;
            while (arr[j] > pivot) j--;
            if (i <= j) {
                float tmp = arr[i];
                arr[i] = arr[j];
                arr[j] = tmp;
                i++; j--;
            }
        }

        if (k <= j) right = j;
        else if (k >= i) left = i;
        else return arr[k];
    }
}

static float medianOf(float* arr, int n) {
    int mid = n / 2;
    return quickSelect(arr, n, mid);
}

float ProximityMonitor::computeMADBaselineAndStd(float& outStd) {
    int n = madFilled ? MAD_WINDOW_SIZE : madIndex;
    if (n == 0) { outStd = 20.0f; return baseline; }

    // Copy data
    float buf[n];
    memcpy(buf, madWindow, n * sizeof(float));

    // Median → baseline
    float med = medianOf(buf, n);

    // Compute absolute deviations
    for (int i = 0; i < n; i++)
        buf[i] = fabs(madWindow[i] - med);

    // MAD
    float mad = medianOf(buf, n);

    // Convert MAD to Gaussian-equivalent std
    outStd = 1.4826f * mad;
    if (outStd < 1.0f) outStd = 1.0f;         // minimum stability
    if (outStd > 30.0f) outStd = 30.0f;       // limit to sane range

    return med;
}

