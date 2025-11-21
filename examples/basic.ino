/**
 * example_usage.ino
 * 
 * Demonstrates how to use the generalized proximity sensor library
 * with different sensor implementations
 */

#include "ProximitySensor.h"
#include "ProximityMonitor.h"
#include "APDS9930Sensor.h"
// #include "VCNL4010Sensor.h"  // Alternative sensor

// Choose your sensor implementation
APDS9930Sensor sensor;
// VCNL4010Sensor sensor;  // Swap to this line to use VCNL4010

// Create monitor with sensor and output pin
#define SOLENOID_EN_PIN 2
ProximityMonitor monitor(&sensor, SOLENOID_EN_PIN);

// Optional: Detection callback function
void onDetectionChange(bool detected) {
  if (detected) {
    Serial.println(">>> CALLBACK: Object entered detection zone!");
    // Add custom logic here (e.g., trigger solenoid, log event, etc.)
  } else {
    Serial.println(">>> CALLBACK: Object left detection zone!");
    // Add custom logic for removal
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("\n=== Proximity Monitor Example ===");
  
  // Optional: Configure monitor parameters before begin()
  monitor.setDetectionSigma(3.0);          // 3 sigma detection threshold
  monitor.setMinDetectionDuration(200);     // 200ms sustained detection
  monitor.setEwmaAlpha(0.01);              // 1% baseline update rate
  monitor.setUpdateInterval(50);            // 20Hz update rate
  monitor.setDebugInterval(500);            // Debug every 500ms
  monitor.enableDebug(true);                // Enable debug output
  
  // Optional: Set callback for detection events
  monitor.setDetectionCallback(onDetectionChange);
  
  // Initialize the monitor (also initializes the sensor)
  if (!monitor.begin()) {
    Serial.println("Monitor initialization failed!");
    while (1) delay(10);
  }
  
  Serial.println("Monitor ready. Monitoring for objects...\n");
}

void loop() {
  // Simply call update() - all logic is handled internally
  monitor.update();
  
  // Your other non-blocking code can go here
  // The monitor handles all timing internally
}

/* ============================================================
 * ALTERNATIVE USAGE PATTERNS
 * ============================================================ */

// Example 1: Without output pin control
void example_no_output_pin() {
  APDS9930Sensor sensor;
  ProximityMonitor monitor(&sensor);  // No output pin
  
  monitor.begin();
  
  // In loop:
  // monitor.update();
  // if (monitor.isObjectPresent()) {
  //   // Handle detection in your own way
  // }
}

// Example 2: Manual state checking
void example_manual_state() {
  APDS9930Sensor sensor;
  ProximityMonitor monitor(&sensor);
  
  monitor.begin();
  
  // In loop:
  // monitor.update();
  // 
  // Serial.print("Object present: ");
  // Serial.println(monitor.isObjectPresent() ? "YES" : "NO");
  // 
  // Serial.print("Current baseline: ");
  // Serial.println(monitor.getBaseline());
  // 
  // Serial.print("Current threshold: ");
  // Serial.println(monitor.getThreshold());
  // 
  // Serial.print("Last reading: ");
  // Serial.println(monitor.getLastReading());
}

// Example 3: Multiple sensors
void example_multiple_sensors() {
  APDS9930Sensor sensor1;
  // VCNL4010Sensor sensor2;
  
  ProximityMonitor monitor1(&sensor1, 2);  // Pin 2
  // ProximityMonitor monitor2(&sensor2, 3);  // Pin 3
  
  monitor1.begin();
  // monitor2.begin();
  
  // In loop:
  // monitor1.update();
  // monitor2.update();
}

// Example 4: Custom tuning for different environments
void example_custom_tuning() {
  APDS9930Sensor sensor;
  ProximityMonitor monitor(&sensor, 2);
  
  // More sensitive - lower sigma, faster baseline adaptation
  monitor.setDetectionSigma(2.0);
  monitor.setEwmaAlpha(0.02);
  
  // Less sensitive - higher sigma, slower baseline adaptation
  // monitor.setDetectionSigma(4.0);
  // monitor.setEwmaAlpha(0.005);
  
  // Require longer sustained detection (reduce false positives)
  monitor.setMinDetectionDuration(500);  // 500ms
  
  monitor.begin();
}
