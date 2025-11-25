#ifndef DUMMY_PROXIMITY_SENSOR_H
#define DUMMY_PROXIMITY_SENSOR_H

#include "proximitySensor.h"
#include <Arduino.h>

/**
 * A mock sensor for testing without sensor hardware, using a GPIO pin.
 */
class DummySensor : public ProximitySensor {
public:
    /**
     * Constructor for the GPIO-controlled dummy sensor.
     * @param controlPin The GPIO pin used to assert the dummy detection.
     * @param inverted If true, the pin must be LOW to assert detection.
     */
    DummyProximitySensor(int controlPin, bool inverted = false)
        : controlPin(controlPin)
        , inverted(inverted)
    {}

    bool init() override {
        pinMode(controlPin, INPUT_PULLUP);
        return true;
    }

    bool readProximity(uint16_t& outData) override {
        int pinState = digitalRead(controlPin);

        bool isAsserted;
        if (inverted) {
            isAsserted = (pinState == LOW);
        } else {
            isAsserted = (pinState == HIGH);
        }

        int noise = random(-5, 6); 
        outData = baseValue + noise;

        if (isAsserted) {
            outData += detectedOffset;
        }
        
        return true; 
    };

    const char* getSensorName() override {
        return "GPIO Dummy Sensor";
    }

private:
    int controlPin;
    bool inverted;
    
    // Internal state for simulation
    uint16_t baseValue = 500;
    uint16_t detectedOffset = 500; // Value added when GPIO is asserted
};

#endif // DUMMY_PROXIMITY_SENSOR_H
