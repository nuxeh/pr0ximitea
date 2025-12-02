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
    DummySensor(int controlPin, bool inverted = false)
        : controlPin(controlPin)
        , inverted(inverted)
        , baseValue(500)
        , detectedOffset(500)
        , lastValue(0)
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

	lastValue = outData;
        
        return true; 
    };

    uint16_t getRaw() const { return lastValue; }

    const char* getSensorName() const override {
        return "GPIO Dummy Sensor";
    }

private:
    int controlPin;
    bool inverted;
    
    // Internal state for simulation
    uint16_t baseValue;
    uint16_t detectedOffset; // Value added when GPIO is asserted
    uint16_t lastValue;
};

#warning Using dummy proximity sensor controlled by GPIO

#endif // DUMMY_PROXIMITY_SENSOR_H
