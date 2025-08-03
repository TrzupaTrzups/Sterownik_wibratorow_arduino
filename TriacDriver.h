#ifndef TRIAC_DRIVER_H
#define TRIAC_DRIVER_H

#include <Arduino.h>

class TriacDriver {
public:
    explicit TriacDriver(int pin);
    void begin();
    void trigger(uint8_t power, uint16_t pulseUs);
    // Trigger pulse and automatically fire a second one after delayMs
    void triggerWithSecond(uint8_t power, uint16_t pulseUs, uint16_t delayMs = 10);
    static void handleTimer1Compare();

private:
    int _pin;
    static TriacDriver* _instance;
    static uint8_t _secondPower;
    static uint16_t _secondPulse;
};

#endif
