#ifndef TRIAC_DRIVER_H
#define TRIAC_DRIVER_H

#include <Arduino.h>

class TriacDriver {
public:
    explicit TriacDriver(int pin);
    void begin();
    void trigger(uint8_t power, uint16_t pulseUs);
    // Trigger a pulse and schedule an identical one delayMs milliseconds later
    void triggerWithSecond(uint8_t power, uint16_t pulseUs, uint16_t delayMs = 10);
    static void handleTimer1Compare();

private:
    int _pin;
    static TriacDriver* _instance;
    static uint16_t _secondDelay;
    static uint16_t _secondPulse;

    void fireDelayed(uint16_t delayUs, uint16_t pulseUs);
};

#endif
