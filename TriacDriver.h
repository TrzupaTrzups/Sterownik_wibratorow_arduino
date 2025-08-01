#ifndef TRIAC_DRIVER_H
#define TRIAC_DRIVER_H

#include <Arduino.h>

class TriacDriver {
public:
    explicit TriacDriver(int pin);
    void begin();
    void trigger(uint8_t power, uint16_t pulseUs);

private:
    int _pin;
};

#endif
