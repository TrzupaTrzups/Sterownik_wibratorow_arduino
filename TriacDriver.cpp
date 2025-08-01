#include "TriacDriver.h"

TriacDriver::TriacDriver(int pin) : _pin(pin) {}

void TriacDriver::begin() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
}

void TriacDriver::trigger(uint8_t power, uint16_t pulseUs) {
    if (power > 0) {
        uint16_t delayValue = map(100 - power, 0, 100, 0, 8822);
        delayMicroseconds(delayValue);
        digitalWrite(_pin, HIGH);
        delayMicroseconds(pulseUs);
        digitalWrite(_pin, LOW);
    } else {
        digitalWrite(_pin, LOW);
    }
}
