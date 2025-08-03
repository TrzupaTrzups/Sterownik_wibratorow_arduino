#include "TriacDriver.h"

TriacDriver::TriacDriver(int pin) : _pin(pin) {}

TriacDriver* TriacDriver::_instance = nullptr;
uint16_t TriacDriver::_secondDelay = 0;
uint16_t TriacDriver::_secondPulse = 0;

void TriacDriver::begin() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    _instance = this;
}

void TriacDriver::fireDelayed(uint16_t delayUs, uint16_t pulseUs) {
    delayMicroseconds(delayUs);
    digitalWrite(_pin, HIGH);
    delayMicroseconds(pulseUs);
    digitalWrite(_pin, LOW);
}

void TriacDriver::trigger(uint8_t power, uint16_t pulseUs) {
    if (power > 0) {
        uint16_t delayUs = map(100 - power, 0, 100, 0, 8822);
        fireDelayed(delayUs, pulseUs);
    } else {
        digitalWrite(_pin, LOW);
    }
}

void TriacDriver::triggerWithSecond(uint8_t power, uint16_t pulseUs, uint16_t delayMs) {
    if (power > 0) {
        uint16_t delayUs = map(100 - power, 0, 100, 0, 8822);
        if (delayMs > 0) {
            _secondDelay = delayUs;
            _secondPulse = pulseUs;

            // Start Timer1 immediately so that, after reapplying the same
            // phase delay, the second pulse fires exactly delayMs
            // milliseconds after the first one.
            TCNT1 = 0;
            OCR1A = (uint32_t)delayMs * 2000; // 0.5us per tick
            TCCR1A = 0;
            TCCR1B = (1 << WGM12) | (1 << CS11);
            TIMSK1 |= (1 << OCIE1A);
        }
        fireDelayed(delayUs, pulseUs);
    } else {
        digitalWrite(_pin, LOW);
    }
}

void TriacDriver::handleTimer1Compare() {
    if (_instance) {
        _instance->fireDelayed(_secondDelay, _secondPulse);
    }
    _secondDelay = 0;
}

ISR(TIMER1_COMPA_vect) {
    TriacDriver::handleTimer1Compare();
    TIMSK1 &= ~(1 << OCIE1A);
    TCCR1B = 0;
}
