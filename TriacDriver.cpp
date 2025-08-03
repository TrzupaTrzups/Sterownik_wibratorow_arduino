#include "TriacDriver.h"

TriacDriver::TriacDriver(int pin) : _pin(pin) {}

TriacDriver* TriacDriver::_instance = nullptr;
uint8_t TriacDriver::_secondPower = 0;
uint16_t TriacDriver::_secondPulse = 0;

void TriacDriver::begin() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    _instance = this;
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

void TriacDriver::triggerWithSecond(uint8_t power, uint16_t pulseUs, uint16_t delayMs) {
    trigger(power, pulseUs);
    if (delayMs == 0) return;
    _secondPower = power;
    _secondPulse = pulseUs;

    // Configure Timer1 for CTC mode with prescaler 8
    TCNT1 = 0;
    OCR1A = (uint32_t)delayMs * 2000; // 2MHz ticks -> 2000 per ms
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    TIMSK1 |= (1 << OCIE1A);
}

void TriacDriver::handleTimerInterrupt() {
    if (_instance) {
        _instance->trigger(_secondPower, _secondPulse);
    }
    TIMSK1 &= ~(1 << OCIE1A);
    TCCR1B = 0;
}

ISR(TIMER1_COMPA_vect) {
    TriacDriver::handleTimerInterrupt();
}
