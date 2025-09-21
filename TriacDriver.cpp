#include "TriacDriver.h"
#include <avr/interrupt.h>

TriacDriver::TriacDriver(int pin) : _pin(pin), _port(nullptr), _bitMask(0) {}

TriacDriver* TriacDriver::_instance = nullptr;
volatile uint16_t TriacDriver::_intervals[4] = {0};
volatile uint8_t TriacDriver::_eventCount = 0;
volatile uint8_t TriacDriver::_currentEvent = 0;

void TriacDriver::begin() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    _port = portOutputRegister(digitalPinToPort(_pin));
    _bitMask = digitalPinToBitMask(_pin);
    _instance = this;
}

uint16_t TriacDriver::microsecondsToTicks(uint32_t microseconds) {
    uint32_t ticks = microseconds * 2; // 0.5us per tick with prescaler 8
    if (ticks == 0 && microseconds > 0) {
        ticks = 1;
    }
    if (ticks > 0xFFFF) {
        ticks = 0xFFFF;
    }
    return static_cast<uint16_t>(ticks);
}

void TriacDriver::stopTimer() {
    TIMSK1 &= ~(1 << OCIE1A);
    TCCR1B = 0;
    _eventCount = 0;
    _currentEvent = 0;
}

void TriacDriver::applyEvent(uint8_t eventIndex) {
    if (!_port) {
        return;
    }

    if ((eventIndex & 0x01) == 0) {
        *_port |= _bitMask; // gate high on even events
    } else {
        *_port &= ~_bitMask; // gate low on odd events
    }
}

void TriacDriver::scheduleIntervals(const uint16_t* intervals, uint8_t count) {
    if (!_port || count == 0) {
        return;
    }

    const uint8_t maxEvents = sizeof(_intervals) / sizeof(_intervals[0]);
    if (count > maxEvents) {
        count = maxEvents;
    }

    uint8_t oldSREG = SREG;
    cli();

    stopTimer();

    for (uint8_t i = 0; i < count; ++i) {
        _intervals[i] = intervals[i];
    }
    _eventCount = count;
    _currentEvent = 0;

    // Ensure the gate starts low
    *_port &= ~_bitMask;

    // Execute any immediate events without arming the timer
    while (_currentEvent < _eventCount && _intervals[_currentEvent] == 0) {
        applyEvent(_currentEvent);
        ++_currentEvent;
    }

    if (_currentEvent < _eventCount) {
        TCCR1A = 0;
        TCNT1 = 0;
        OCR1A = _intervals[_currentEvent];
        TCCR1B = (1 << WGM12) | (1 << CS11);
        TIMSK1 |= (1 << OCIE1A);
    }

    SREG = oldSREG;
}

void TriacDriver::fireDelayed(uint16_t delayUs, uint16_t pulseUs) {
    if (pulseUs == 0) {
        uint8_t oldSREG = SREG;
        cli();
        stopTimer();
        if (_port) {
            *_port &= ~_bitMask;
        } else {
            digitalWrite(_pin, LOW);
        }
        SREG = oldSREG;
        return;
    }

    uint16_t intervals[2];
    intervals[0] = microsecondsToTicks(delayUs);
    intervals[1] = microsecondsToTicks(pulseUs);
    scheduleIntervals(intervals, 2);
}

void TriacDriver::trigger(uint8_t power, uint16_t pulseUs) {
    if (power > 0 && pulseUs > 0) {
        uint16_t delayUs = map(100 - power, 0, 100, 0, 8822);
        fireDelayed(delayUs, pulseUs);
    } else {
        uint8_t oldSREG = SREG;
        cli();
        stopTimer();
        if (_port) {
            *_port &= ~_bitMask;
        } else {
            digitalWrite(_pin, LOW);
        }
        SREG = oldSREG;
    }
}

void TriacDriver::triggerWithSecond(uint8_t power, uint16_t pulseUs, uint16_t delayMs) {
    if (power > 0 && pulseUs > 0) {
        uint16_t delayUs = map(100 - power, 0, 100, 0, 8822);
        if (delayMs > 0) {
            uint16_t intervals[4];
            intervals[0] = microsecondsToTicks(delayUs);
            uint16_t pulseTicks = microsecondsToTicks(pulseUs);
            intervals[1] = pulseTicks;
            uint32_t spacingTicks32 = static_cast<uint32_t>(delayMs) * 2000;
            if (spacingTicks32 > 0xFFFF) {
                spacingTicks32 = 0xFFFF;
            }
            uint16_t spacingTicks = static_cast<uint16_t>(spacingTicks32);
            uint16_t betweenPulses = (spacingTicks > pulseTicks) ? (spacingTicks - pulseTicks) : 0;
            intervals[2] = betweenPulses;
            intervals[3] = pulseTicks;
            scheduleIntervals(intervals, 4);
        } else {
            fireDelayed(delayUs, pulseUs);
        }
    } else {
        uint8_t oldSREG = SREG;
        cli();
        stopTimer();
        if (_port) {
            *_port &= ~_bitMask;
        } else {
            digitalWrite(_pin, LOW);
        }
        SREG = oldSREG;
    }
}

void TriacDriver::handleTimer1Compare() {
    TriacDriver* driver = _instance;
    if (!driver) {
        TIMSK1 &= ~(1 << OCIE1A);
        TCCR1B = 0;
        _eventCount = 0;
        _currentEvent = 0;
        return;
    }

    uint8_t index = _currentEvent;
    if (index >= _eventCount) {
        driver->stopTimer();
        return;
    }

    driver->applyEvent(index);
    ++index;
    _currentEvent = index;

    while (index < _eventCount && _intervals[index] == 0) {
        driver->applyEvent(index);
        ++index;
        _currentEvent = index;
    }

    if (index >= _eventCount) {
        driver->stopTimer();
        return;
    }

    OCR1A = _intervals[index];
}

ISR(TIMER1_COMPA_vect) {
    TriacDriver::handleTimer1Compare();
}
