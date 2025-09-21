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
    volatile uint8_t* _port;
    uint8_t _bitMask;
    static TriacDriver* _instance;
    static volatile uint16_t _intervals[4];
    static volatile uint8_t _eventCount;
    static volatile uint8_t _currentEvent;

    void fireDelayed(uint16_t delayUs, uint16_t pulseUs);
    void scheduleIntervals(const uint16_t* intervals, uint8_t count);
    static uint16_t microsecondsToTicks(uint32_t microseconds);
    void applyEvent(uint8_t eventIndex);
    void stopTimer();
};

#endif
