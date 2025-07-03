#pragma once

#include <Arduino.h>

class Timer {
public:
    Timer();
    void start(unsigned long duration_ms);
    bool isFinished();
    void reset();

    static bool wait(unsigned long duration);  // Static wait function

private:
    unsigned long _startTime;
    unsigned long _duration;
    bool _running;
};

