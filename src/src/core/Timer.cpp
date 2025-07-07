#include "Timer.h"

/**
 * @brief Construct a new Timer::Timer object
 */
Timer::Timer() {
    _startTime = 0;
    _duration = 0;
    _running = false;
}

/**
 * @brief Starts the timer with a specific duration.
 * @param duration_ms The duration of the timer in milliseconds.
 */
void Timer::start(unsigned long duration_ms) {
    _startTime = millis();
    _duration = duration_ms;
    _running = true;
}

/**
 * @brief Checks if the timer has finished.
 * @return true if the timer has completed its duration, false otherwise.
 * Once finished, the timer stops running until started again.
 */
bool Timer::isFinished() {
    if (!_running) return false;
    if (millis() - _startTime >= _duration) {
        _running = false;
        return true;
    }
    return false;
}

/**
 * @brief Resets the timer to its initial state.
 */
void Timer::reset() {
    _running = false;
    _startTime = 0;
    _duration = 0;
}

void Timer::wait(unsigned long duration) {
    static Timer waitTimer;
    waitTimer.start(duration);
        while (!timer.isFinished())
            ;
}
