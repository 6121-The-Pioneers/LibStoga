/** ✅
 * @file timer.hpp
 * @author Asish Nakka
 * @brief file that contains all code related to timing. Mostly for user convienience.
 * @version 0.1
 * @date 2024-11-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//// TODO ============================================================================================= Rishit Varshney: get comments
#ifndef TIMER_HPP
#define TIMER_HPP

#include "pros/rtos.hpp"

namespace ls {

    class Timer {
    public:
        Timer(uint32_t time);

        uint32_t getTimeSet();
        uint32_t getTimeLeft();
        uint32_t getTimePassed();
        bool isDone();
        bool isPaused();
        void set(uint32_t time);
        void reset();
        void pause();
        void resume();
        void waitUntilDone();

    private:
        uint32_t period;
        uint32_t lastTime;
        uint32_t timeWaited = 0;
        bool paused = false;
    };

} // namespace ls

#endif // TIMER_HPP
