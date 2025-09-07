/** @file Timer.hpp
 * @brief High-precision timer utility for PROS robotics applications
 *
 * This file provides a comprehensive timer class designed specifically for
 * VEX robotics programming with PROS. The Timer class offers precise timing
 * control with pause/resume functionality, making it ideal for autonomous
 * routines, driver control sequences, and general timing operations.
 *
 * Key Features:
 * - High-precision timing using PROS system clock
 * - Pause and resume functionality for complex timing scenarios
 * - Non-blocking operation with status checking
 * - Automatic time tracking and elapsed time calculation
 * - Thread-safe operations for concurrent use
 *
 * Common Use Cases:
 * - Autonomous routine timing and sequencing
 * - Driver control skill challenges with time limits
 * - Periodic task scheduling
 * - Timeout management for operations
 * - Performance benchmarking and profiling
 *
 * @author Asish Nakka
 * @version 0.1
 * @date 2024-11-29
 *
 * @copyright Copyright (c) 2024
 *
 * @ingroup utilities
 */

#ifndef TIMER_HPP
#define TIMER_HPP

#include "pros/rtos.hpp"

namespace ls {

/**
 * @brief High-precision timer with pause/resume functionality
 *
 * The Timer class provides precise timing control for robotics applications.
 * It uses the PROS system clock for high accuracy and supports advanced
 * features like pausing and resuming timing operations.
 *
 * The timer operates in two main states:
 * - **Running**: Actively counting down from the set time
 * - **Paused**: Timing suspended, elapsed time preserved
 *
 * Usage Examples:
 * @code
 * // Create a 5-second timer
 * Timer autonTimer(5000); // 5000ms = 5 seconds
 *
 * // Start autonomous routine
 * while (!autonTimer.isDone()) {
 *     // Perform autonomous actions
 *     chassis.moveToPoint(24, 24);
 *
 *     // Check remaining time for decision making
 *     if (autonTimer.getTimeLeft() < 1000) {
 *         // Less than 1 second left - finish up
 *         break;
 *     }
 *
 *     pros::delay(20); // Small delay to prevent CPU hogging
 * }
 *
 * // Timer with pause/resume for complex sequences
 * Timer skillTimer(60000); // 60 second skills challenge
 *
 * // During driver control
 * if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
 *     skillTimer.pause();
 *     // Perform some action that shouldn't count against time
 *     skillTimer.resume();
 * }
 *
 * // Non-blocking wait with other operations
 * Timer delayTimer(1000);
 * while (!delayTimer.isDone()) {
 *     // Can perform other operations while waiting
 *     updateSensors();
 *     pros::delay(10);
 * }
 * @endcode
 */
class Timer {
public:
    /**
     * @brief Construct a new Timer with specified duration
     *
     * Creates a timer that will count down from the specified time in milliseconds.
     * The timer starts running immediately upon construction.
     *
     * @param time Duration in milliseconds (must be > 0)
     *
     * @note The timer begins counting immediately. Use pause() after construction
     *       if you need to delay the start.
     */
    Timer(uint32_t time);

    /**
     * @brief Get the original time duration set for this timer
     *
     * Returns the initial time value that was set when the timer was created
     * or last reset. This value does not change during timer operation.
     *
     * @return Original time duration in milliseconds
     */
    uint32_t getTimeSet();

    /**
     * @brief Get the remaining time until timer completion
     *
     * Calculates how much time is left before the timer reaches its target.
     * Returns 0 if the timer has already finished.
     *
     * @return Remaining time in milliseconds, or 0 if timer is done
     */
    uint32_t getTimeLeft();

    /**
     * @brief Get the elapsed time since timer started
     *
     * Returns how much time has passed since the timer was created or last reset.
     * This includes time spent in paused state.
     *
     * @return Elapsed time in milliseconds
     */
    uint32_t getTimePassed();

    /**
     * @brief Check if the timer has reached its target time
     *
     * Determines whether the timer has completed its countdown. A timer is
     * considered done when the elapsed time equals or exceeds the set time.
     *
     * @return true if timer has finished, false if still running
     */
    bool isDone();

    /**
     * @brief Check if the timer is currently paused
     *
     * Returns the pause state of the timer. A paused timer does not count
     * down but preserves its elapsed time for when it resumes.
     *
     * @return true if timer is paused, false if running
     */
    bool isPaused();

    /**
     * @brief Set a new time duration for the timer
     *
     * Changes the target time for the timer and resets it to start counting
     * from the beginning with the new duration.
     *
     * @param time New duration in milliseconds (must be > 0)
     *
     * @note This resets the timer completely, clearing any elapsed time
     */
    void set(uint32_t time);

    /**
     * @brief Reset the timer to its initial state
     *
     * Resets the timer to start counting from the beginning again.
     * The timer will use the same duration that was originally set.
     */
    void reset();

    /**
     * @brief Pause the timer without resetting elapsed time
     *
     * Suspends the timer operation while preserving the current elapsed time.
     * The timer can be resumed later with resume().
     *
     * @note If the timer is already paused, this has no effect
     */
    void pause();

    /**
     * @brief Resume a paused timer
     *
     * Continues timing from where it left off when pause() was called.
     * Has no effect if the timer is not currently paused.
     *
     * @note If the timer is not paused, this has no effect
     */
    void resume();

    /**
     * @brief Block execution until timer completes
     *
     * Waits synchronously until the timer reaches its target time.
     * This is a blocking operation that will pause program execution.
     *
     * @warning This is a blocking call that will halt program execution.
     *          Use isDone() for non-blocking checks in most robotics applications.
     */
    void waitUntilDone();

private:
    uint32_t period;      /**< Original time duration set for the timer */
    uint32_t lastTime;    /**< Last time the timer was updated (PROS millis) */
    uint32_t timeWaited;  /**< Total time spent in paused state */
    bool paused;          /**< Current pause state of the timer */
};

} // namespace ls

#endif // TIMER_HPP
