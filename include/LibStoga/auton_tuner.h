/** @file auton_tuner.h
 * @brief Autonomous tuning system with adaptive PID controllers
 *
 * This file provides an advanced autonomous tuning system that includes
 * automatic PID tuning and real-time adaptive PID control. The system
 * can automatically tune PID gains using a star pattern test and continuously
 * adapt gains during operation based on battery voltage, surface conditions,
 * and performance metrics.
 *
 * Key Features:
 * - Automatic PID tuning using star pattern movement tests
 * - Real-time adaptive PID with battery voltage compensation
 * - Surface condition detection and compensation
 * - Performance tracking and continuous learning
 * - Separate lateral and angular PID adaptation
 *
 * Tuning Process:
 * 1. **Star Pattern Test**: Robot moves in star pattern to test different scenarios
 * 2. **Performance Analysis**: System analyzes settling time and final error
 * 3. **Gain Optimization**: Automatically adjusts PID gains for optimal performance
 * 4. **Adaptive Learning**: Continuously adapts gains during operation
 *
 * Adaptive Features:
 * - **Battery Compensation**: Adjusts gains based on battery voltage levels
 * - **Surface Detection**: Detects smooth vs rough surfaces and adapts accordingly
 * - **Performance Tracking**: Monitors movement accuracy and speed
 * - **Real-time Adaptation**: Updates gains every 5 seconds during operation
 *
 * Usage Example:
 * @code
 * // Auto-tune PID gains
 * autoTune(chassis);
 *
 * // During autonomous operation
 * while (autonomous_running) {
 *     updateAdaptivePID();  // Update adaptive controllers
 *
 *     // Record movement performance after each movement
 *     recordMovementPerformance(lateral_error, lateral_settling_time,
 *                              angular_error, angular_settling_time);
 *
 *     pros::delay(50);
 * }
 *
 * // Get current adapted gains
 * PIDGains lateral = getCurrentLateralGains();
 * PIDGains angular = getCurrentAngularGains();
 * @endcode
 *
 * @author 6121D (The Pioneers)
 * @version 2.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup control
 */

#ifndef LS_AUTON_TUNER_H
#define LS_AUTON_TUNER_H

#include <memory>
#include <vector>
#include "gains.h"
#include "pid.h"

namespace ls {
    class Chassis; // Forward declaration

    /**
     * @brief Adaptive PID controller with real-time gain adjustment
     *
     * The AdaptivePIDController continuously monitors robot performance and
     * environmental conditions to automatically adjust PID gains for optimal
     * control. It compensates for battery voltage changes, surface conditions,
     * and learns from movement performance to improve accuracy over time.
     *
     * Adaptation Mechanisms:
     * - **Battery Compensation**: Increases gains when battery voltage drops
     * - **Surface Detection**: Adjusts for different surface types (smooth/rough)
     * - **Performance Learning**: Analyzes settling time and final error
     * - **Conservative Adaptation**: Gradual changes to prevent instability
     *
     * The controller maintains separate compensation factors for each PID term
     * and applies them multiplicatively to the base tuned gains.
     */
    class AdaptivePIDController {
    private:
        std::shared_ptr<AbstractPID> pid;        /**< Underlying PID controller */
        PIDGains base_gains;                     /**< Original manually tuned gains */
        PIDGains current_gains;                  /**< Currently adapted gains */
        PIDGains battery_compensation;           /**< Battery voltage compensation factors */
        PIDGains surface_compensation;           /**< Surface condition compensation factors */

        // Adaptation parameters
        const float ADAPTATION_RATE = 0.02f;     /**< How aggressively to adapt (0.01 = conservative, 0.1 = aggressive) */
        const float BATTERY_THRESHOLD_LOW = 11.0f;  /**< Voltage below which to start compensating */
        const float BATTERY_THRESHOLD_HIGH = 12.8f; /**< Full battery voltage */
        const int PERFORMANCE_WINDOW = 10;       /**< Number of movements to average for performance */
        const float MAX_GAIN_MULTIPLIER = 2.0f;  /**< Maximum gain increase allowed */
        const float MIN_GAIN_MULTIPLIER = 0.3f;  /**< Minimum gain decrease allowed */

        // Performance tracking
        std::vector<double> recent_errors;       /**< Recent movement final errors */
        std::vector<double> recent_settling_times; /**< Recent movement settling times */
        double last_battery_voltage = 12.8f;     /**< Last recorded battery voltage */
        uint32_t last_adaptation_time = 0;       /**< Last time gains were adapted */
        const uint32_t ADAPTATION_INTERVAL = 5000; /**< Adapt every 5 seconds */

        // Surface detection
        enum SurfaceType { UNKNOWN, SMOOTH, ROUGH, CARPET }; /**< Detected surface types */
        SurfaceType current_surface = UNKNOWN;    /**< Currently detected surface */
        uint32_t surface_detection_time = 0;      /**< Last surface detection time */

    public:
        /**
         * @brief Construct adaptive PID controller
         *
         * @param pid_controller Underlying PID controller to adapt
         * @param base Base tuned gains to adapt from
         */
        AdaptivePIDController(std::shared_ptr<AbstractPID> pid_controller, PIDGains base)
            : pid(pid_controller), base_gains(base), current_gains(base) {

            // Initialize compensation factors
            battery_compensation = {1.0f, 1.0f, 1.0f};  // Start with no compensation
            surface_compensation = {1.0f, 1.0f, 1.0f};

            // Note: Logging removed from constructor to avoid including iostream in header
        }

        /**
         * @brief Update adaptive controller
         *
         * Performs periodic updates to adapt gains based on current conditions
         * and recent performance data. Should be called regularly during operation.
         */
        void update();

        /**
         * @brief Record movement performance for learning
         *
         * Records the performance of a completed movement to improve future
         * gain adaptations. This data helps the system learn optimal gain
         * adjustments for different scenarios.
         *
         * @param final_error Final position/angle error at movement end
         * @param settling_time Time taken to settle within acceptable range
         */
        void recordMovement(double final_error, double settling_time);

        /**
         * @brief Get current adapted gains
         *
         * @return Currently adapted PID gains
         */
        PIDGains getCurrentGains() const { return current_gains; }

    private:
        /**
         * @brief Update battery voltage compensation
         *
         * Adjusts gain compensation based on current battery voltage.
         * Lower voltage requires higher gains to maintain performance.
         *
         * @param current_voltage Current battery voltage
         */
        void updateBatteryCompensation(double current_voltage);

        /**
         * @brief Detect current surface conditions
         *
         * Analyzes recent movement data to determine surface type
         * (smooth, rough, carpet) for appropriate gain compensation.
         */
        void detectSurfaceConditions();

        /**
         * @brief Update surface condition compensation
         *
         * Adjusts gains based on detected surface conditions.
         * Rough surfaces typically require different gain settings.
         */
        void updateSurfaceCompensation();

        /**
         * @brief Apply adapted gains to PID controller
         *
         * Calculates final gains by combining base gains with all
         * compensation factors and applies them to the PID controller.
         */
        void applyAdaptiveGains();
    };

    /**
     * @brief Automatically tune PID controllers using star pattern test
     *
     * Performs automatic PID tuning by commanding the robot to move in a
     * star pattern (multiple directions and distances). The system analyzes
     * the performance of each movement and optimizes PID gains accordingly.
     *
     * Tuning Process:
     * 1. Moves robot in 8 different directions (45-degree increments)
     * 2. Tests various distances to assess different speed ranges
     * 3. Analyzes overshoot, settling time, and steady-state error
     * 4. Iteratively adjusts gains using optimization algorithms
     * 5. Validates final gains with additional test movements
     *
     * @param chassis Shared pointer to chassis object to tune
     *
     * @note This is a blocking operation that may take several minutes
     * @warning Robot should be in a clear area with space to move freely
     */
    void autoTune(std::shared_ptr<Chassis> chassis);

    /**
     * @brief Update all adaptive PID controllers
     *
     * Updates both lateral and angular adaptive PID controllers with
     * current conditions and performance data. Should be called periodically
     * during autonomous or driver control operation (every 50-100ms).
     *
     * This function:
     * - Updates battery voltage compensation
     * - Performs surface condition detection
     * - Applies adaptive gain adjustments
     * - Maintains performance tracking windows
     */
    void updateAdaptivePID();

    /**
     * @brief Record movement performance data for adaptive learning
     *
     * Records performance metrics from completed movements to enable
     * continuous learning and adaptation. This data helps improve
     * PID performance over time and across different conditions.
     *
     * @param lateral_error Final position error for lateral movement (inches)
     * @param lateral_settling_time Time taken to settle laterally (milliseconds)
     * @param angular_error Final angle error for angular movement (degrees)
     * @param angular_settling_time Time taken to settle angularly (milliseconds)
     */
    void recordMovementPerformance(double lateral_error, double lateral_settling_time,
                                  double angular_error, double angular_settling_time);

    /**
     * @brief Get current adapted lateral PID gains
     *
     * Returns the current PID gains for lateral (forward/backward) movement
     * after applying all adaptive compensations.
     *
     * @return Current lateral PID gains with adaptations applied
     */
    PIDGains getCurrentLateralGains();

    /**
     * @brief Get current adapted angular PID gains
     *
     * Returns the current PID gains for angular (turning) movement
     * after applying all adaptive compensations.
     *
     * @return Current angular PID gains with adaptations applied
     */
    PIDGains getCurrentAngularGains();

} // namespace ls

#endif // LS_AUTON_TUNER_H