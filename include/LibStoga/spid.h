/** @file spid.h
 * @brief Smart PID (SPID) - Adaptive PID control system
 *
 * This file implements the Smart PID algorithm, an advanced extension of traditional
 * PID control that adapts its parameters in real-time based on system performance.
 * The SPID system learns from its own behavior and adjusts gains to optimize
 * control performance dynamically.
 *
 * Key Features:
 * - Adaptive gain tuning based on system response
 * - Learning algorithm that improves performance over time
 * - Sigmoid-based learning rate adjustment
 * - Configurable correction constants and windup protection
 * - Extension of standard PID with intelligent adaptation
 *
 * Smart PID Algorithm:
 * The SPID algorithm extends traditional PID by adding a learning mechanism that:
 * 1. Monitors system response and error patterns
 * 2. Calculates expected vs actual performance
 * 3. Adjusts PID gains using a learning rate and correction constant
 * 4. Uses sigmoid function to prevent over-correction near optimal values
 *
 * @warning This algorithm can produce rapid output changes. Use with caution and
 * proper parameter tuning. Always test in a safe environment before field use.
 *
 * @author 6121D (The Pioneers)
 * @version 3.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup control
 */

#ifndef LS_SPID_H
#define LS_SPID_H

#include "pid.h"
#include "error.h"
#include "gains.h"

namespace ls {

/**
 * @brief Configuration parameters for Smart PID controller
 *
 * Defines all parameters needed to configure a Smart PID controller.
 * These parameters control the learning behavior and performance characteristics.
 */
struct smart_pid_parameters_t {
    double correction_constant = 1.0;  /**< Speed of overall motion and adaptation rate */
    double windup = 1.0;               /**< Integral windup protection and dominance factor */
    double learning_constant = 0.001;  /**< Rate at which the system learns and adapts */
    double max_value = 127.0;          /**< Maximum absolute output value */
};

/**
 * @brief Smart PID controller with adaptive learning capabilities
 *
 * Implements an intelligent PID controller that adapts its gains in real-time
 * based on system performance. The controller learns from its own behavior
 * and continuously optimizes its parameters for better control.
 *
 * The Smart PID algorithm works by:
 * 1. Running traditional PID calculations
 * 2. Comparing actual vs expected performance
 * 3. Adjusting gains using learning algorithms
 * 4. Applying sigmoid smoothing to prevent oscillations
 *
 * @warning This controller can produce aggressive output changes. Always:
 * - Start with conservative parameters
 * - Test in controlled environments
 * - Monitor system response carefully
 * - Use appropriate maximum output limits
 *
 * Usage Example:
 * @code
 * // Create Smart PID with conservative settings
 * SmartPID smartController(0.5, 5.0, 0.0005, 100.0);
 *
 * // Or use parameter structure
 * smart_pid_parameters_t params = {0.5, 5.0, 0.0005, 100.0};
 * SmartPID controller(params);
 *
 * // Use in control loop
 * double error = targetPosition - currentPosition;
 * double output = controller.update(error);
 * motor.move(output);
 * @endcode
 */
class SmartPID : public ls::AbstractPID {
public:
    /**
     * @brief Construct Smart PID controller with individual parameters
     *
     * Creates a Smart PID controller with explicitly specified parameters.
     * Use conservative values initially and tune based on system response.
     *
     * @param cc Correction constant (0.1-2.0 recommended). Higher values = faster adaptation
     * @param w Windup value (1.0-20.0 recommended). Controls integral behavior
     * @param lc Learning constant (0.0001-0.01 recommended). Lower = more stable, slower learning
     * @param max Maximum output value (typically motor voltage limit)
     *
     * @warning High correction_constant values can cause instability
     */
    explicit SmartPID(double cc = 1.0, double w = 10.0, double lc = 0.001, double max = 127.0);

    /**
     * @brief Construct Smart PID controller from parameter structure
     *
     * Creates a Smart PID controller using a configuration structure.
     * This approach is recommended for cleaner code and easier parameter management.
     *
     * @param parameters Complete parameter set for Smart PID configuration
     */
    explicit SmartPID(smart_pid_parameters_t& parameters);

    /**
     * @brief Calculate Smart PID control output
     *
     * Computes the control output using the Smart PID algorithm. This method
     * performs traditional PID calculations plus adaptive learning to improve
     * performance over time.
     *
     * @param e Current error value (target - current)
     * @return Control output value (clamped to max_value)
     */
    double update(const double e) override;

    /**
     * @brief Reset Smart PID controller state
     *
     * Resets all internal state variables while preserving learned parameters.
     * This includes proportional, derivative, and previous values, but keeps
     * the adapted gains for continuous learning.
     */
    void reset() override;

    /**
     * @brief Set new PID gains (inherited from AbstractPID)
     *
     * @note Smart PID adapts gains automatically, so this method primarily
     * sets initial values that will be modified by the learning algorithm.
     *
     * @param gains New gain parameters
     */
    void setGains(const PIDGains& gains) override;

    /**
     * @brief Get current PID gains (inherited from AbstractPID)
     *
     * Returns the current adapted gain values. These may differ from initial
     * values due to the learning algorithm.
     *
     * @return Current adapted PID gains
     */
    PIDGains getGains() const override;

private:
    double kp;  /**< Current proportional gain (adapted) */
    double kd;  /**< Current derivative gain (adapted) */

    double P;           /**< Current proportional component */
    double D;           /**< Current derivative component */
    double prev_val;    /**< Previous error value for derivative calculation */

    double correction_constant;  /**< Adaptation speed multiplier */
    double windup;               /**< Integral windup protection factor */
    double learning_constant;    /**< Learning rate for adaptation */
    double max_value;            /**< Maximum output magnitude */

    /**
     * @brief Update traditional PID components
     *
     * Calculates the proportional and derivative components using
     * standard PID mathematics.
     *
     * @param e Current error value
     */
    void update_components(const double e);

    /**
     * @brief Update adaptive gains using learning algorithm
     *
     * Adjusts the PID gains based on system performance and learning
     * rate. This is the core of the Smart PID adaptation mechanism.
     *
     * @param e Current error value
     */
    void update_constants(const double e);

    /**
     * @brief Calculate expected system response
     *
     * Predicts what the system output should be based on current
     * error and adapted gains. Used for learning algorithm comparison.
     *
     * @param e Current error value
     * @return Expected output value
     */
    double get_expected(const double e) const;

    /**
     * @brief Sigmoid function for learning rate adjustment
     *
     * Applies a sigmoid transformation to reduce learning rate as
     * the system approaches optimal performance. This prevents
     * oscillations around the optimal point.
     *
     * @param x Input value to transform
     * @return Sigmoid-transformed value
     */
    double learn_sigmoid(double x) const;
};

} // namespace ls

#endif // LS_SPID_H