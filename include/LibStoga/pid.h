/** @file pid.h
 * @brief PID control system implementation for LibStoga robotics framework
 *
 * This file provides comprehensive PID (Proportional-Integral-Derivative) control
 * system implementation essential for precise robot control. The PID system
 * includes both abstract and concrete implementations with advanced features
 * like integral windup protection, sign flip reset, and configurable gains.
 *
 * Key Features:
 * - Abstract base class for extensible PID implementations
 * - Standard PID controller with configurable parameters
 * - Integral windup protection to prevent overshoot
 * - Sign flip reset for handling direction changes
 * - Time-based derivative calculation for smooth control
 * - Gain management through structured configuration
 *
 * PID Theory:
 * - **Proportional (P)**: Responds to current error
 * - **Integral (I)**: Corrects for accumulated error over time
 * - **Derivative (D)**: Predicts future error based on rate of change
 *
 * @author 6121D (The Pioneers)
 * @version 3.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup control
 */

#ifndef PID_H
#define PID_H

#include "error.h"
#include "gains.h"

namespace ls {

/**
 * @brief Abstract base class for PID controllers
 *
 * Defines the interface for all PID controller implementations in the framework.
 * This abstraction allows for different PID variants (standard, adaptive, fuzzy, etc.)
 * while maintaining a consistent API for users.
 *
 * All concrete PID implementations must inherit from this class and provide
 * implementations for the pure virtual methods.
 */
class AbstractPID {
public:
    /**
     * @brief Calculate PID output for given error
     *
     * This is the core method that computes the control output based on the
     * current error from the target. The implementation should handle all
     * PID calculations including proportional, integral, and derivative terms.
     *
     * @param error Current error value (target - current)
     * @return Control output value for actuators
     */
    virtual double update(const double error) = 0;

    /**
     * @brief Reset PID controller state
     *
     * Resets all internal state variables (integral accumulator, previous error,
     * timing information) to initial values. This should be called when starting
     * a new control sequence or when the system state changes significantly.
     */
    virtual void reset() = 0;

    /**
     * @brief Set new PID gain parameters
     *
     * Updates the PID controller with new gain values. This allows dynamic
     * tuning of the controller during operation or loading of preset configurations.
     *
     * @param gains New PID gain parameters
     */
    virtual void setGains(const PIDGains& gains) = 0;

    /**
     * @brief Get current PID gain parameters
     *
     * Returns the currently active gain parameters. Useful for debugging,
     * logging, or saving controller configurations.
     *
     * @return Current PID gain parameters
     */
    virtual PIDGains getGains() const = 0;

    /**
     * @brief Virtual destructor for proper cleanup
     */
    virtual ~AbstractPID() = default;
};

/**
 * @brief Standard PID controller implementation
 *
 * Implements a complete PID controller with advanced features for robotics applications.
 * This controller includes integral windup protection, sign flip reset handling,
 * and time-based derivative calculations for robust performance.
 *
 * Features:
 * - Configurable P, I, D gains
 * - Integral windup protection with configurable range
 * - Automatic reset on error sign changes (prevents windup during direction reversals)
 * - Time-based derivative calculation for consistent performance
 * - Integral limiting to prevent excessive accumulation
 *
 * Usage Example:
 * @code
 * // Create PID controller for velocity control
 * PID velocityController(1.2f, 0.05f, 0.8f, 50.0f, true);
 *
 * // Or create from gains struct
 * PIDGains gains = {1.2f, 0.05f, 0.8f, 50.0f, true};
 * PID controller(gains);
 *
 * // Use in control loop
 * double error = targetVelocity - currentVelocity;
 * double output = controller.update(error);
 * motor.move(output);
 * @endcode
 */
class PID : public AbstractPID {
public:
    /**
     * @brief Default constructor (deprecated)
     *
     * This constructor exists only to satisfy C++ requirements but should not
     * be used for functional code. It will throw an error if called.
     *
     * @deprecated Use parameterized constructors instead
     */
    explicit PID() { THROW_ERROR("PID Constructor requires parameters"); }

    /**
     * @brief Construct PID controller with individual parameters
     *
     * Creates a PID controller with explicitly specified gain parameters.
     * This constructor provides fine-grained control over PID behavior.
     *
     * @param kP Proportional gain (responsiveness to current error)
     * @param kI Integral gain (correction for accumulated error)
     * @param kD Derivative gain (prediction of future error)
     * @param windupRange Maximum integral accumulation range (prevents windup)
     * @param signFlipReset Whether to reset integral on error sign changes
     * @param integralLimit Maximum absolute value for integral term (default: 1e9)
     */
    explicit PID(float kP, float kI, float kD, float windupRange, bool signFlipReset, float integralLimit = 1e9f);

    /**
     * @brief Construct PID controller from gains structure
     *
     * Creates a PID controller using a PIDGains structure for cleaner code
     * and easier configuration management.
     *
     * @param gains PIDGains structure containing all controller parameters
     */
    explicit PID(const PIDGains& gains);

    /**
     * @brief Calculate PID control output
     *
     * Computes the control output based on the current error. This method
     * implements the complete PID algorithm with timing considerations.
     *
     * @param error Current error value (target - current)
     * @return Control output value (typically -1 to 1 or voltage values)
     */
    double update(const double error) override;

    /**
     * @brief Reset PID controller state
     *
     * Resets all internal state variables to initial values:
     * - Integral accumulator = 0
     * - Previous error = 0
     * - Last time = current time
     */
    void reset() override;

    /**
     * @brief Update PID gain parameters
     *
     * Dynamically changes the PID gains during operation. Useful for
     * adaptive control or switching between different control modes.
     *
     * @param gains New PID gain parameters
     */
    void setGains(const PIDGains& gains) override;

    /**
     * @brief Get current PID gain parameters
     *
     * Returns a copy of the current gain configuration for inspection
     * or serialization.
     *
     * @return Current PIDGains structure
     */
    PIDGains getGains() const override;

protected:
    float kP;                    /**< Proportional gain coefficient */
    float kI;                    /**< Integral gain coefficient */
    float kD;                    /**< Derivative gain coefficient */
    float windupRange;           /**< Integral windup protection range */
    bool signFlipReset;          /**< Whether to reset integral on sign changes */
    float integral;              /**< Accumulated integral value */
    float prevError;             /**< Previous error value for derivative calculation */
    float integralLimit;         /**< Maximum absolute integral value */
    unsigned long lastTimeMs;    /**< Timestamp of last update (milliseconds) */
};

} // namespace ls

#endif // PID_H
