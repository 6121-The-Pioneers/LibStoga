/** @file driver_control.h
 * @brief Advanced driver control system with joystick curves and multiple drive modes
 *
 * This file provides a sophisticated driver control system designed to enhance
 * driver experience and performance in VEX robotics competitions. It features
 * multiple joystick curve types, various drive modes, and configurable parameters
 * to optimize control feel for different drivers and robot configurations.
 *
 * Key Features:
 * - Multiple joystick curve types (Linear, Exponential, Sigmoid, Cubic, Cheesy Drive)
 * - Support for Arcade, Tank, Cheesy Drive, Curvature, and GTA drive modes
 * - Configurable deadzones and gain settings
 * - Real-time curve application for smooth control
 * - PROS controller integration
 *
 * Drive Modes:
 * - **Arcade**: Single-stick control with throttle and turn
 * - **Tank**: Dual-stick control for independent left/right movement
 * - **Cheesy Drive**: Enhanced arcade with quick-turn boost
 * - **Curvature**: Radius-based turning for smooth curved paths
 * - **GTA**: Trigger-based acceleration/braking with analog steering
 *
 * Joystick Curves:
 * - **Linear**: Direct 1:1 input/output mapping
 * - **Exponential**: Fine control at low speeds, aggressive at high speeds
 * - **Sigmoid**: Smooth S-curve acceleration for natural feel
 * - **Cubic**: Very aggressive low-speed control
 * - **Cheesy Drive**: Optimized curve for Cheesy Drive mode
 *
 * @author 6121D (The Pioneers)
 * @version 2.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup control
 */

#pragma once
#include <cmath>
#include <algorithm>

namespace ls {

/**
 * @brief Joystick curve utilities for enhanced driver control
 *
 * The JoystickCurve class provides mathematical transformations to modify
 * controller input for better driver feel and precision. Different curve
 * types optimize control for various driving scenarios and driver preferences.
 *
 * Curve Types and Use Cases:
 * - **Linear**: Best for precise, predictable control (good for beginners)
 * - **Exponential**: Excellent for fine low-speed control with high-speed capability
 * - **Sigmoid**: Natural acceleration feel, smooth throughout range
 * - **Cubic**: Maximum precision at low speeds (advanced drivers)
 * - **Cheesy Drive**: Optimized for quick turns and smooth driving
 *
 * Usage Example:
 * @code
 * // Apply exponential curve to throttle input
 * double rawThrottle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
 * double curvedThrottle = JoystickCurve::applyCurve(rawThrottle,
 *                                                   JoystickCurve::EXPONENTIAL,
 *                                                   1.5); // Aggressive curve
 * @endcode
 */
class JoystickCurve {
public:
    /**
     * @brief Available joystick curve types
     */
    enum CurveType {
        LINEAR,           /**< No curve, direct 1:1 mapping */
        EXPONENTIAL,      /**< Exponential curve for fine low-speed control */
        SIGMOID,          /**< S-curve for smooth, natural acceleration */
        CUBIC,           /**< Cubic curve for maximum low-speed precision */
        CHEESY_DRIVE     /**< Special curve optimized for Cheesy Drive mode */
    };

    /**
     * @brief Apply a curve transformation to joystick input
     *
     * Transforms raw joystick input using the specified curve type and gain.
     * All curves include automatic deadzone handling to prevent jitter.
     *
     * @param input Raw joystick input (-127 to 127)
     * @param type Type of curve to apply
     * @param gain Curve intensity multiplier (0.0 = linear, higher = more aggressive)
     * @return Curved output value (-127 to 127)
     *
     * @note Input is automatically clamped and includes 5% deadzone by default
     */
    static double applyCurve(double input, CurveType type, double gain = 1.0) {
        // Normalize input to -1.0 to 1.0
        double normalized = input / 127.0;

        // Apply deadzone
        if (std::abs(normalized) < 0.05) {
            return 0.0;
        }

        double curved = 0.0;

        switch (type) {
            case LINEAR:
                curved = normalized;
                break;

            case EXPONENTIAL:
                curved = expoCurve(normalized, gain);
                break;

            case SIGMOID:
                curved = sigmoidCurve(normalized, gain);
                break;

            case CUBIC:
                curved = cubicCurve(normalized, gain);
                break;

            case CHEESY_DRIVE:
                curved = cheesyCurve(normalized, gain);
                break;
        }

        // Denormalize back to -127 to 127
        return curved * 127.0;
    }

    /**
     * @brief Apply exponential curve for fine low-speed control
     *
     * Creates a curve where small inputs produce even smaller outputs,
     * allowing for precise control at low speeds while maintaining
     * full power capability at high speeds.
     *
     * @param input Normalized input (-1.0 to 1.0)
     * @param gain Curve aggressiveness (higher = more pronounced curve)
     * @return Curved output (-1.0 to 1.0)
     */
    static double expoCurve(double input, double gain) {
        return std::copysign(std::pow(std::abs(input), 1.0 + gain), input);
    }

    /**
     * @brief Apply sigmoid curve for smooth S-curve acceleration
     *
     * Creates a smooth S-shaped curve that provides natural acceleration
     * feel, with gradual changes at low speeds and more responsive
     * changes at higher speeds.
     *
     * @param input Normalized input (-1.0 to 1.0)
     * @param gain Curve steepness (higher = steeper S-curve)
     * @return Curved output (-1.0 to 1.0)
     */
    static double sigmoidCurve(double input, double gain) {
        double scaled = input * gain * 3.0; // Scale for better curve shape
        return std::tanh(scaled);
    }

    /**
     * @brief Apply cubic curve for aggressive low-speed control
     *
     * Provides maximum precision at low speeds by cubing the input,
     * making small joystick movements produce very small outputs.
     * Best for advanced drivers who need fine control.
     *
     * @param input Normalized input (-1.0 to 1.0)
     * @param gain Additional linear component (higher = less aggressive)
     * @return Curved output (-1.0 to 1.0)
     */
    static double cubicCurve(double input, double gain) {
        double absInput = std::abs(input);
        double cubed = std::pow(absInput, 3.0);
        return std::copysign(cubed + (absInput * gain), input);
    }

    /**
     * @brief Cheesy Drive optimized curve
     *
     * Special curve designed specifically for Cheesy Drive mode,
     * blending cubic precision at low speeds with linear response
     * at high speeds for optimal driving feel.
     *
     * @param input Normalized input (-1.0 to 1.0)
     * @param gain Curve blend factor (higher = more linear at high speeds)
     * @return Curved output (-1.0 to 1.0)
     */
    static double cheesyCurve(double input, double gain) {
        double absInput = std::abs(input);
        // Mix of cubic for low speeds and linear for high speeds
        double lowSpeed = std::pow(absInput, 2.5) * gain;
        double highSpeed = absInput * (2.0 - gain);
        double blended = absInput < 0.5 ? lowSpeed : highSpeed;
        return std::copysign(blended, input);
    }
};

/**
 * @brief Enhanced driver control system with multiple drive modes
 *
 * The DriverControl class provides a complete driver control solution
 * with support for multiple drive modes, configurable joystick curves,
 * and real-time parameter adjustment. It integrates seamlessly with
 * PROS controllers and provides optimized motor power calculations.
 *
 * Drive Mode Details:
 * - **Arcade**: Traditional single-stick driving (throttle + turn)
 * - **Tank**: Independent left/right stick control
 * - **Cheesy Drive**: Enhanced arcade with quick-turn capability
 * - **Curvature**: Radius-based turning for smooth curved paths
 * - **GTA**: Trigger-based acceleration with analog steering
 *
 * Configuration Example:
 * @code
 * DriverControl::DriveConfig config;
 * config.type = DriverControl::CHEESY;
 * config.throttleCurve = JoystickCurve::EXPONENTIAL;
 * config.turnCurve = JoystickCurve::SIGMOID;
 * config.throttleGain = 1.2;
 * config.turnGain = 0.8;
 *
 * DriverControl driver(config);
 *
 * // In driver control loop
 * double leftPower, rightPower;
 * driver.calculateDrivePowers(controller, leftPower, rightPower);
 * leftMotors.move(leftPower);
 * rightMotors.move(rightPower);
 * @endcode
 */
class DriverControl {
public:
    /**
     * @brief Available drive control modes
     */
    enum DriveType {
        ARCADE,      /**< Single stick: throttle + turn */
        TANK,        /**< Dual stick: independent left/right control */
        CHEESY,      /**< Cheesy Drive: throttle + quick turn boost */
        CURVATURE,   /**< Curvature drive: throttle + radius control */
        GTA         /**< GTA-style: trigger acceleration + analog steering */
    };

    /**
     * @brief Configuration structure for driver control settings
     */
    struct DriveConfig {
        DriveType type = ARCADE;                    /**< Drive mode selection */
        JoystickCurve::CurveType throttleCurve = JoystickCurve::EXPONENTIAL; /**< Throttle input curve */
        JoystickCurve::CurveType turnCurve = JoystickCurve::SIGMOID;        /**< Turn input curve */
        double throttleGain = 1.0;                  /**< Throttle curve intensity */
        double turnGain = 1.0;                      /**< Turn curve intensity */
        bool enableDeadzone = true;                 /**< Enable input deadzone */
        double deadzoneThreshold = 5.0;             /**< Deadzone threshold (0-127) */
    };

    /**
     * @brief Default constructor with standard configuration
     */
    DriverControl() : config_(getDefaultConfig()) {}

    /**
     * @brief Constructor with custom configuration
     *
     * @param config Drive configuration settings
     */
    DriverControl(const DriveConfig& config) : config_(config) {}

    /**
     * @brief Update drive configuration at runtime
     *
     * Allows changing drive settings during operation, useful for
     * driver preference adjustments or mode switching.
     *
     * @param config New drive configuration
     */
    void setConfig(const DriveConfig& config) {
        config_ = config;
    }

    /**
     * @brief Get current drive configuration
     *
     * @return Current drive configuration settings
     */
    const DriveConfig& getConfig() const {
        return config_;
    }

    /**
     * @brief Calculate motor powers from controller input
     *
     * Processes controller input according to the current drive configuration
     * and calculates appropriate motor powers for left and right drive motors.
     * All curves, deadzones, and drive mode logic are applied automatically.
     *
     * @param controller PROS controller reference
     * @param leftMotor Output power for left motors (-127 to 127)
     * @param rightMotor Output power for right motors (-127 to 127)
     *
     * @note Motor powers are automatically clamped to valid range
     */
    void calculateDrivePowers(pros::Controller& controller, double& leftMotor, double& rightMotor) {
        double throttle = 0.0;
        double turn = 0.0;

        // Get raw controller inputs
        switch (config_.type) {
            case ARCADE:
                throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                break;

            case TANK:
                leftMotor = applyCurveToInput(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), config_.throttleCurve, config_.throttleGain);
                rightMotor = applyCurveToInput(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), config_.throttleCurve, config_.throttleGain);
                return;

            case CHEESY:
                throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                // Cheesy Drive quick turn with right bumper
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                    turn = turn * 2.0; // Boost turn for quick turns
                }
                break;

            case CURVATURE:
                throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                // Curvature drive: turn controls radius, not speed
                break;

            case GTA:
                // For VEX V5, shoulder buttons are digital, so we use them as on/off
                double brake = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) ? 127.0 : 0.0;
                double accelerate = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ? 127.0 : 0.0;
                throttle = accelerate - brake;
                turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
                break;
        }

        // Apply curves
        throttle = applyCurveToInput(throttle, config_.throttleCurve, config_.throttleGain);
        turn = applyCurveToInput(turn, config_.turnCurve, config_.turnGain);

        // Calculate motor powers based on drive type
        switch (config_.type) {
            case ARCADE:
                leftMotor = throttle + turn;
                rightMotor = throttle - turn;
                break;

            case CHEESY:
                leftMotor = throttle + turn;
                rightMotor = throttle - turn;
                break;

            case CURVATURE:
                // Curvature drive implementation
                if (throttle == 0.0) {
                    // Pure rotation when stopped
                    leftMotor = turn;
                    rightMotor = -turn;
                } else {
                    // Curved movement
                    double radius = 1.0 / std::max(std::abs(turn), 0.01);
                    leftMotor = throttle * (radius + 1.0) / (2.0 * radius);
                    rightMotor = throttle * (radius - 1.0) / (2.0 * radius);
                }
                break;

            case GTA:
                leftMotor = throttle + turn;
                rightMotor = throttle - turn;
                break;

            default:
                leftMotor = 0.0;
                rightMotor = 0.0;
                break;
        }

        // Clamp outputs
        leftMotor = std::max(-127.0, std::min(127.0, leftMotor));
        rightMotor = std::max(-127.0, std::min(127.0, rightMotor));
    }

private:
    DriveConfig config_; /**< Current drive configuration */

    /**
     * @brief Get default drive configuration
     *
     * @return Default configuration with sensible defaults
     */
    static DriveConfig getDefaultConfig() {
        DriveConfig config;
        config.type = ARCADE;
        config.throttleCurve = JoystickCurve::EXPONENTIAL;
        config.turnCurve = JoystickCurve::SIGMOID;
        config.throttleGain = 1.0;
        config.turnGain = 1.0;
        config.enableDeadzone = true;
        config.deadzoneThreshold = 5.0;
        return config;
    }

    /**
     * @brief Apply curve and deadzone to input value
     *
     * @param input Raw input value
     * @param curveType Curve type to apply
     * @param gain Curve gain factor
     * @return Processed output value
     */
    double applyCurveToInput(double input, JoystickCurve::CurveType curveType, double gain) {
        if (config_.enableDeadzone && std::abs(input) < config_.deadzoneThreshold) {
            return 0.0;
        }
        return JoystickCurve::applyCurve(input, curveType, gain);
    }
};

} // namespace ls