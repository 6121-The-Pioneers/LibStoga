#pragma once
#include <cmath>
#include <algorithm>

namespace ls {

/**
 * @brief Joystick curve utilities for enhanced driver control
 */
class JoystickCurve {
public:
    enum CurveType {
        LINEAR,           // No curve, direct mapping
        EXPONENTIAL,      // Exponential curve for fine control at low speeds
        SIGMOID,          // S-curve for smooth acceleration
        CUBIC,           // Cubic curve for aggressive low-speed control
        CHEESY_DRIVE     // Special curve optimized for Cheesy Drive
    };

    /**
     * @brief Apply a curve to joystick input
     * @param input Raw joystick input (-127 to 127)
     * @param type Type of curve to apply
     * @param gain Curve intensity (0.0 = linear, higher = more aggressive)
     * @return Curved output (-127 to 127)
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
     * @brief Apply exponential curve (good for fine control)
     */
    static double expoCurve(double input, double gain) {
        return std::copysign(std::pow(std::abs(input), 1.0 + gain), input);
    }

    /**
     * @brief Apply sigmoid curve (smooth S-curve)
     */
    static double sigmoidCurve(double input, double gain) {
        double scaled = input * gain * 3.0; // Scale for better curve shape
        return std::tanh(scaled);
    }

    /**
     * @brief Apply cubic curve (aggressive low-speed control)
     */
    static double cubicCurve(double input, double gain) {
        double absInput = std::abs(input);
        double cubed = std::pow(absInput, 3.0);
        return std::copysign(cubed + (absInput * gain), input);
    }

    /**
     * @brief Cheesy Drive optimized curve
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
 * @brief Enhanced driver control with configurable curves and drive types
 */
class DriverControl {
public:
    enum DriveType {
        ARCADE,      // Single stick: throttle + turn
        TANK,        // Dual stick: left/right independently
        CHEESY,      // Cheesy Drive: throttle + quick turn
        CURVATURE,   // Curvature drive: throttle + radius control
        GTA         // GTA-style: left trigger brake, right trigger accelerate
    };

    struct DriveConfig {
        DriveType type = ARCADE;
        JoystickCurve::CurveType throttleCurve = JoystickCurve::EXPONENTIAL;
        JoystickCurve::CurveType turnCurve = JoystickCurve::SIGMOID;
        double throttleGain = 1.0;
        double turnGain = 1.0;
        bool enableDeadzone = true;
        double deadzoneThreshold = 5.0;
    };

    DriverControl() : config_(getDefaultConfig()) {}
    DriverControl(const DriveConfig& config) : config_(config) {}

    /**
     * @brief Update drive configuration
     */
    void setConfig(const DriveConfig& config) {
        config_ = config;
    }

    /**
     * @brief Get current drive configuration
     */
    const DriveConfig& getConfig() const {
        return config_;
    }

    /**
     * @brief Calculate motor powers based on controller input and drive type
     * @param controller PROS controller reference
     * @param leftMotor Output for left motor power (-127 to 127)
     * @param rightMotor Output for right motor power (-127 to 127)
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
    DriveConfig config_;

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

    double applyCurveToInput(double input, JoystickCurve::CurveType curveType, double gain) {
        if (config_.enableDeadzone && std::abs(input) < config_.deadzoneThreshold) {
            return 0.0;
        }
        return JoystickCurve::applyCurve(input, curveType, gain);
    }
};

} // namespace ls