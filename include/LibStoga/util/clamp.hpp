/** @file clamp.hpp
 * @brief Mathematical utility functions for value clamping, scaling, and processing
 *
 * This file provides a collection of mathematical utility functions commonly
 * used in robotics applications, particularly for processing sensor data,
 * motor control signals, and user inputs. These functions help ensure
 * values stay within valid ranges and provide smooth control characteristics.
 *
 * Key Functions:
 * - Value clamping to specified ranges
 * - Deadband processing for noise reduction
 * - Linear scaling between different ranges
 * - Sign extraction for directional logic
 * - Motor power clamping for VEX V5 compatibility
 *
 * Common Use Cases:
 * - **Motor Control**: Clamp power values to valid ranges (-127 to 127)
 * - **Sensor Processing**: Apply deadbands to reduce noise from sensors
 * - **User Input**: Scale joystick inputs to appropriate control ranges
 * - **PID Output**: Limit controller outputs to safe operating ranges
 * - **Angle Wrapping**: Keep angles within 0-360 degree ranges
 *
 * Usage Examples:
 * @code
 * // Clamp motor power to safe range
 * double motorPower = pidController.getOutput();
 * double safePower = clampPower(motorPower);
 * leftMotor.move(safePower);
 *
 * // Apply deadband to joystick input
 * double rawInput = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
 * double processedInput = deadband(rawInput, 5.0); // 5-unit deadband
 *
 * // Scale sensor reading to different range
 * double distance = ultrasonic.get_value(); // 0-100 inches
 * double motorSpeed = scale(distance, 0, 100, -100, 100); // Scale to motor speed
 *
 * // Get direction sign for logic
 * int direction = sign(error); // -1, 0, or 1
 * if (direction > 0) {
 *     // Move forward
 * } else if (direction < 0) {
 *     // Move backward
 * }
 * @endcode
 *
 * @author 6121D (The Pioneers)
 * @version 1.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup utilities
 */

#pragma once
#include <algorithm>
#include <cmath>

namespace ls {

/**
 * @brief Generic value clamping function
 *
 * Constrains a value to fall within a specified range. This is a convenience
 * wrapper around std::clamp that maintains namespace consistency.
 *
 * @tparam T Value type (must support comparison operators)
 * @param v Value to clamp
 * @param lo Lower bound (inclusive)
 * @param hi Upper bound (inclusive)
 * @return Value constrained to [lo, hi] range
 *
 * @note If v < lo, returns lo. If v > hi, returns hi. Otherwise returns v.
 */
template <typename T>
inline T clamp(const T& v, const T& lo, const T& hi) {
    return std::clamp(v, lo, hi);
}

/**
 * @brief Clamp motor power to VEX V5 range
 *
 * Constrains motor power values to the valid range for VEX V5 motors.
 * This prevents motor damage from over-powering and ensures compatibility
 * with PROS motor control functions.
 *
 * @param v Raw motor power value
 * @return Power value clamped to [-127, 127] range
 *
 * @note VEX V5 motors accept power values from -127 (full reverse) to 127 (full forward)
 */
inline double clampPower(double v) { return clamp(v, -127.0, 127.0); }

/**
 * @brief Apply deadband to reduce noise
 *
 * Zeroes out small magnitude values to reduce noise and prevent
 * unwanted small movements. Values within the deadband range are
 * treated as zero, while larger values are passed through unchanged.
 *
 * @param v Input value
 * @param db Deadband threshold (must be >= 0)
 * @return Input value with deadband applied
 *
 * @note Commonly used with joystick inputs and sensor readings
 * @note db = 0 disables deadband (no effect)
 */
inline double deadband(double v, double db) { return (std::fabs(v) < db) ? 0.0 : v; }

/**
 * @brief Linear scaling between ranges
 *
 * Scales a value from one range to another using linear interpolation.
 * Optionally clamps the result to the output range bounds.
 *
 * @param v Input value to scale
 * @param inMin Input range minimum
 * @param inMax Input range maximum
 * @param outMin Output range minimum
 * @param outMax Output range maximum
 * @param clampResult Whether to clamp result to output range
 * @return Scaled value in output range
 *
 * @note If inMax == inMin, returns outMin to avoid division by zero
 * @warning Ensure inMin != inMax to avoid undefined behavior
 */
inline double scale(double v, double inMin, double inMax, double outMin, double outMax, bool clampResult = true) {
    if (inMax - inMin == 0) return outMin; // avoid div/0
    double t = (v - inMin) / (inMax - inMin);
    double out = outMin + t * (outMax - outMin);
    if (clampResult) return clamp(out, std::min(outMin, outMax), std::max(outMin, outMax));
    return out;
}

/**
 * @brief Extract sign of value
 *
 * Returns the sign of a value as an integer: -1 for negative,
 * 0 for zero, and 1 for positive. Useful for directional logic
 * and sign-preserving operations.
 *
 * @tparam T Value type (must support comparison operators)
 * @param v Input value
 * @return Sign as integer: -1 (negative), 0 (zero), or 1 (positive)
 *
 * @note Equivalent to: (v > 0) - (v < 0)
 */
template <typename T>
inline int sign(T v) { return (v > 0) - (v < 0); }

} // namespace ls
