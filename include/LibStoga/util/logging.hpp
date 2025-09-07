/** @file logging.hpp
 * @brief Lightweight logging utilities and stream operators for LibStoga debugging
 *
 * This file provides debugging and logging utilities for the LibStoga framework,
 * including stream operators for common LibStoga types and a configurable logging
 * macro. These utilities help with debugging autonomous routines, monitoring
 * system state, and troubleshooting robot behavior.
 *
 * Key Features:
 * - Stream operators for Angle, Position, and other LibStoga types
 * - Configurable logging macro with compile-time enable/disable
 * - Formatted output for debugging and data analysis
 * - Lightweight design with minimal performance impact
 *
 * Stream Operators:
 * - **Angle**: Outputs angle in degrees with "deg" suffix
 * - **Position**: Outputs X, Y coordinates and heading angle
 * - **Custom Types**: Easily extensible for other LibStoga classes
 *
 * Logging Macro:
 * - **Compile-time Control**: Enable/disable with LS_STDLOG define
 * - **Standard Output**: Uses std::cout for console output
 * - **Timestamp Prefix**: Adds "[LS]" prefix for easy identification
 * - **Zero Overhead**: Disabled macro compiles to no-op
 *
 * Usage Examples:
 * @code
 * // Enable logging by defining LS_STDLOG before including this file
 * #define LS_STDLOG
 * #include "util/logging.hpp"
 *
 * // Log simple messages
 * LS_LOG("Autonomous routine started");
 *
 * // Log with variables using stream operators
 * Position currentPos = odometry.getPosition();
 * LS_LOG("Current position: " << currentPos);
 *
 * // Log angle information
 * Angle targetHeading(90.0);
 * LS_LOG("Turning to heading: " << targetHeading);
 *
 * // Complex logging with multiple values
 * double battery = pros::battery::get_voltage() / 1000.0;
 * LS_LOG("Status - Position: " << currentPos << ", Battery: " << battery << "V");
 * @endcode
 *
 * Compile-time Control:
 * @code
 * // To enable logging, add this before including logging.hpp:
 * #define LS_STDLOG
 *
 * // To disable logging (default), simply don't define LS_STDLOG
 * // The logging macro will compile to nothing, ensuring zero runtime overhead
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

#ifndef LS_LOGGING_HPP
#define LS_LOGGING_HPP

#include <iostream>
#include "LibStoga/geometry.h"
#include "LibStoga/odom.h"

namespace ls {

/**
 * @brief Stream operator for Angle type
 *
 * Outputs an Angle object to a stream in a human-readable format.
 * Displays the angle value in degrees followed by "deg" suffix.
 *
 * @param os Output stream
 * @param a Angle object to output
 * @return Reference to the output stream
 *
 * @note Output format: "45.2deg"
 */
inline std::ostream& operator<<(std::ostream& os, const Angle& a) {
    os << a.getAngle() << "deg"; return os;
}

/**
 * @brief Stream operator for Position type
 *
 * Outputs a Position object to a stream showing X, Y coordinates
 * and heading angle in a structured format.
 *
 * @param os Output stream
 * @param p Position object to output
 * @return Reference to the output stream
 *
 * @note Output format: "{X=24.5, Y=12.3, theta=45.2deg}"
 */
inline std::ostream& operator<<(std::ostream& os, const Position& p) {
    os << "{X=" << p.X << ", Y=" << p.Y << ", theta=" << p.theta.getAngle() << "}"; return os;
}

} // namespace ls

/**
 * @brief LibStoga logging macro
 *
 * A configurable logging macro that outputs messages to standard output
 * with a LibStoga prefix. The macro can be enabled or disabled at compile-time
 * using the LS_STDLOG preprocessor definition.
 *
 * When enabled (LS_STDLOG defined):
 * - Outputs "[LS] " prefix followed by the message
 * - Uses std::cout for output
 * - Automatically appends newline character
 *
 * When disabled (LS_STDLOG not defined):
 * - Compiles to no-op with zero runtime overhead
 * - Safe to leave in production code
 *
 * @param msg Message to log (can include stream operations)
 *
 * @note Define LS_STDLOG before including this file to enable logging
 * @note Message can include stream operations like: LS_LOG("Value: " << variable)
 */
#ifdef LS_STDLOG
  #define LS_LOG(msg) do { std::cout << "[LS] " << msg << '\n'; } while(0)
#else
  #define LS_LOG(msg) do {} while(0)
#endif

#endif // LS_LOGGING_HPP
