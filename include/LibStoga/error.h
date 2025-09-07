/** @file error.h
 * @brief Error handling system for LibStoga robotics framework
 *
 * This file provides a comprehensive error handling system specifically designed
 * for robotics applications running on the PROS platform. The system includes
 * both compile-time assertions and runtime error handling with visual feedback
 * on the robot's LCD screen.
 *
 * Key Features:
 * - Runtime error display on PROS LCD
 * - File and line number reporting for debugging
 * - Fatal error handling that halts execution
 * - Simple macro-based error throwing
 * - VEX Brain LCD integration for field debugging
 *
 * Error Handling Strategy:
 * - **Fatal Errors**: System halts with LCD display (use THROW_ERROR macro)
 * - **Debug Information**: File and line number automatically included
 * - **User Feedback**: Clear error messages displayed on robot LCD
 * - **Development Aid**: Helps identify issues during testing and competition
 *
 * @warning Fatal errors will halt robot execution. Use only for critical failures
 * that require immediate attention. For recoverable errors, consider alternative
 * error handling strategies.
 *
 * @author 6121D (The Pioneers)
 * @version 3.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup utilities
 */

#ifndef ERROR_LS_H
#define ERROR_LS_H

#include "api.h"

/**
 * @brief Macro for throwing fatal errors with automatic file/line information
 *
 * This macro provides a convenient way to throw fatal errors that will:
 * - Display the error message on the PROS LCD
 * - Show the file name and line number where the error occurred
 * - Halt program execution to prevent further damage
 *
 * Usage Example:
 * @code
 * if (motor == nullptr) {
 *     THROW_ERROR("Motor initialization failed");
 * }
 * @endcode
 *
 * @param error Error message string to display
 */
#define THROW_ERROR(error) __throw_error(error, __LINE__, __FILE__)

namespace ls {

    /**
     * @brief Internal function for error handling and display
     *
     * This function handles the actual error display and program halting.
     * It displays the error information on the PROS LCD screen and enters
     * an infinite loop to halt program execution.
     *
     * The function clears the first two lines of the LCD and displays:
     * - Line 0: File and line number information
     * - Line 1: The error message
     *
     * @param error The error message to display
     * @param line The line number where the error occurred
     * @param file The source file where the error occurred
     *
     * @note This function never returns - it halts execution indefinitely
     * @warning Only call this function through the THROW_ERROR macro
     */
    inline void __throw_error(const char* error, unsigned long line, const char* file) {
        pros::lcd::clear_line(0);
        pros::lcd::clear_line(1);
        pros::lcd::print(0, "at line %lu, file %s", line, file);
        pros::lcd::print(1, error);
        while (true) pros::delay(1000);
    }

} // namespace ls

#endif // ERROR_LS_H