/** @file gains.h
 * @brief PID gain configuration structures for LibStoga robotics framework
 *
 * This file provides structured configuration for PID (Proportional-Integral-Derivative)
 * controller gains and related parameters. It centralizes gain management for
 * consistent tuning across all control systems in the framework.
 *
 * Key Features:
 * - Structured PID gain storage
 * - Configurable integral windup protection
 * - Zero-crossing reset control
 * - Integral limiting for stability
 * - Type-safe gain management
 *
 * PID Gain Components:
 * - **Proportional (kP)**: Response to current error
 * - **Integral (kI)**: Correction for accumulated error
 * - **Derivative (kD)**: Prediction based on error rate of change
 * - **Windup**: Maximum integral accumulation to prevent overshoot
 * - **Zero-crossing Reset**: Reset integral when error crosses zero
 * - **Integral Limit**: Maximum absolute value for integral term
 *
 * @author 6121D (The Pioneers)
 * @version 3.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup control
 */

#ifndef LS_GAINS_H
#define LS_GAINS_H

namespace ls {

    /**
     * @brief Complete PID controller gain configuration
     *
     * This structure encapsulates all parameters needed to configure a PID controller,
     * including the three main gains (P, I, D) and advanced features like integral
     * windup protection and zero-crossing reset.
     *
     * The structure provides sensible defaults that work for many applications,
     * but should be tuned for specific use cases.
     */
    struct PIDGains {
        float kP{0.0f};                    /**< Proportional gain - response to current error */
        float kI{0.0f};                    /**< Integral gain - correction for accumulated error */
        float kD{0.0f};                    /**< Derivative gain - prediction based on error rate */
        float windup{10.0f};               /**< Integral windup protection range */
        bool resetOnZeroCross{true};       /**< Reset integral when error crosses zero */
        float integralLimit{1000.0f};      /**< Maximum absolute value for integral term */

        /**
         * @brief Default constructor with sensible defaults
         *
         * Initializes gains to zero (disabled) with reasonable safety limits.
         * These defaults prevent runaway behavior while allowing easy tuning.
         */
        PIDGains() = default;

        /**
         * @brief Constructor with basic PID gains
         *
         * Creates a PIDGains structure with the three main gains, using
         * default values for advanced features.
         *
         * @param p Proportional gain
         * @param i Integral gain
         * @param d Derivative gain
         */
        PIDGains(float p, float i, float d)
            : kP(p), kI(i), kD(d) {}

        /**
         * @brief Constructor with all parameters
         *
         * Creates a complete PIDGains configuration with all available parameters.
         *
         * @param p Proportional gain
         * @param i Integral gain
         * @param d Derivative gain
         * @param w Windup protection range
         * @param reset Whether to reset on zero crossing
         * @param limit Integral term limit
         */
        PIDGains(float p, float i, float d, float w, bool reset, float limit)
            : kP(p), kI(i), kD(d), windup(w), resetOnZeroCross(reset), integralLimit(limit) {}
    };

} // namespace ls

#endif // LS_GAINS_H
