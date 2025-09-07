/** @file config.h
 * @brief Configuration management system for LibStoga robotics framework
 *
 * This file provides comprehensive configuration structures and a builder pattern
 * for setting up robot components. It centralizes all configuration parameters
 * for drivetrain, odometry, and control systems, making robot setup more
 * organized and maintainable.
 *
 * Key Features:
 * - Builder pattern for fluent chassis configuration
 * - Structured configuration for all major subsystems
 * - Type-safe parameter validation
 * - Centralized configuration management
 * - Easy serialization and deserialization
 *
 * Configuration Areas:
 * - **Drivetrain**: Motor ports, dimensions, and mechanical properties
 * - **Odometry**: Sensor configuration and positioning parameters
 * - **Control**: PID gains, tolerances, and control behavior
 * - **Tolerances**: Movement completion thresholds and timing
 *
 * @author 6121D (The Pioneers)
 * @version 3.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup core
 */

#pragma once
#include <memory>
#include <vector>

#include "pros/motors.hpp"
#include "odom.h"
#include "gains.h"
#include "pure_pursuit.h"

namespace ls {
    class Chassis; // Forward declaration

    /**
     * @brief Movement completion thresholds and timing parameters
     *
     * Defines the criteria for determining when autonomous movements are complete.
     * These parameters control the precision and timing of chassis movements.
     */
    struct ChassisTolerances {
        double lateralThresh{1.0};     /**< Distance threshold for drive completion (inches) */
        double angularThresh{5.0};     /**< Angle threshold for turn completion (degrees) */
        unsigned int dwellMs{300};     /**< Time movement must stay within threshold (milliseconds) */
    };

    /**
     * @brief Drivetrain mechanical and electrical configuration
     *
     * Contains all parameters related to the physical drivetrain including
     * motor ports, dimensions, and performance characteristics.
     */
    struct DrivetrainConfig {
        std::vector<int8_t> leftMotors;    /**< PROS port numbers for left side motors */
        std::vector<int8_t> rightMotors;   /**< PROS port numbers for right side motors */
        double trackWidth;                 /**< Distance between left and right wheels (inches) */
        double wheelDiameter;              /**< Diameter of drive wheels (inches) */
        double rpm;                        /**< Motor RPM rating (affects speed calculations) */
    };

    /**
     * @brief Odometry system configuration
     *
     * Defines which odometry implementation to use and provides all necessary
     * parameters for sensor setup and calibration.
     */
    struct OdomConfig {
        bool useThreeWheel;                    /**< True for 3-wheel odometry, false for IMU odometry */
        threewheel_odom_parameters_t threeWheelParams;  /**< Parameters for 3-wheel odometry */
        imu_odom_parameters_t imuParams;       /**< Parameters for IMU odometry */
    };

    /**
     * @brief Chassis controller configuration
     *
     * Comprehensive configuration for chassis control systems including PID
     * gains, movement tolerances, and advanced features.
     */
    struct ChassisControllerConfig {
        PIDGains driveGains;               /**< PID gains for forward/backward motion */
        PIDGains turnGains;                /**< PID gains for turning motion */
        ChassisTolerances tolerances;      /**< Movement completion thresholds */
        double turnSensitivity{1.0};       /**< Multiplier for turn input sensitivity */
        bool enableAdaptivePID{false};     /**< Enable adaptive PID that adjusts to conditions */
    };

    /**
     * @brief Builder pattern for chassis configuration and construction
     *
     * Provides a fluent interface for configuring and building a Chassis object.
     * This pattern ensures all required parameters are set before construction
     * and provides a clean, readable configuration process.
     *
     * Usage Example:
     * @code
     * auto chassis = ChassisBuilder{}
     *     .withDrivetrain({
     *         .leftMotors = {1, 2},
     *         .rightMotors = {3, 4},
     *         .trackWidth = 12.0,
     *         .wheelDiameter = 4.0,
     *         .rpm = 360
     *     })
     *     .withOdom({
     *         .useThreeWheel = true,
     *         .threeWheelParams = {...}
     *     })
     *     .withController({
     *         .driveGains = {1.2, 0.05, 0.8},
     *         .turnGains = {2.0, 0.1, 1.0},
     *         .tolerances = {1.0, 5.0, 300}
     *     })
     *     .build();
     * @endcode
     */
    class ChassisBuilder {
    public:
        /**
         * @brief Configure the drivetrain parameters
         *
         * @param cfg Drivetrain configuration including motors and dimensions
         * @return Reference to this builder for method chaining
         */
        ChassisBuilder& withDrivetrain(const DrivetrainConfig& cfg);

        /**
         * @brief Configure the odometry system
         *
         * @param cfg Odometry configuration including sensor setup
         * @return Reference to this builder for method chaining
         */
        ChassisBuilder& withOdom(const OdomConfig& cfg);

        /**
         * @brief Configure the chassis controller
         *
         * @param cfg Controller configuration including PID gains and tolerances
         * @return Reference to this builder for method chaining
         */
        ChassisBuilder& withController(const ChassisControllerConfig& cfg);

        /**
         * @brief Build and return the configured chassis
         *
         * Constructs a Chassis object using all the configured parameters.
         * All required configurations must be set before calling this method.
         *
         * @return Shared pointer to the constructed chassis
         * @throws std::runtime_error if required configurations are missing
         */
        std::shared_ptr<Chassis> build();

    private:
        DrivetrainConfig drivetrainConfig;        /**< Stored drivetrain configuration */
        OdomConfig odomConfig;                    /**< Stored odometry configuration */
        ChassisControllerConfig controllerConfig; /**< Stored controller configuration */
    };

} // namespace ls
