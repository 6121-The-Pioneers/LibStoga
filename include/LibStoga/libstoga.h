/** @file libstoga.h
 * @brief Main header file for LibStoga - High-Performance PROS/C++ Robotics Framework
 *
 * This file serves as the central include point for the entire LibStoga library.
 * It includes all major components and provides a unified interface for robotics
 * applications using the PROS framework.
 *
 * LibStoga provides a comprehensive set of tools for robotics development including:
 * - Advanced odometry systems (IMU, three-wheel, MCL)
 * - PID and SmartPID controllers with adaptive features
 * - Chassis control with autonomous motion planning
 * - Pure pursuit path following
 * - System health monitoring and data logging
 * - Enhanced driver control interfaces
 *
 * @author 6121D (The Pioneers)
 * @version 3.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @note This library is designed for VEX Robotics Competition teams using PROS
 * @warning Requires PROS kernel >= 3.8.0 for full functionality
 */

#ifndef LIBSTOGA_LS_H
#define LIBSTOGA_LS_H

/**
 * @defgroup libstoga LibStoga Main Library
 * @brief High-Performance PROS/C++ Robotics Framework by 6121D
 *
 * LibStoga is a comprehensive robotics framework designed for VEX Robotics
 * Competition teams. It provides production-ready implementations of advanced
 * robotics algorithms including odometry, motion control, and autonomous routines.
 *
 * ## Key Features
 * - **Advanced Odometry**: IMU, three-wheel, and Monte Carlo Localization
 * - **Motion Control**: PID, SmartPID with adaptive features
 * - **Path Planning**: Pure pursuit with smooth curvature control
 * - **System Monitoring**: Real-time health checks and data logging
 * - **Driver Control**: Enhanced interfaces with customizable curves
 *
 * ## Architecture
 * The library is organized into several key modules:
 * - **Core**: Geometry, tracking, and basic utilities
 * - **Control**: PID controllers and motion algorithms
 * - **Motion**: Chassis control and autonomous routines
 * - **Monitoring**: System health and data collection
 *
 * ## Usage Example
 * @code
 * #include "libstoga.h"
 *
 * // Initialize odometry
 * ls::ThreeWheelOdom odom(params);
 *
 * // Create PID controllers
 * ls::PID drivePID(12.0f, 0.05f, 4.0f, 200, true);
 * ls::PID turnPID(18.0f, 0.0f, 6.5f, 0, true);
 *
 * // Initialize chassis
 * ls::Chassis chassis(rightMotors, leftMotors, odom, drivePID, turnPID);
 *
 * // Autonomous routine
 * chassis.moveToPoint(36, 12, 2500);
 * @endcode
 *
 * @see https://github.com/6121-The-Pioneers/LibStoga
 */

//==============================================================================
// CORE COMPONENTS
//==============================================================================

/** @defgroup core Core Components
 * @brief Fundamental building blocks of the robotics system
 * @{
 */

/**
 * @brief Geometry utilities for points, poses, angles, and coordinate transformations
 * @see geometry.h
 */
#include "geometry.h"

/**
 * @brief Tracking wheel interfaces and sensor management
 * @see tracking.h
 */
#include "tracking.h"

/**
 * @brief Odometry systems including IMU, three-wheel, and MCL implementations
 * @see odom.h
 */
#include "odom.h"

/**
 * @brief Monte Carlo Localization for advanced position estimation
 * @see mcl.h
 */
#include "mcl.h"

/**
 * @brief PID and SmartPID controllers with adaptive features
 * @see pid.h
 */
#include "pid.h"

/**
 * @brief SmartPID controller with sigmoid dampening and learning
 * @see spid.h
 */
#include "spid.h"

/**
 * @brief Main chassis control system for autonomous and driver control
 * @see chassis.h
 */
#include "chassis.h"

/**
 * @brief High-precision timer utilities for motion control
 * @see Timer.hpp
 */
#include "Timer.hpp"

/** @} */ // end of core group

//==============================================================================
// MOTION & CONTROL SYSTEMS
//==============================================================================

/** @defgroup motion Motion Control
 * @brief High-level motion planning and control systems
 * @{
 */

/**
 * @brief Pure pursuit path following with lookahead and curvature control
 * @see pure_pursuit.h
 */
#include "pure_pursuit.h"

/**
 * @brief Configuration management for chassis and control systems
 * @see config.h
 */
#include "config.h"

/** @} */ // end of motion group

//==============================================================================
// ENHANCED FEATURES
//==============================================================================

/** @defgroup enhanced Enhanced Features
 * @brief Advanced features for competition robotics
 * @{
 */

/**
 * @brief Enhanced driver control with customizable curves and deadzones
 * @see driver_control.h
 */
#include "driver_control.h"

/**
 * @brief Data logging system for tuning and analysis
 * @see data_logger.h
 */
#include "data_logger.h"

/**
 * @brief Enhanced chassis with advanced autonomous capabilities
 * @see enhanced_chassis.h
 */
#include "enhanced_chassis.h"

/**
 * @brief System health monitoring and diagnostics
 * @see system_health.h
 */
#include "system_health.h"

/** @} */ // end of enhanced group

//==============================================================================
// UTILITY COMPONENTS
//==============================================================================

/** @defgroup utilities Utilities
 * @brief Helper classes and utilities
 * @{
 */

/**
 * @brief Error handling and exception management
 * @see error.h
 */
#include "error.h"

/**
 * @brief PID gain structures and management
 * @see gains.h
 */
#include "gains.h"

/**
 * @brief Autonomous tuning utilities (experimental)
 * @see auton_tuner.h
 */
#include "auton_tuner.h"

/** @} */ // end of utilities group

#endif // !LIBSTOGA_LS_H
