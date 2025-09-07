/** @file chassis.h
 * @brief Chassis control system for LibStoga robotics framework
 *
 * This file provides the core Chassis class that manages robot movement and control.
 * It supports tank drive configuration with integrated odometry, PID control systems,
 * and autonomous motion primitives. The chassis system is designed for VEX robotics
 * platforms using the PROS framework.
 *
 * Key Features:
 * - Tank drive control with arcade drive support
 * - Integrated odometry for position tracking
 * - PID-based motion control for precise movements
 * - Autonomous motion primitives (drive, turn, point-to-point)
 * - Path following with pure pursuit algorithm
 * - Adaptive PID tuning capabilities
 * - Controller-based cancellation support
 *
 * @author 6121D (The Pioneers)
 * @version 3.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup core
 */

#ifndef LS_CHASSIS_H
#define LS_CHASSIS_H

#include "api.h"
#include "odom.h"
#include "pid.h"
#include "auton_tuner.h"
#include <memory>
#include <vector>

#include "pros/misc.hpp" // For controller button enums

namespace ls {

/**
 * @brief Configuration structure for Chassis behavior
 *
 * Defines various parameters that control chassis movement characteristics,
 * thresholds, and feature enablement flags.
 */
struct ChassisConfig {
    double turnSensitivity{1.0};     /**< Multiplier for turn input sensitivity */
    double lateralThreshold{1.0};    /**< Distance threshold for lateral movement completion (inches) */
    double angularThreshold{5.0};    /**< Angle threshold for turn completion (degrees) */
    unsigned int dwellTimeMs{500};   /**< Time to wait after movement completion before considering done (ms) */
    bool enableCancel{true};         /**< Whether to allow controller-based cancellation of movements */
    pros::controller_digital_e_t cancelButton{pros::E_CONTROLLER_DIGITAL_A}; /**< Controller button for cancellation */
    bool enableAdaptivePID{false};   /**< Whether to use adaptive PID tuning during autonomous */
};

/**
 * @brief Main chassis control class for robot movement
 *
 * The Chassis class provides comprehensive control over robot movement including:
 * - Manual control via tank or arcade drive
 * - Autonomous motion primitives with PID control
 * - Path following capabilities
 * - Integrated odometry for position feedback
 * - Adaptive PID tuning for optimal performance
 *
 * The class currently supports tank drive configuration with left/right motor groups.
 * Future versions may add support for other drive configurations (Mecanum, X-drive, etc.).
 *
 * @note All distances are in inches, angles in degrees (bearing system: 0° = North)
 * @warning Requires proper motor calibration and PID tuning for optimal performance
 */
class Chassis
{
private:
    std::vector<pros::Motor> left_motors;              /**< Left side motors */
    std::vector<pros::Motor> right_motors;             /**< Right side motors */
    std::shared_ptr<pros::MotorGroup> right;           /**< Right side motor group */
    std::shared_ptr<pros::MotorGroup> left;            /**< Left side motor group */
    std::shared_ptr<ls::AbstractOdom> odom;            /**< Odometry system for position tracking */

    std::shared_ptr<ls::AbstractPID> lateral_control;  /**< PID controller for forward/backward motion */
    std::shared_ptr<ls::AbstractPID> angular_control;  /**< PID controller for turning motion */

    double turn_sensitivity;                            /**< Turn input sensitivity multiplier */
    double threshold_lateral;                           /**< Lateral movement completion threshold */
    double threshold_angular;                           /**< Angular movement completion threshold */
    unsigned int threshold_timeout;                     /**< Dwell time after threshold met */
    bool enable_cancel;                                 /**< Controller cancellation enable flag */
    pros::controller_digital_e_t cancel_button;         /**< Controller button for cancellation */
    bool enable_adaptive_pid;                           /**< Adaptive PID enable flag */
    pros::Controller* controller;                       /**< PROS controller for user input */

    // Adaptive PID controllers
    std::unique_ptr<AdaptivePIDController> lateral_adaptive_controller;  /**< Adaptive lateral PID */
    std::unique_ptr<AdaptivePIDController> angular_adaptive_controller;  /**< Adaptive angular PID */

public:
    /**
     * @brief Set the controller for user input and cancellation
     *
     * Associates a PROS controller with the chassis for manual control
     * and autonomous movement cancellation.
     *
     * @param cont Reference to PROS controller object
     */
    void setController(pros::Controller& cont);

    /**
     * @brief Get the currently associated controller
     *
     * @return Pointer to the PROS controller (nullptr if not set)
     */
    pros::Controller* getController() { return controller; }

    /**
     * @brief Get access to the odometry system
     *
     * Provides access to the chassis's odometry system for position
     * tracking and pose estimation.
     *
     * @return Shared pointer to the odometry system
     */
    std::shared_ptr<ls::AbstractOdom> getOdom();

    /**
     * @brief Get the lateral motion PID controller
     *
     * @return Shared pointer to the lateral PID controller
     */
    std::shared_ptr<AbstractPID> getLateralControl();

    /**
     * @brief Get the angular motion PID controller
     *
     * @return Shared pointer to the angular PID controller
     */
    std::shared_ptr<AbstractPID> getAngularControl();

    /**
     * @brief Get the left motor vector
     *
     * @return Vector of left motors
     */
    std::vector<pros::Motor>& getLeftMotorVec();

    /**
     * @brief Get the right motor vector
     *
     * @return Vector of right motors
     */
    std::vector<pros::Motor>& getRightMotorVec();

    /**
     * @brief Get the left motor group
     *
     * @return Shared pointer to the left motor group
     */
    std::shared_ptr<pros::MotorGroup> getLeftMotors();

    /**
     * @brief Get the right motor group
     *
     * @return Shared pointer to the right motor group
     */
    std::shared_ptr<pros::MotorGroup> getRightMotors();

    /**
     * @brief Check if adaptive PID is enabled
     *
     * @return true if adaptive PID is enabled, false otherwise
     */
    bool isAdaptivePIDEnabled() const;

    /**
     * @brief Update adaptive PID controllers
     *
     * Should be called periodically during autonomous operation to
     * allow adaptive PID controllers to adjust parameters based on
     * system performance.
     */
    void updateAdaptivePID();

    /**
     * @brief Construct a new Chassis object
     *
     * Creates a chassis with the specified motor groups, odometry system,
     * and PID controllers. Configuration can be customized via the config parameter.
     *
     * @param _left_motors Vector of left side motors
     * @param _right_motors Vector of right side motors
     * @param _right Shared pointer to right side motor group
     * @param _left Shared pointer to left side motor group
     * @param _odom Shared pointer to odometry system
     * @param _lateral_control Shared pointer to lateral PID controller
     * @param _angular_control Shared pointer to angular PID controller
     * @param cfg Configuration struct (uses defaults if not provided)
     */
    explicit Chassis(std::vector<pros::Motor> _left_motors,
                     std::vector<pros::Motor> _right_motors,
                     std::shared_ptr<pros::MotorGroup> _right,
                     std::shared_ptr<pros::MotorGroup> _left,
                     std::shared_ptr<ls::AbstractOdom> _odom,
                     std::shared_ptr<ls::AbstractPID> _lateral_control,
                     std::shared_ptr<ls::AbstractPID> _angular_control,
                     const ChassisConfig& cfg = {});

    // Rule of five - allow copying and moving
    Chassis(const Chassis&) = default;
    Chassis& operator=(const Chassis&) = default;
    Chassis(Chassis&&) = default;
    Chassis& operator=(Chassis&&) = default;

    /**
     * @brief Move chassis to a specific point in Cartesian coordinates
     *
     * Moves the robot to the specified (X, Y) position using odometry feedback
     * and PID control. The origin (0, 0) is the latest odometry reset point.
     *
     * @param X Target X coordinate in inches
     * @param Y Target Y coordinate in inches
     * @param timeout Maximum time to complete movement in milliseconds
     * @param reverse Whether to drive in reverse (default: false)
     *
     * @note This function is currently marked as TODO and may not be fully implemented
     */
    void moveToPoint(double X, double Y, unsigned int timeout, bool reverse = false);

    /**
     * @brief Turn chassis to face a specific point
     *
     * Rotates the robot to face the specified (X, Y) point while maintaining
     * current position. Uses odometry for position feedback.
     *
     * @param X Target X coordinate to face
     * @param Y Target Y coordinate to face
     * @param timeout Maximum time to complete turn in milliseconds
     * @param reverse Whether to face the point in reverse (180° offset)
     *
     * @note This function is currently marked as TODO and may not be fully implemented
     */
    void turnToPoint(double X, double Y, unsigned int timeout, bool reverse = false);

    /**
     * @brief Turn chassis to a specific absolute angle
     *
     * Rotates the robot to face the specified absolute bearing angle using
     * PID control and odometry feedback.
     *
     * @param angle Target angle in degrees (0° = North, 90° = East)
     * @param timeout Maximum time to complete turn in milliseconds
     * @return Actual angle achieved when movement completed or timed out
     */
    double turnToAngle(double angle, unsigned int timeout);

    /**
     * @brief Drive forward/backward a relative distance
     *
     * Moves the robot forward or backward by the specified distance from
     * its current position using PID control.
     *
     * @param distance Distance to drive in inches (positive = forward, negative = backward)
     * @param timeout Maximum time to complete movement in milliseconds
     * @return Actual distance traveled when movement completed or timed out
     */
    double driveRelative(double distance, unsigned int timeout);

    /**
     * @brief Turn by a relative angle change
     *
     * Rotates the robot by the specified angle delta from its current heading.
     * Positive angles turn clockwise, negative turn counterclockwise.
     *
     * @param deltaAngle Angle change in degrees (positive = clockwise)
     * @param timeout Maximum time to complete turn in milliseconds
     * @return Actual angle change achieved when movement completed or timed out
     */
    double turnRelative(double deltaAngle, unsigned int timeout);

    /**
     * @brief Manual tank drive control
     *
     * Controls the chassis using tank drive logic where left and right
     * sides are controlled independently. Values should be in range [-1, 1]
     * or [-127, 127] depending on motor configuration.
     *
     * @param left_y Left side motor power/input
     * @param right_y Right side motor power/input
     */
    void tank_drive(double left_y, double right_y);

    /**
     * @brief Manual arcade drive control
     *
     * Controls the chassis using arcade drive logic with separate forward/backward
     * and turn inputs. This provides more intuitive control for drivers.
     *
     * @param forward Forward/backward movement input (-1 to 1 or -127 to 127)
     * @param turn Left/right turn input (-1 to 1 or -127 to 127)
     */
    void arcade(double forward, double turn);

    /**
     * @brief Follow a path using pure pursuit algorithm
     *
     * Executes path following using the pure pursuit algorithm to smoothly
     * navigate through a series of waypoints. The lookahead distance determines
     * how far ahead on the path the robot aims.
     *
     * @param path Vector of waypoints defining the path
     * @param lookahead Lookahead distance in inches for path following
     * @param timeout Maximum time to complete path following in milliseconds
     */
    void followPath(const Path& path, double lookahead, unsigned int timeout);

private:
    /**
     * @brief Calculate movement priority for coordinated motion
     *
     * Calculates a priority value from [0, 1] representing the percentage
     * by which turn motion has priority over lateral motion. This is useful
     * for smooth point-to-point movements that require both translation and rotation.
     *
     * @param angle Target angle for movement coordination
     * @return Priority value between 0.0 and 1.0
     */
    double move_priority(Angle& angle) const;
};

} // namespace ls

#endif // LS_CHASSIS_H