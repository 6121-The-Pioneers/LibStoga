/** @file enhanced_chassis.h
 * @brief Advanced chassis control interface with LemLib-inspired movement parameters
 *
 * This file provides an enhanced chassis control system with sophisticated movement
 * parameters inspired by LemLib. It offers fine-grained control over robot motion
 * including speed profiles, early exit conditions, path following parameters,
 * and advanced turning behaviors.
 *
 * Key Features:
 * - Configurable speed profiles with min/max speeds
 * - Early exit conditions for precise stopping
 * - Advanced path following with carrot chasing algorithm
 * - Multiple turning modes (swing turns, point-to-point)
 * - Asynchronous operation support
 * - Comprehensive motion state management
 *
 * Movement Types:
 * - **Point-to-Point**: Move to specific coordinates with heading control
 * - **Heading Turns**: Rotate to specific angles with direction control
 * - **Swing Turns**: Pivot turns with one side locked for precision
 * - **Path Following**: Follow predefined paths with lookahead control
 * - **Pose Control**: Combined position and heading movement
 *
 * Advanced Parameters:
 * - **Lead/Carrot**: Controls how far ahead on the path the robot aims
 * - **Horizontal Drift**: Adjusts cornering aggressiveness
 * - **Early Exit**: Stop movement when close enough to target
 * - **Speed Profiles**: Minimum and maximum speed constraints
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
#include <cstdint>

namespace ls {

/**
 * @brief Angular rotation direction options
 *
 * Specifies the direction for rotational movements, allowing control
 * over whether turns should be clockwise, counter-clockwise, or
 * automatically choose the shortest path.
 */
enum class AngularDirection {
    AUTO,           /**< Automatically choose shortest rotation direction */
    CW_CLOCKWISE,   /**< Force clockwise rotation */
    CCW_COUNTERCLOCKWISE  /**< Force counter-clockwise rotation */
};

/**
 * @brief Parameters for point-to-point movement
 *
 * Configures advanced movement behavior when moving to a specific
 * coordinate position. These parameters control speed, accuracy,
 * and path following characteristics.
 */
struct MoveToPointParams {
    bool forwards = true;           /**< Move forwards (true) or backwards (false) to target */
    double maxSpeed = 127.0;        /**< Maximum linear speed (0-127) */
    double minSpeed = 0.0;          /**< Minimum linear speed (0-127) for final approach */
    double earlyExitRange = 0.0;    /**< Exit early when within this distance (inches) of target */
    double lead = 0.6;              /**< Carrot point multiplier (0-1) for path following lookahead */
    double horizontalDrift = 2.0;   /**< Cornering aggressiveness (higher = faster, sharper turns) */
};

/**
 * @brief Parameters for heading rotation movements
 *
 * Controls the behavior of turns to specific heading angles,
 * including direction preferences and speed constraints.
 */
struct TurnToHeadingParams {
    AngularDirection direction = AngularDirection::AUTO;  /**< Preferred turn direction */
    double maxSpeed = 127.0;        /**< Maximum angular speed (0-127) */
    double minSpeed = 0.0;          /**< Minimum angular speed (0-127) for final approach */
    double earlyExitRange = 0.0;    /**< Exit early when within this angle (degrees) of target */
};

/**
 * @brief Parameters for pose (position + heading) movement
 *
 * Advanced parameters for moving to a complete pose consisting of
 * both position coordinates and target heading angle.
 */
struct MoveToPoseParams {
    bool forwards = true;           /**< Move forwards (true) or backwards (false) to pose */
    double maxSpeed = 127.0;        /**< Maximum linear speed (0-127) */
    double minSpeed = 0.0;          /**< Minimum linear speed (0-127) for final approach */
    double earlyExitRange = 0.0;    /**< Exit early when within this distance (inches) of pose */
    double lead = 0.6;              /**< Carrot point multiplier (0-1) for path following */
    double horizontalDrift = 2.0;   /**< Cornering aggressiveness for curved paths */
};

/**
 * @brief Parameters for swing turn movements
 *
 * Configures pivot turns where one side of the robot remains stationary
 * while the other side rotates around it. Useful for precise angular movements.
 */
struct SwingToHeadingParams {
    AngularDirection direction = AngularDirection::AUTO;  /**< Preferred turn direction */
    double maxSpeed = 127.0;        /**< Maximum angular speed (0-127) */
    double minSpeed = 0.0;          /**< Minimum angular speed (0-127) for final approach */
    double earlyExitRange = 0.0;    /**< Exit early when within this angle (degrees) of target */
};

/**
 * @brief Parameters for swing turns to face a point
 *
 * Advanced swing turn parameters that rotate the robot to face a specific
 * point while keeping one side stationary for precise positioning.
 */
struct SwingToPointParams {
    bool forwards = true;           /**< Face point with front (true) or back (false) */
    AngularDirection direction = AngularDirection::AUTO;  /**< Preferred turn direction */
    double maxSpeed = 127.0;        /**< Maximum angular speed (0-127) */
    double minSpeed = 0.0;          /**< Minimum angular speed (0-127) for final approach */
    double earlyExitRange = 0.0;    /**< Exit early when within this angle (degrees) of target */
};

/**
 * @brief Parameters for path following movements
 *
 * Configures behavior when following predefined paths, including
 * speed control, direction, and lookahead characteristics.
 */
struct FollowPathParams {
    double maxSpeed = 127.0;        /**< Maximum speed along path (0-127) */
    double minSpeed = 0.0;          /**< Minimum speed along path (0-127) */
    bool forwards = true;           /**< Follow path forwards (true) or backwards (false) */
    double lead = 0.6;              /**< Carrot point multiplier (0-1) for lookahead distance */
    bool async = true;              /**< Run path following asynchronously */
};

/**
 * @brief Enhanced chassis control interface with advanced movement capabilities
 *
 * The EnhancedChassis class provides a comprehensive interface for advanced
 * robot movement control. It supports multiple movement types with fine-grained
 * parameter control, inspired by the LemLib library but adapted for LibStoga.
 *
 * This is an abstract base class that must be implemented by specific chassis
 * controllers (differential drive, holonomic, etc.). It provides a consistent
 * API for complex autonomous movements and driver assistance features.
 *
 * Movement Capabilities:
 * - **Point Movement**: Move to specific coordinates with advanced path control
 * - **Pose Control**: Combined position and heading movement
 * - **Angular Turns**: Precise heading rotations with direction control
 * - **Swing Turns**: Pivot turns for accurate positioning
 * - **Path Following**: Follow predefined paths with lookahead control
 *
 * Motion Management:
 * - Asynchronous operation support
 * - Motion state monitoring
 * - Early exit conditions
 * - Motion cancellation and queuing
 * - Pose tracking and resetting
 *
 * Usage Example:
 * @code
 * // Move to a point with custom parameters
 * MoveToPointParams params;
 * params.forwards = true;
 * params.maxSpeed = 100.0;
 * params.earlyExitRange = 1.0;  // Stop when within 1 inch
 * params.lead = 0.8;            // Look further ahead
 *
 * chassis->moveToPoint(24.0, 24.0, 5000, params);
 *
 * // Wait for completion or check status
 * if (chassis->isInMotion()) {
 *     chassis->waitUntilDone();
 * }
 *
 * // Swing turn to face a point
 * SwingToPointParams swingParams;
 * swingParams.forwards = true;
 * swingParams.maxSpeed = 80.0;
 *
 * chassis->swingToPoint(36.0, 12.0, true, 3000, swingParams);
 * @endcode
 */
class EnhancedChassis {
public:
    /**
     * @brief Move to a specific point with advanced parameters
     *
     * Moves the robot to the specified coordinates using advanced path
     * following with configurable speed profiles and early exit conditions.
     *
     * @param x Target X coordinate (inches)
     * @param y Target Y coordinate (inches)
     * @param timeout Maximum time to complete movement (milliseconds)
     * @param params Movement configuration parameters
     *
     * @note Movement is blocking unless implemented as asynchronous
     */
    virtual void moveToPoint(double x, double y, int timeout, const MoveToPointParams& params = {}) = 0;

    /**
     * @brief Rotate to a specific heading with advanced parameters
     *
     * Turns the robot to face the specified heading angle with configurable
     * speed profiles and direction preferences.
     *
     * @param heading Target heading angle (degrees, 0 = forward)
     * @param timeout Maximum time to complete turn (milliseconds)
     * @param params Turn configuration parameters
     */
    virtual void turnToHeading(double heading, int timeout, const TurnToHeadingParams& params = {}) = 0;

    /**
     * @brief Move to a complete pose (position + heading) with advanced parameters
     *
     * Moves to specified coordinates while simultaneously rotating to the
     * target heading, providing smooth coordinated motion.
     *
     * @param x Target X coordinate (inches)
     * @param y Target Y coordinate (inches)
     * @param heading Target heading angle (degrees)
     * @param timeout Maximum time to complete movement (milliseconds)
     * @param params Pose movement configuration parameters
     */
    virtual void moveToPose(double x, double y, double heading, int timeout, const MoveToPoseParams& params = {}) = 0;

    /**
     * @brief Perform swing turn to heading (one side locked)
     *
     * Executes a pivot turn where one side of the robot remains stationary
     * while the other side rotates, providing precise angular control.
     *
     * @param heading Target heading angle (degrees)
     * @param leftSideLocked True to lock left side, false to lock right side
     * @param timeout Maximum time to complete turn (milliseconds)
     * @param params Swing turn configuration parameters
     */
    virtual void swingToHeading(double heading, bool leftSideLocked, int timeout, const SwingToHeadingParams& params = {}) = 0;

    /**
     * @brief Perform swing turn to face a point (one side locked)
     *
     * Rotates the robot to face the specified point using a swing turn,
     * keeping one side stationary for precise positioning.
     *
     * @param x Target point X coordinate (inches)
     * @param y Target point Y coordinate (inches)
     * @param leftSideLocked True to lock left side, false to lock right side
     * @param timeout Maximum time to complete turn (milliseconds)
     * @param params Swing turn configuration parameters
     */
    virtual void swingToPoint(double x, double y, bool leftSideLocked, int timeout, const SwingToPointParams& params = {}) = 0;

    /**
     * @brief Follow a predefined path with advanced parameters
     *
     * Follows a series of waypoints using advanced path following algorithms
     * with lookahead control and configurable speed profiles.
     *
     * @param path Vector of points defining the path
     * @param lookahead Lookahead distance for path following (inches)
     * @param timeout Maximum time to complete path following (milliseconds)
     * @param params Path following configuration parameters
     */
    virtual void followPath(const std::vector<Point>& path, double lookahead, int timeout, const FollowPathParams& params = {}) = 0;

    /**
     * @brief Check if chassis is currently executing a motion
     *
     * @return true if robot is in motion, false if stationary
     */
    virtual bool isInMotion() const = 0;

    /**
     * @brief Wait until robot has traveled a specific distance
     *
     * Blocks execution until the robot has moved the specified distance
     * from its position when this method was called.
     *
     * @param distance Distance to wait for (inches)
     */
    virtual void waitUntil(double distance) = 0;

    /**
     * @brief Wait until current motion is complete
     *
     * Blocks execution until the current movement command finishes,
     * either by reaching the target or timing out.
     */
    virtual void waitUntilDone() = 0;

    /**
     * @brief Cancel the current motion
     *
     * Stops the currently executing movement command without affecting
     * any queued movements.
     */
    virtual void cancelMotion() = 0;

    /**
     * @brief Cancel all motions including queued movements
     *
     * Stops all movement commands, including any that are queued
     * for execution after the current movement.
     */
    virtual void cancelAllMotions() = 0;

    /**
     * @brief Get current robot pose
     *
     * @return Current pose containing position and heading
     */
    virtual Pose getPose() const = 0;

    /**
     * @brief Set current robot pose
     *
     * Updates the robot's position and heading tracking to the specified values.
     * Useful for resetting position after manual repositioning.
     *
     * @param pose New pose to set
     */
    virtual void setPose(const Pose& pose) = 0;

    /**
     * @brief Reset local position tracking (preserve heading)
     *
     * Resets the robot's position to (0,0) while maintaining the current
     * heading. Useful for relative movement after repositioning.
     */
    virtual void resetLocalPosition() = 0;

    /**
     * @brief Set brake mode for drivetrain motors
     *
     * Configures the braking behavior of the drivetrain motors when
     * power is removed or set to zero.
     *
     * @param mode PROS brake mode (COAST, BRAKE, HOLD)
     */
    virtual void setBrakeMode(pros::motor_brake_mode_e mode) = 0;
};

} // namespace ls