/** @file pure_pursuit.h
 * @brief Pure Pursuit path following algorithm for autonomous navigation
 *
 * This file implements the Pure Pursuit algorithm, a geometric path tracking
 * method commonly used in autonomous robotics. The algorithm calculates the
 * required curvature to follow a predefined path by finding a "lookahead point"
 * ahead of the robot and steering towards it.
 *
 * Key Features:
 * - Smooth path following with configurable lookahead distance
 * - Real-time curvature calculation based on robot pose
 * - Adaptive lookahead point selection
 * - Integration with odometry systems for position feedback
 * - Completion detection when reaching path end
 *
 * Algorithm Overview:
 * 1. Find the point on the path that is exactly "lookahead distance" ahead
 * 2. Calculate the curvature needed to reach that point
 * 3. Apply proportional control to minimize cross-track error
 * 4. Update robot control outputs (linear and angular velocity)
 *
 * @author 6121D (The Pioneers)
 * @version 3.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup motion
 */

#pragma once
#include "geometry.h"
#include "odom.h"
#include <optional>

namespace ls {

/**
 * @brief Output structure for Pure Pursuit calculations
 *
 * Contains the control outputs from the Pure Pursuit algorithm that
 * should be applied to the robot's motion controllers.
 */
struct PursuitOutput {
    double linear{0};  /**< Linear velocity command (forward/backward) */
    double angular{0}; /**< Angular velocity command (turning) */
    bool done{false};  /**< Whether the path following is complete */
};

/**
 * @brief Pure Pursuit path following controller
 *
 * Implements the Pure Pursuit algorithm for smooth autonomous path following.
 * The controller takes a series of waypoints and generates velocity commands
 * to guide the robot along the desired path.
 *
 * The algorithm works by:
 * 1. Finding a "lookahead point" on the path ahead of the robot
 * 2. Calculating the curvature needed to reach that point
 * 3. Generating proportional control outputs to minimize error
 *
 * Usage Example:
 * @code
 * // Create odometry system
 * ThreeWheelOdom odom(centerToRight, centerToLeft, centerToBack);
 * odom.initialize({1, 2, 3});
 *
 * // Create Pure Pursuit controller
 * PurePursuit controller(odom, 12.0); // 12 inch lookahead
 *
 * // Set path waypoints
 * Path waypoints = {{0, 0}, {24, 0}, {24, 24}, {0, 24}};
 * controller.setPath(waypoints);
 *
 * // In control loop
 * while (!controller.isFinished()) {
 *     PursuitOutput output = controller.update();
 *     chassis.arcade(output.linear, output.angular);
 *     pros::delay(20);
 * }
 * @endcode
 */
class PurePursuit {
public:
    /**
     * @brief Construct Pure Pursuit controller
     *
     * Creates a new Pure Pursuit controller with the specified odometry system
     * and lookahead distance.
     *
     * @param tracker Reference to odometry system for position feedback
     * @param lookahead Lookahead distance in inches (typically 6-18 inches)
     * @param kTurn Curvature gain multiplier (default: 1.0)
     */
    explicit PurePursuit(AbstractOdom& tracker, double lookahead, double kTurn = 1.0);

    /**
     * @brief Set the path to follow
     *
     * Loads a new path for the controller to follow. The path should be
     * a series of waypoints that define the desired robot trajectory.
     *
     * @param p Vector of waypoints defining the path
     */
    void setPath(const Path& p);

    /**
     * @brief Set the lookahead distance
     *
     * Dynamically adjusts the lookahead distance during operation.
     * Larger values provide smoother but less precise following.
     * Smaller values provide more precise but potentially jerky following.
     *
     * @param l New lookahead distance in inches
     */
    void setLookahead(double l);

    /**
     * @brief Update controller and get motion commands
     *
     * Calculates the current control outputs based on robot position
     * and path following requirements. This should be called regularly
     * in the control loop (typically 50Hz).
     *
     * @return PursuitOutput containing linear/angular velocities and completion status
     */
    PursuitOutput update();

    /**
     * @brief Check if path following is complete
     *
     * @return true if the robot has reached the end of the path
     */
    bool isFinished() const { return finished; }

private:
    AbstractOdom& odom;        /**< Reference to odometry system for position feedback */
    Path path;                 /**< Current path being followed */
    size_t currentIdx{0};      /**< Index of current segment in path */
    double lookaheadDist;      /**< Lookahead distance in inches */
    double kTurn;              /**< Curvature gain multiplier */
    bool finished{false};      /**< Whether path following is complete */

    /**
     * @brief Find the lookahead point on the path
     *
     * Searches the path to find the point that is exactly the lookahead
     * distance ahead of the robot's current position. This point becomes
     * the target for curvature calculations.
     *
     * @param robot Current robot pose (position + heading)
     * @return Lookahead point if found, std::nullopt if at path end
     */
    std::optional<Point> findLookahead(const Pose& robot);
};

} // namespace ls
