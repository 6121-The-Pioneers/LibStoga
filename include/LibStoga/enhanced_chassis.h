#pragma once
#include <cstdint>

namespace ls {

/**
 * @brief Movement parameter structures inspired by LemLib for enhanced control
 */
enum class AngularDirection {
    AUTO,           // Automatically choose shortest direction
    CW_CLOCKWISE,   // Force clockwise rotation
    CCW_COUNTERCLOCKWISE  // Force counter-clockwise rotation
};

struct MoveToPointParams {
    bool forwards = true;           // Move forwards or backwards
    double maxSpeed = 127.0;        // Maximum speed (0-127)
    double minSpeed = 0.0;          // Minimum speed (0-127)
    double earlyExitRange = 0.0;    // Exit early when within this distance (inches)
    double lead = 0.6;              // Carrot point multiplier for path following (0-1)
    double horizontalDrift = 2.0;   // Cornering aggressiveness (higher = faster turns)
};

struct TurnToHeadingParams {
    AngularDirection direction = AngularDirection::AUTO;  // Turn direction
    double maxSpeed = 127.0;        // Maximum angular speed (0-127)
    double minSpeed = 0.0;          // Minimum angular speed (0-127)
    double earlyExitRange = 0.0;    // Exit early when within this angle (degrees)
};

struct MoveToPoseParams {
    bool forwards = true;           // Move forwards or backwards
    double maxSpeed = 127.0;        // Maximum speed (0-127)
    double minSpeed = 0.0;          // Minimum speed (0-127)
    double earlyExitRange = 0.0;    // Exit early when within this distance (inches)
    double lead = 0.6;              // Carrot point multiplier (0-1)
    double horizontalDrift = 2.0;   // Cornering aggressiveness
};

struct SwingToHeadingParams {
    AngularDirection direction = AngularDirection::AUTO;  // Turn direction
    double maxSpeed = 127.0;        // Maximum angular speed (0-127)
    double minSpeed = 0.0;          // Minimum angular speed (0-127)
    double earlyExitRange = 0.0;    // Exit early when within this angle (degrees)
};

struct SwingToPointParams {
    bool forwards = true;           // Face point with front or back
    AngularDirection direction = AngularDirection::AUTO;  // Turn direction
    double maxSpeed = 127.0;        // Maximum angular speed (0-127)
    double minSpeed = 0.0;          // Minimum angular speed (0-127)
    double earlyExitRange = 0.0;    // Exit early when within this angle (degrees)
};

struct FollowPathParams {
    double maxSpeed = 127.0;        // Maximum speed (0-127)
    double minSpeed = 0.0;          // Minimum speed (0-127)
    bool forwards = true;           // Follow path forwards or backwards
    double lead = 0.6;              // Carrot point multiplier (0-1)
    bool async = true;              // Run asynchronously
};

/**
 * @brief Enhanced chassis with LemLib-style movement parameters
 */
class EnhancedChassis {
public:
    /**
     * @brief Move to a point with advanced parameters
     */
    virtual void moveToPoint(double x, double y, int timeout, const MoveToPointParams& params = {}) = 0;

    /**
     * @brief Turn to a heading with advanced parameters
     */
    virtual void turnToHeading(double heading, int timeout, const TurnToHeadingParams& params = {}) = 0;

    /**
     * @brief Move to a pose (position + heading) with advanced parameters
     */
    virtual void moveToPose(double x, double y, double heading, int timeout, const MoveToPoseParams& params = {}) = 0;

    /**
     * @brief Swing turn to heading (one side locked) with advanced parameters
     */
    virtual void swingToHeading(double heading, bool leftSideLocked, int timeout, const SwingToHeadingParams& params = {}) = 0;

    /**
     * @brief Swing turn to point (one side locked) with advanced parameters
     */
    virtual void swingToPoint(double x, double y, bool leftSideLocked, int timeout, const SwingToPointParams& params = {}) = 0;

    /**
     * @brief Follow path with advanced parameters
     */
    virtual void followPath(const std::vector<Point>& path, double lookahead, int timeout, const FollowPathParams& params = {}) = 0;

    /**
     * @brief Check if chassis is currently in motion
     */
    virtual bool isInMotion() const = 0;

    /**
     * @brief Wait until robot has traveled a specific distance
     */
    virtual void waitUntil(double distance) = 0;

    /**
     * @brief Wait until current motion is complete
     */
    virtual void waitUntilDone() = 0;

    /**
     * @brief Cancel current motion
     */
    virtual void cancelMotion() = 0;

    /**
     * @brief Cancel all motions (including queued)
     */
    virtual void cancelAllMotions() = 0;

    /**
     * @brief Get current pose
     */
    virtual Pose getPose() const = 0;

    /**
     * @brief Set current pose
     */
    virtual void setPose(const Pose& pose) = 0;

    /**
     * @brief Reset local position (keep heading)
     */
    virtual void resetLocalPosition() = 0;

    /**
     * @brief Set brake mode for drivetrain
     */
    virtual void setBrakeMode(pros::motor_brake_mode_e mode) = 0;
};

} // namespace ls