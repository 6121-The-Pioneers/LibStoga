/** @file odom.h
 * @brief Odometry system implementations for LibStoga robotics framework
 *
 * This file provides comprehensive odometry (position tracking) functionality
 * essential for autonomous robot navigation. It includes multiple odometry
 * implementations supporting different sensor configurations commonly used
 * in VEX robotics.
 *
 * Supported Odometry Types:
 * - **Three Wheel Odometry**: Uses three tracking wheels (left, right, back)
 * - **IMU Odometry**: Uses two tracking wheels + IMU for enhanced accuracy
 *
 * Key Features:
 * - Abstract base class for extensible odometry implementations
 * - Position tracking with X, Y coordinates and heading angle
 * - Delta calculations for incremental position updates
 * - Bearing-based angle system (0° = North)
 * - Comprehensive error handling and validation
 * - Support for sensor reversal and calibration
 *
 * Coordinate System:
 * - Origin (0, 0) is robot's starting position
 * - Angles use bearing system: 0° = North, 90° = East, etc.
 * - All distances in inches
 * - All angles in degrees
 *
 * @author 6121D (The Pioneers)
 * @version 3.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup core
 */

#ifndef ODOM_ABSTRACT_LS_H
#define ODOM_ABSTRACT_LS_H

#include <initializer_list>
#include <exception>
#include <vector>
#include "tracking.h"
#include "geometry.h"
#include "api.h"
#include <cmath>

namespace ls {

/**
 * @brief Complete robot pose representation (position + orientation)
 *
 * Represents the full state of a robot in 2D space including X, Y coordinates
 * and heading angle. This structure is fundamental to odometry and navigation
 * systems throughout the framework.
 *
 * The coordinate system uses:
 * - **Origin**: Robot's starting position (0, 0)
 * - **Angles**: Bearing system where 0° = North (forward direction)
 * - **Units**: All distances in inches, angles in degrees
 */
struct Position {
    double X;        /**< X coordinate in inches from origin */
    double Y;        /**< Y coordinate in inches from origin */
    Angle theta;     /**< Heading angle in degrees (bearing: 0° = North) */

    /**
     * @brief Default constructor - initializes to origin with 0° heading
     */
    Position(): X(0), Y(0), theta(Angle(0)) {}

    /**
     * @brief Construct position with specified coordinates and heading
     *
     * @param x X coordinate in inches
     * @param y Y coordinate in inches
     * @param t Heading angle in degrees (bearing system)
     */
    Position(double x, double y, double t): X(x), Y(y), theta(t) {}

    /**
     * @brief Calculate Euclidean distance to another position
     *
     * Computes the straight-line distance between this position and another.
     * Always returns a positive value.
     *
     * @param pos Target position to measure distance to
     * @return Distance in inches (always positive)
     */
    double distanceFromPoint(const Position& pos) const;

    /**
     * @brief Calculate signed distance to another position
     *
     * Returns signed distance based on robot's current heading. Positive values
     * indicate points in front of the robot, negative values indicate points behind.
     *
     * @param pos Target position to measure distance to
     * @return Signed distance in inches (positive = forward, negative = backward)
     */
    double distanceFromPointSigned(const Position& pos) const;

    /**
     * @brief Determine relative position (front/behind robot)
     *
     * Analyzes whether a point is in front of, behind, or neither relative
     * to the robot's current heading.
     *
     * @param pos Position to analyze
     * @return -1 if behind, 1 if in front, 0 if indeterminate (too close)
     */
    int isBehind(const Position& pos) const;

    /**
     * @brief Calculate bearing angle to target position
     *
     * Computes the bearing angle from current position to target position.
     * Returns angle in bearing system (0° = North).
     *
     * @param pos Target position
     * @return Bearing angle in degrees [0, 360), or infinity if positions are too close
     */
    Angle angleToPosition(const Position& pos) const;

    /**
     * @brief Stream output operator for debugging
     *
     * Formats position as "Pos(X, Y, theta)" for console output.
     *
     * @param os Output stream
     * @param pos Position to output
     * @return Modified output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const Position& pos) {
        os << "Pos(" << pos.X << ", " << pos.Y << ", " << pos.theta.getAngle() << ")";
        return os;
    }
};

/**
 * @brief Abstract base class for all odometry implementations
 *
 * Defines the common interface for all odometry systems in the framework.
 * Concrete implementations must provide position tracking capabilities
 * using different sensor configurations (wheels, IMU, etc.).
 *
 * This abstraction allows users to switch between different odometry
 * implementations without changing their application code.
 */
class AbstractOdom {
protected:
    Position pos;      /**< Current robot position and heading */
    Position prev_pos; /**< Previous position for delta calculations */

    /**
     * @brief Protected constructor for derived classes
     */
    explicit AbstractOdom(): pos(Position()) {};

public:
    /**
     * @brief Initialize odometry system with sensor ports
     *
     * Sets up the odometry system with the specified sensor ports. The exact
     * meaning and order of ports depends on the concrete implementation:
     * - ThreeWheelOdom: right, left, back tracking wheel ports
     * - ImuOdom: horizontal, vertical tracking wheel ports, IMU port
     *
     * Negative port numbers indicate reversed sensor mounting.
     *
     * @param ports List of sensor ports to initialize
     */
    virtual void initialize(std::initializer_list<uint8_t> ports) = 0;

    /**
     * @brief Update robot position based on sensor readings
     *
     * Computes new position and heading based on sensor data since last update.
     * This method should be called regularly (typically in a control loop)
     * to maintain accurate position tracking.
     *
     * @throws std::bad_function_call if not properly initialized
     */
    virtual void compute();

    /**
     * @brief Get current X coordinate
     *
     * @return X coordinate in inches from origin
     * @throws std::bad_function_call if not properly initialized
     */
    virtual double getX();

    /**
     * @brief Get current Y coordinate
     *
     * @return Y coordinate in inches from origin
     * @throws std::bad_function_call if not properly initialized
     */
    virtual double getY();

    /**
     * @brief Get current heading angle
     *
     * @return Heading angle in degrees (bearing system: 0° = North)
     * @throws std::bad_function_call if not properly initialized
     */
    virtual double getAngle();

    /**
     * @brief Get complete current position
     *
     * @return Position structure with X, Y, and heading
     */
    virtual Position getPosition();

    /**
     * @brief Reset X coordinate to zero
     *
     * Sets current X position to 0, maintaining Y and heading.
     */
    virtual void resetX();

    /**
     * @brief Reset Y coordinate to zero
     *
     * Sets current Y position to 0, maintaining X and heading.
     */
    virtual void resetY();

    /**
     * @brief Reset heading angle to zero
     *
     * Sets current heading to 0°, maintaining X and Y position.
     */
    virtual void resetAngle();

    /**
     * @brief Reset all position and heading to zero
     *
     * Resets robot to origin (0, 0) with 0° heading.
     */
    virtual void resetAll();

    /**
     * @brief Get X position change since last compute() call
     *
     * Returns the incremental change in X position without resetting.
     * Returns 0 on first call.
     *
     * @return Delta X in inches
     * @throws std::bad_function_call if not properly initialized
     */
    virtual double getDeltaX() = 0;

    /**
     * @brief Get Y position change since last compute() call
     *
     * Returns the incremental change in Y position without resetting.
     * Returns 0 on first call.
     *
     * @return Delta Y in inches
     * @throws std::bad_function_call if not properly initialized
     */
    virtual double getDeltaY() = 0;

    /**
     * @brief Get heading change since last compute() call
     *
     * Returns the incremental change in heading without resetting.
     * Returns 0 on first call.
     *
     * @return Delta angle in degrees
     * @throws std::bad_function_call if not properly initialized
     */
    virtual Angle getDeltaAngle() = 0;

    /**
     * @brief Virtual destructor for proper cleanup
     */
    virtual ~AbstractOdom() = default;
};

/**
 * @brief Configuration parameters for tracking wheel sensors
 *
 * Defines the complete setup for a single tracking wheel including
 * sensor port, wheel radius, and mounting orientation.
 */
struct tracking_rotation_parameters_t {
    int16_t port;      /**< PROS port number (1-21, negative for reversed) */
    double radius = 2.75; /**< Wheel radius in inches (default: 2.75") */
    bool reversed = false; /**< Whether sensor is mounted in reverse */
};

/**
 * @brief Configuration parameters for IMU-based odometry
 *
 * Complete setup for 2-wheel + IMU odometry system including
 * tracking wheel configurations and IMU sensor.
 */
struct imu_odom_parameters_t {
    tracking_rotation_parameters_t parallel;     /**< Parallel tracking wheel (left/right) */
    double center_to_parallel;                   /**< Distance from center to parallel wheel */
    tracking_rotation_parameters_t perpendicular; /**< Perpendicular tracking wheel (front/back) */
    double center_to_perpendicular;              /**< Distance from center to perpendicular wheel */
    int16_t imu_port;                           /**< PROS port for IMU sensor */
};

/**
 * @brief Configuration parameters for three-wheel odometry
 *
 * Complete setup for traditional three-wheel odometry system
 * with left, right, and back tracking wheels.
 */
struct threewheel_odom_parameters_t {
    tracking_rotation_parameters_t right;  /**< Right tracking wheel configuration */
    double center_to_right;                /**< Distance from center to right wheel */
    tracking_rotation_parameters_t left;   /**< Left tracking wheel configuration */
    double center_to_left;                 /**< Distance from center to left wheel */
    tracking_rotation_parameters_t back;   /**< Back tracking wheel configuration */
    double center_to_back;                 /**< Distance from center to back wheel */
};

/**
 * @brief Three-wheel odometry implementation
 *
 * Implements odometry using three tracking wheels positioned at different
 * locations on the robot. This is the traditional odometry approach that
 * doesn't require an IMU but provides good position accuracy.
 *
 * Wheel Configuration:
 * - **Right wheel**: Tracks lateral movement
 * - **Left wheel**: Tracks lateral movement (redundant with right)
 * - **Back wheel**: Tracks forward/backward movement
 *
 * The system uses trigonometry to calculate robot movement based on
 * the relative motion of each wheel.
 */
class ThreeWheelOdom: public AbstractOdom {
private:
    std::unique_ptr<TrackingWheel> right;   /**< Right tracking wheel sensor */
    std::unique_ptr<TrackingWheel> left;    /**< Left tracking wheel sensor */
    std::unique_ptr<TrackingWheel> back;    /**< Back tracking wheel sensor */

    double centerToRight;   /**< Distance from robot center to right wheel */
    double centerToLeft;    /**< Distance from robot center to left wheel */
    double centerToBack;    /**< Distance from robot center to back wheel */

    double deltaL = 0;      /**< Left wheel movement delta */
    double deltaR = 0;      /**< Right wheel movement delta */
    double deltaB = 0;      /**< Back wheel movement delta */
    double deltaT = 0;      /**< Heading angle delta */

public:
    /**
     * @brief Construct with wheel distances only
     *
     * Creates odometry system with specified wheel distances. Sensors
     * must be initialized separately using initialize() or by providing
     * TrackingWheel objects.
     *
     * @param center_to_right Distance to right wheel in inches
     * @param center_to_left Distance to left wheel in inches
     * @param center_to_back Distance to back wheel in inches
     */
    explicit ThreeWheelOdom(double center_to_right, double center_to_left, double center_to_back);

    /**
     * @brief Construct with wheel distances and sensor objects
     *
     * Creates fully configured odometry system with pre-configured sensors.
     *
     * @param center_to_right Distance to right wheel in inches
     * @param center_to_left Distance to left wheel in inches
     * @param center_to_back Distance to back wheel in inches
     * @param right Right tracking wheel sensor
     * @param left Left tracking wheel sensor
     * @param back Back tracking wheel sensor
     */
    explicit ThreeWheelOdom(double center_to_right, double center_to_left, double center_to_back,
                           TrackingWheel&& right, TrackingWheel&& left, TrackingWheel&& back);

    /**
     * @brief Construct from configuration structure
     *
     * Creates odometry system using a complete configuration structure.
     *
     * @param param Three-wheel odometry configuration
     */
    explicit ThreeWheelOdom(threewheel_odom_parameters_t& param);

    /**
     * @brief Initialize sensors on specified ports
     *
     * Sets up tracking wheel sensors on the specified PROS ports.
     * Expected order: right, left, back.
     *
     * @param ports List containing exactly 3 port numbers
     * @throws std::invalid_argument if incorrect number of ports provided
     */
    void initialize(std::initializer_list<uint8_t> ports) override;

    /**
     * @brief Get X position change since last compute()
     *
     * @return Delta X in inches
     * @throws std::bad_function_call if not initialized
     */
    double getDeltaX() override;

    /**
     * @brief Get Y position change since last compute()
     *
     * @return Delta Y in inches
     * @throws std::bad_function_call if not initialized
     */
    double getDeltaY() override;

    /**
     * @brief Get heading change since last compute()
     *
     * @return Delta angle in degrees
     * @throws std::bad_function_call if not initialized
     */
    Angle getDeltaAngle() override;

    /**
     * @brief Update position based on sensor readings
     *
     * Computes new robot position using three-wheel odometry calculations.
     */
    void compute() override;
};

/**
 * @brief IMU-enhanced two-wheel odometry implementation
 *
 * Implements odometry using two tracking wheels plus an IMU sensor for
 * enhanced accuracy and reliability. The IMU provides direct heading
 * measurements while tracking wheels handle position calculations.
 *
 * Sensor Configuration:
 * - **Horizontal wheel**: Tracks lateral movement (left/right)
 * - **Vertical wheel**: Tracks forward/backward movement
 * - **IMU**: Provides direct heading angle measurements
 *
 * This system is more accurate than three-wheel odometry, especially
 * during high-speed maneuvers or when wheel slippage occurs.
 */
class ImuOdom: public AbstractOdom {
private:
    std::unique_ptr<TrackingWheel> horiz;  /**< Horizontal tracking wheel (left/right movement) */
    std::unique_ptr<TrackingWheel> vert;   /**< Vertical tracking wheel (forward/backward movement) */
    std::unique_ptr<pros::Imu> IMU;        /**< Inertial Measurement Unit for heading */

    double centerToVert;     /**< Distance from center to vertical wheel */
    double centerToHoriz;    /**< Distance from center to horizontal wheel */
    double prevRotation;     /**< Previous IMU rotation reading */

    double deltaGlobalX = 0; /**< Global X movement delta */
    double deltaGlobalY = 0; /**< Global Y movement delta */

public:
    /**
     * @brief Construct with wheel distances only
     *
     * Creates odometry system with specified wheel distances. Sensors
     * must be initialized separately.
     *
     * @param center_to_horiz Distance to horizontal wheel in inches
     * @param center_to_vert Distance to vertical wheel in inches
     */
    explicit ImuOdom(double center_to_horiz, double center_to_vert);

    /**
     * @brief Construct with wheel distances and sensor objects
     *
     * Creates fully configured odometry system with pre-configured sensors.
     *
     * @param center_to_horiz Distance to horizontal wheel in inches
     * @param center_to_vert Distance to vertical wheel in inches
     * @param horiz Horizontal tracking wheel sensor
     * @param vert Vertical tracking wheel sensor
     * @param IMU Inertial measurement unit sensor
     */
    explicit ImuOdom(double center_to_horiz, double center_to_vert,
                    TrackingWheel&& horiz, TrackingWheel&& vert, pros::Imu& IMU);

    /**
     * @brief Construct from configuration structure
     *
     * Creates odometry system using a complete configuration structure.
     *
     * @param param IMU odometry configuration
     */
    explicit ImuOdom(imu_odom_parameters_t& param);

    /**
     * @brief Initialize sensors on specified ports
     *
     * Sets up sensors on the specified PROS ports.
     * Expected order: horizontal wheel, vertical wheel, IMU.
     *
     * @param ports List containing exactly 3 port numbers
     * @throws std::invalid_argument if incorrect number of ports provided
     */
    void initialize(std::initializer_list<uint8_t> ports) override;

    /**
     * @brief Get X position change since last compute()
     *
     * @return Delta X in inches
     * @throws std::bad_function_call if not initialized
     */
    double getDeltaX() override;

    /**
     * @brief Get Y position change since last compute()
     *
     * @return Delta Y in inches
     * @throws std::bad_function_call if not initialized
     */
    double getDeltaY() override;

    /**
     * @brief Get heading change since last compute()
     *
     * @return Delta angle in degrees
     * @throws std::bad_function_call if not initialized
     */
    Angle getDeltaAngle() override;

    /**
     * @brief Update position using IMU and tracking wheels
     *
     * Computes new robot position using IMU heading and wheel tracking data.
     */
    void compute() override;

    /**
     * @brief Reset all position and heading to zero
     *
     * Resets position to origin and calibrates IMU to 0° heading.
     */
    void resetAll() override;
};

} // namespace ls

#endif // !ODOM_ABSTRACT_LS_H
