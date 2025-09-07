/** @file geometry.h
 * @brief Core geometry utilities for LibStoga robotics framework
 *
 * This file provides fundamental geometric primitives and utilities essential
 * for robotics applications. It includes angle manipulation, point representations,
 * pose management, and coordinate system transformations.
 *
 * The geometry system uses a **bearing-based coordinate system** where:
 * - 0° = North (positive Y direction)
 * - 90° = East (positive X direction)
 * - 180° = South (negative Y direction)
 * - 270° = West (negative X direction)
 *
 * @author 6121D (The Pioneers)
 * @version 3.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup core
 */

#ifndef GEOMETRY_LS_H
#define GEOMETRY_LS_H

#include <vector>
#include <ostream>
#include <cmath>

namespace ls {

/**
 * @brief A class to represent and manipulate angles in robotics applications.
 *
 * The Angle class provides robust angle manipulation with automatic normalization
 * and conversion between degrees and radians. All angles are stored internally
 * in degrees using the bearing system (0° = North).
 *
 * @note Angles are automatically normalized to [0, 360) degree range
 * @warning Input angles should be in degrees unless specified otherwise
 */
class Angle {
private:
    double angle; /**< Internal angle storage in degrees */

public:
    /**
     * @brief Default constructor - initializes to 0°
     */
    Angle(): angle(0) {}

    /**
     * @brief Construct angle from degrees
     * @param a Angle in degrees (automatically normalized)
     */
    Angle(double a): angle(a) {}

    /**
     * @brief Set the angle value
     * @param val New angle in degrees
     */
    void setAngle(double val);

    /**
     * @brief Get the current angle value
     * @return Angle in degrees [0, 360)
     */
    double getAngle() const;

    /**
     * @brief Convert angle to radians
     * @return Angle in radians [-π, π]
     */
    double convertToRadians() const;

    /**
     * @brief Normalize angle to [0, 360) degree range
     * @return Normalized angle in degrees
     */
    double normalize() const;

    /**
     * @brief Calculate minimum angle difference to another angle
     *
     * Returns the smallest angle difference, considering that angles wrap
     * around at 360°. The result will be in range [-180, 180].
     *
     * @param other The target angle to compare against
     * @return Minimum angle difference in degrees
     */
    Angle minimumAngleDifference(const Angle& other) const;

    /**
     * @brief Assignment operator from double
     * @param x Angle value in degrees
     */
    void operator=(double x);

    /**
     * @brief Addition assignment operator
     * @param other Angle to add
     */
    void operator+=(Angle other);
};

/**
 * @brief 2D point representation in Cartesian coordinates
 *
 * Simple structure representing a point in 2D space with X and Y coordinates.
 * Used throughout the library for waypoints, positions, and geometric calculations.
 */
struct Point {
    double x{0}; /**< X coordinate */
    double y{0}; /**< Y coordinate */

    /**
     * @brief Default constructor - initializes to origin (0, 0)
     */
    Point() = default;

    /**
     * @brief Construct point from coordinates
     * @param _x X coordinate
     * @param _y Y coordinate
     */
    Point(double _x, double _y): x(_x), y(_y) {}
};

/**
 * @brief 2D pose representation with position and orientation
 *
 * Extends Point to include heading information, representing the complete
 * state of a robot or object in 2D space. The heading uses the bearing
 * coordinate system where 0° = North.
 */
struct Pose : public Point {
    Angle heading; /**< Bearing angle in degrees (0° = North) */

    /**
     * @brief Default constructor - initializes to origin with 0° heading
     */
    Pose(): Point(), heading(0) {}

    /**
     * @brief Construct pose from position and heading
     * @param _x X coordinate
     * @param _y Y coordinate
     * @param h Heading angle in degrees
     */
    Pose(double _x, double _y, double h): Point(_x,_y), heading(h) {}
};

/**
 * @brief Type alias for a sequence of waypoints
 *
 * Represents a path as a series of points that can be followed by motion
 * planning algorithms like pure pursuit.
 */
using Path = std::vector<Point>;

/**
 * @brief Convert degrees to radians
 * @param degrees Angle in degrees
 * @return Equivalent angle in radians
 */
double degreesToRadians(double degrees);

/**
 * @brief Convert radians to degrees
 * @param radians Angle in radians
 * @return Equivalent angle in degrees
 */
double radiansToDegrees(double radians);

/**
 * @brief Calculate Euclidean distance between two points
 * @param a First point
 * @param b Second point
 * @return Distance between points (always positive)
 */
double distance(const Point& a, const Point& b);

/**
 * @brief Calculate bearing angle from point A to point B
 *
 * Returns the bearing angle (in degrees) from point A to point B,
 * where 0° = North, 90° = East, etc.
 *
 * @param a Starting point
 * @param b Target point
 * @return Bearing angle in degrees [0, 360)
 */
Angle bearingTo(const Point& a, const Point& b);

/**
 * @brief Calculate the smallest angle difference between two angles
 *
 * Returns the minimum angle difference considering the circular nature
 * of angles. Result is in range [-180, 180].
 *
 * @param a First angle in degrees
 * @param b Second angle in degrees
 * @return Minimum angle difference in degrees
 */
double angleDiff(double a, double b);

} // namespace ls

#endif // GEOMETRY_LS_H