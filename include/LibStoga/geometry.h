/** ✅
 * @file geometry.h
 * @author Rishit Varshney
 * @brief Contains all code for geometrical constructions (like Angles) and other such helpful stuff
 * @version 0.1
 * @date 2024-11-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef GEOMETRY_LS_H
#define GEOMETRY_LS_H

#include <vector>
#include <ostream>

namespace ls {
    /**
     * @brief A class to represent and manipulate angles.
     */
    class Angle {
    private:
        double angle;
    public:
        Angle(): angle(0) {}
        Angle(double a): angle(a) {}
        void setAngle(double val);
        double getAngle() const;
        double convertToRadians() const;
        double normalize() const;
        Angle minimumAngleDifference(const Angle& other) const; 
        void operator=(double x);
        void operator+=(Angle other);
    };

    struct Point {
        double x{0};
        double y{0};
        Point() = default;
        Point(double _x, double _y): x(_x), y(_y) {}
    };

    struct Pose : public Point {
        Angle heading; // bearing style (0 = field north)
        Pose(): Point(), heading(0) {}
        Pose(double _x, double _y, double h): Point(_x,_y), heading(h) {}
    };

    using Path = std::vector<Point>; // simple waypoint path

    double degreesToRadians(double degrees);
    double radiansToDegrees(double radians);
    double distance(const Point& a, const Point& b);
    Angle bearingTo(const Point& a, const Point& b);
    double angleDiff(double a, double b);
}

#endif // GEOMETRY_LS_H