#include "geometry.h"
#include <cmath>

namespace ls {

void Angle::setAngle(double val)
{
    angle = val;
}

double Angle::getAngle() const
{
    return angle;
}

double Angle::convertToRadians() const
{
    return degreesToRadians(angle);
}

double Angle::normalize() const
{
    double val = fmod(angle, 360.0);
    if (val < 0) val += 360.0;
    return val;
}

Angle Angle::minimumAngleDifference(const Angle &other) const
{
    double diff = angle - other.angle;
    while (diff > 180) {
        diff -= 360;
    }
    while (diff <= -180) {
        diff += 360;
    }
    return diff;
}

void Angle::operator=(double x)
{
    setAngle(x);
}

void Angle::operator+=(Angle other)
{
    angle += other.angle;
}

double degreesToRadians(double degrees)
{
    return degrees * 0.0174533;
}

double radiansToDegrees(double radians)
{
    return radians * 57.2958;
}

double distance(const Point &a, const Point &b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return std::sqrt(dx*dx + dy*dy);
}

Angle bearingTo(const Point &a, const Point &b) {
    double raw = std::atan2(b.y - a.y, b.x - a.x); // radians (-pi,pi]
    double deg = 90.0 - radiansToDegrees(raw);
    while (deg >= 360) deg -= 360;
    while (deg < 0) deg += 360;
    return Angle(deg);
}

double angleDiff(double a, double b) {
    double diff = a - b;
    while (diff > 180) diff -= 360;
    while (diff <= -180) diff += 360;
    return diff;
}

} // namespace ls
