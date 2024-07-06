#include "geometry.h"
#include <math.h>

void ls::Angle::setAngle(double val)
{
    angle = val;
}

double ls::Angle::getAngle() const
{
    return angle;
}

double ls::Angle::convertToRadians() const
{
    return degreesToRadians(angle);
}

double ls::Angle::normalize() const
{
    double val = angle;
    while (val > 360) val -= 360;
    while (val < 0) val += 360;
    return val;
}

ls::Angle ls::Angle::minimumAngleDifference(Angle &other) const
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

void ls::Angle::operator=(double x)
{
    setAngle(x);
}

void ls::Angle::operator+=(Angle other)
{
    angle += other.angle;
}

double ls::degreesToRadians(double degrees)
{
    return degrees * 0.0174533;
}

double ls::radiansToDegrees(double radians)
{
    return radians * 57.2958;
}
