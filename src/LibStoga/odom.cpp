#include "odom.h"
#include "tracking.h"

double ls::AbstractOdom::getX()
{
	return pos.X;
}

double ls::AbstractOdom::getY()
{
	return pos.Y;
}

double ls::AbstractOdom::getAngle()
{
	return pos.theta.getAngle();
}

void ls::AbstractOdom::resetAll()
{
	resetX();
	resetY();
	resetAngle();
}

double ls::Position::distanceFromPoint(Position &pos) const
{
	const double dx = pos.X - X;
	const double dy = pos.Y - Y;
	return sqrt(dx*dx + dy*dy);
}

double ls::Position::distanceFromPointSigned(Position &pos) const
{
	return isBehind(pos) * distanceFromPoint(pos);
}

int ls::Position::isBehind(Position &pos) const
{
	Angle a = angleToPosition(pos);
	const Angle lower_bound = theta.getAngle() - 90;
	const Angle upper_bound = theta.getAngle() + 90;

	if (a.getAngle() == infinity()) {
		return 0;
	}
	else if (a.getAngle() > lower_bound.normalize() && a.getAngle() < upper_bound.normalize()) {
		return 1;
	} 
	else {
		return -1;
	}
}

ls::Angle ls::Position::angleToPosition(Position &pos) const
{
    Angle tor = (90 - atan2(pos.Y-Y, pos.X-X) * 57.2957795131);
	tor = tor.normalize();
	return tor;
}

ls::Angle ls::Position::angleToPositionSigned(Position &pos) const
{
	return (90 - atan2(pos.Y-Y, pos.X-X) * 57.2957795131);
}
