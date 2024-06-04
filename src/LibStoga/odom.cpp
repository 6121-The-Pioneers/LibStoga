#include "odom.h"
#include "tracking.h"

double ls::AbstractOdom::getX()
{
	return X;
}

double ls::AbstractOdom::getY()
{
	return Y;
}

double ls::AbstractOdom::getAngle()
{
	return angle;
}


void ls::AbstractOdom::resetAll()
{
	resetX();
	resetY();
	resetAngle();
}

ls::PositionTracker::PositionTracker()
{
	wheels = std::vector<ls::TrackingWheel>();
}

void ls::PositionTracker::addTrackingWheel(TrackingWheel& wheel)
{
	wheels.push_back(wheel);
}

void ls::PositionTracker::reset()
{
	wheels.clear();
}

ls::TrackingWheel& ls::PositionTracker::getWheelByIndex(size_t index)
{
	if (index < 0 || index >= size()) {
		throw std::invalid_argument("Please give a valid index.");
	}
	return wheels[index];
}

ls::TrackingWheel& ls::PositionTracker::operator[](size_t i)
{
	return getWheelByIndex(i);
}

size_t ls::PositionTracker::size()

{
	return wheels.size();
}


