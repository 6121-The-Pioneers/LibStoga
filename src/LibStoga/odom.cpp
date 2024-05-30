#include "odom.h"

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
