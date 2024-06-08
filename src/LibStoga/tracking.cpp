#include "tracking.h"

ls::TrackingWheel::TrackingWheel(int8_t port, double wr, bool reverse)
{
    wheel = new pros::Rotation(port);
    if (reverse) wheel->reverse();
    wheel->set_data_rate(5);
    radius = wr;
    conversion_factor = radius / 360000.0;
}

void ls::TrackingWheel::reverse()
{
    wheel->reverse();
}

double ls::TrackingWheel::getLinearSpeed()
{
    return wheel->get_velocity() * conversion_factor;
}

double ls::TrackingWheel::getLinearDistance()
{
    return wheel->get_position() * conversion_factor;
}

void ls::TrackingWheel::setRadius(double wr)
{
    radius = wr;
    conversion_factor = radius / 360000.0;
}

ls::TrackingWheel::~TrackingWheel()
{
    delete wheel;
}
