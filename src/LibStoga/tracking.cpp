#include "tracking.h"

ls::TrackingWheel::TrackingWheel(int8_t port, bool reverse)
{
    wheel = new pros::Rotation(port);
    if (reverse) wheel->reverse();
    wheel->set_data_rate(5);
}

ls::TrackingWheel::~TrackingWheel()
{
    delete wheel;
}
