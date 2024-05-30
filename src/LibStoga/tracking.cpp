#include "tracking.h"

ls::TrackingWheel::TrackingWheel(int8_t port)
{
    wheel = new pros::Rotation(port);
}

ls::TrackingWheel::~TrackingWheel()
{
    delete wheel;
}
