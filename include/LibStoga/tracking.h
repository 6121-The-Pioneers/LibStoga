/*
* This file contains all classes for Tracking movements
* Used extensively in odom.h
*/
#ifndef TRACKING_H
#define TRACKING_H

#include "api.h"

namespace ls {
    class TrackingWheel {
    private:
        pros::Rotation* wheel;
    public:
        TrackingWheel(int8_t);

        ~TrackingWheel();
    };
}

#endif // TRACKING_H