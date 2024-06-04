/*
* This file contains all classes for Tracking movements
* Used extensively in odom.h
*/
#ifndef TRACKING_H
#define TRACKING_H

#include "api.h"

namespace ls {
    /*
    * Wrapper class for pros::Rotation.
    * Used extensively for Odom configurations.
    */
    class TrackingWheel {
    private:
        pros::Rotation* wheel;
    public:
        /*
        * Constructs a tracking wheel object.
        * Can be reversed if necessary.
        * 
        * @param port the port rotation sensor is on
        * @param reverse if motor needs to be reversed.
        */
        TrackingWheel(int8_t port, bool reverse=false);

        /*
        * Reverses the wheel direction.
        */
        void reverse();

        /*
        * TODO... add more methods
        */

        /*
        * Destroyes this TrackingWheel object.
        */
        ~TrackingWheel();
    };
}

#endif // TRACKING_H