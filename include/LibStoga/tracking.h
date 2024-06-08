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
        double radius;
        double conversion_factor;
    public:
        /*
        * Constructs a tracking wheel object.
        * Can be reversed if necessary.
        * 
        * @param port the port rotation sensor is on
        * @param reverse if motor needs to be reversed.
        * @param tracking wheel radius in inches
        */
        TrackingWheel(int8_t port, double wheel_radius=2.75, bool reverse=false);

        /*
        * Reverses the wheel direction.
        */
        void reverse();

        /*
        * Gets the linear speed of this TrackingWheel.
        * Depending on rotary encoder it will output a linear speed in in/s
        * 
        * @returns Linear speed of this encoder.
        */
        double getLinearSpeed();

        /*
        * Gets the displacement of this TrackingWheel.
        * Will change depending on radius. 
        * 
        * @returns Displacement in '(in) 
        */
        double getLinearDistance();

        /*
        * Set the radius of this TrackingWheel object.
        * 
        * @param wheel_radius new radius in '(in)
        */
        void setRadius(double wheel_radius);

        /*
        * Destroyes this TrackingWheel object.
        */
        ~TrackingWheel();
    };
}

#endif // TRACKING_H