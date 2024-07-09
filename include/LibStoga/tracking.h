/*
* This file contains all classes for Tracking movements
* Used extensively in odom.h
*/
#ifndef TRACKING_H
#define TRACKING_H

#include "api.h"
#include <cstdint>

namespace ls {
    /*
    * Wrapper class for pros::Rotation.
    * Used extensively for Odom wheels.
    */
    class TrackingWheel {
    private:
        double radius;
        double conversion_factor;

        std::unique_ptr<pros::adi::Encoder> encoder;
        std::unique_ptr<pros::Rotation> rotation;

        std::int8_t port_upper = 0, port_lower = 0;
        bool reversed = false;
    public:
        /**
         * @brief Construct a new Tracking Wheel object
         * This will construct as a pros::Rotation object.
         * 
         * @param port the smart port [1,24] to connect
         * @param radius radius of the wheel attached
         * @param reversed if is reversed by default
         */
        explicit TrackingWheel(std::uint8_t port, double radius=2.75, bool reversed=false);

        /**
         * @brief Construct a new Tracking Wheel object
         * This will construct as a pros::ADIEncoder object.
         * 
         * @param port_upper the upper port
         * @param port_lower the lower port
         * @param radius radius of the wheel attached
         * @param reversed if is reversed by default
         */
        explicit TrackingWheel(std::uint8_t port_upper, std::uint8_t port_lower, double radius=2.75, bool reversed=false);

        /**
         * @brief Construct a new Tracking Wheel object
         * Takes the configurations from the 'other' tracking wheel and puts them into this.
         * Will erase all configurations from 'other' in the proccess.
         * 
         * @param other the other to pull configurations from.
         */
        explicit TrackingWheel(TrackingWheel& other);

        /**
        * @brief reverses the wheel direction.
        */
        virtual void reverse();

        /**
        * @brief Gets the linear speed of this TrackingWheel.
        * Depending on rotary encoder it will output a linear speed in in/s
        * 
        * @returns Linear speed of this encoder.
        */
        virtual double getLinearSpeed();

        /**
        * @brief Gets the displacement of this TrackingWheel.
        * Will change depending on radius. 
        * 
        * @returns Displacement in '(in) 
        */
        virtual double getLinearDistance();

        /**
        * @brief Gets the change in distance of this tracking wheel since the last call of this function.
        * Will change depending on radius.
        * 
        * @returns change in distance in '(in)
        */
        virtual double getLinearDeltaDistance();

        /**
        * @brief Set the radius of this TrackingWheel object.
        * 
        * @param wheel_radius new radius in '(in)
        */
        virtual void setRadius(double wheel_radius);
    private:
        double prev_distance = 0;
        long double prev_time = 0;
    };
}

#endif // TRACKING_H