/** @file tracking.h
 * @brief Tracking wheel sensor management for odometry systems
 *
 * This file provides the TrackingWheel class that wraps PROS rotation sensors
 * and ADI encoders for use in odometry calculations. Tracking wheels are essential
 * components of dead reckoning systems, measuring wheel rotation to calculate
 * robot displacement and velocity.
 *
 * Key Features:
 * - Support for both PROS Rotation sensors and ADI encoders
 * - Automatic linear distance and velocity calculations
 * - Configurable wheel radius and reversal
 * - Delta distance tracking for incremental measurements
 * - Thread-safe sensor access with proper error handling
 *
 * Sensor Types:
 * - **PROS Rotation**: Smart port sensors (ports 1-24) with high resolution
 * - **ADI Encoder**: Analog ports (A-H) for legacy encoder support
 *
 * Coordinate System:
 * - All distances returned in inches
 * - All velocities returned in inches per second
 * - Positive direction based on sensor mounting and reversal setting
 *
 * @author 6121D (The Pioneers)
 * @version 3.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup sensors
 */
#ifndef TRACKING_H
#define TRACKING_H

#include "api.h"
#include <cstdint>

namespace ls {
    /**
     * @brief Tracking wheel sensor wrapper for odometry calculations
     *
     * Provides a unified interface for tracking wheel sensors used in odometry systems.
     * This class handles both PROS Rotation sensors and ADI encoders, automatically
     * converting rotary motion to linear displacement and velocity measurements.
     *
     * The class supports two types of encoders:
     * 1. **PROS Rotation sensors** (smart ports 1-24): Higher resolution, easier setup
     * 2. **ADI encoders** (analog ports A-H): Legacy support for older encoder setups
     *
     * Usage Examples:
     * @code
     * // PROS Rotation sensor on smart port 1
     * TrackingWheel wheel1(1, 2.75); // 2.75" radius wheel
     *
     * // ADI encoder on ports A and B
     * TrackingWheel wheel2('A', 'B', 1.375); // 1.375" radius wheel
     *
     * // Get measurements
     * double distance = wheel1.getLinearDistance();
     * double speed = wheel1.getLinearSpeed();
     * double delta = wheel1.getLinearDeltaDistance();
     * @endcode
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
         * @param port_upper the upper port (only A-H allowed)
         * @param port_lower the lower port (only A-H allowed)
         * @param radius radius of the wheel attached
         * @param reversed if is reversed by default
         */
        explicit TrackingWheel(std::uint8_t port_upper, std::uint8_t port_lower, double radius=1.375, bool reversed=false);

        TrackingWheel(const TrackingWheel&) = delete;
        TrackingWheel& operator=(const TrackingWheel&) = delete;
        TrackingWheel(TrackingWheel&&) = default;
        TrackingWheel& operator=(TrackingWheel&&) = default;

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

        /**
         * @brief reset this tracking wheel.
         * 
         */
        virtual void reset();
    private:
        double prev_distance = 0;
        long double prev_time = 0;
    };
}

#endif // TRACKING_H