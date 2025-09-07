/** @file mcl.h
 * @brief Monte Carlo Localization (MCL) for advanced robot positioning
 *
 * This file implements Monte Carlo Localization, also known as particle filtering,
 * for robust robot localization in dynamic environments. MCL uses a population
 * of particles to represent the probability distribution of the robot's pose,
 * providing better accuracy than traditional odometry in the presence of
 * wheel slippage, uneven terrain, or kidnapped robot scenarios.
 *
 * Key Features:
 * - Particle filter-based localization
 * - Robust to odometry drift and wheel slippage
 * - Handles kidnapped robot problem
 * - Configurable particle count and noise parameters
 * - Measurement injection for absolute position updates
 * - Probabilistic pose estimation
 *
 * How MCL Works:
 * 1. **Initialization**: Create particles representing possible robot poses
 * 2. **Motion Update**: Predict particle movement based on odometry
 * 3. **Measurement Update**: Adjust particle weights based on sensor measurements
 * 4. **Resampling**: Create new particle set based on weights
 * 5. **Pose Estimation**: Calculate robot position from particle distribution
 *
 * @author 6121D (The Pioneers)
 * @version 3.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup localization
 */

#pragma once
#include <vector>
#include <random>
#include "geometry.h"
#include "odom.h"

namespace ls {

    /**
     * @brief Individual particle representing a possible robot pose
     *
     * Each particle represents a hypothesis about the robot's current position
     * and orientation. The weight indicates how likely this hypothesis is
     * based on sensor measurements and motion predictions.
     */
    struct Particle {
        double x;        /**< X coordinate of particle position */
        double y;        /**< Y coordinate of particle position */
        double heading;  /**< Heading angle in degrees (bearing system) */
        double weight;   /**< Importance weight (probability) of this particle */
    };

    /**
     * @brief Monte Carlo Localization tracker using particle filtering
     *
     * Implements advanced robot localization using a particle filter approach.
     * This system maintains multiple hypotheses about the robot's position
     * and uses sensor measurements to determine the most likely pose.
     *
     * The MCL system is particularly effective in environments where:
     * - Wheel odometry is unreliable (slippery surfaces, uneven terrain)
     * - The robot may be moved by external forces (kidnapped robot)
     * - Absolute position measurements are available (vision, GPS, etc.)
     *
     * Usage Example:
     * @code
     * // Configure MCL with 300 particles for 12x12ft field
     * MCLTracker::Config config{
     *     .count = 300,
     *     .processNoiseXY = 0.3,
     *     .processNoiseTheta = 1.5,
     *     .fieldW = 144.0,
     *     .fieldH = 144.0
     * };
     *
     * MCLTracker tracker(config);
     *
     * // In main loop
     * tracker.compute(); // Update particle positions
     *
     * // Inject absolute measurements when available
     * tracker.injectMeasurement(24.0, 36.0, 45.0, 1.0, 3.0);
     *
     * // Get current position estimate
     * double x = tracker.getX();
     * double y = tracker.getY();
     * @endcode
     */
    class MCLTracker : public AbstractOdom {
    public:
        /**
         * @brief Configuration parameters for MCL system
         *
         * Defines all parameters needed to configure the Monte Carlo Localization system,
         * including particle count, noise characteristics, and field dimensions.
         */
        struct Config {
            size_t count = 200;          /**< Number of particles (more = better accuracy, slower) */
            double processNoiseXY = 0.5; /**< Process noise for X/Y position (inches) */
            double processNoiseTheta = 2.0; /**< Process noise for heading (degrees) */
            double fieldW = 144.0;       /**< Field width for particle initialization (inches) */
            double fieldH = 144.0;       /**< Field height for particle initialization (inches) */
        };

        /**
         * @brief Construct MCL tracker with configuration
         *
         * Creates a Monte Carlo Localization system with the specified configuration.
         * The system will initialize particles randomly across the field.
         *
         * @param cfg Configuration parameters for the MCL system
         */
        explicit MCLTracker(const Config& cfg);

        /**
         * @brief Initialize the MCL system (not used directly)
         *
         * This method is required by the AbstractOdom interface but is not
         * used in MCL since initialization is handled in the constructor.
         *
         * @param ports Not used in MCL implementation
         */
        void initialize(std::initializer_list<uint8_t>) override {}

        /**
         * @brief Update particle positions and compute robot pose
         *
         * Performs the complete MCL update cycle:
         * 1. Motion update based on odometry
         * 2. Weight normalization
         * 3. Resampling based on particle weights
         * 4. Update position estimate from particle distribution
         */
        void compute() override;

        /**
         * @brief Get X position change since last compute
         *
         * @return Delta X in inches since last update
         */
        double getDeltaX() override { return deltaX; }

        /**
         * @brief Get Y position change since last compute
         *
         * @return Delta Y in inches since last update
         */
        double getDeltaY() override { return deltaY; }

        /**
         * @brief Get heading change since last compute
         *
         * @return Delta angle in degrees since last update
         */
        Angle getDeltaAngle() override { return Angle(deltaTheta); }

        /**
         * @brief Inject absolute position measurement
         *
         * Updates particle weights based on an absolute position measurement
         * from external sensors (vision system, GPS, etc.). This helps correct
         * for odometry drift and improves localization accuracy.
         *
         * @param x Measured X coordinate in inches
         * @param y Measured Y coordinate in inches
         * @param headingDeg Measured heading in degrees (bearing system)
         * @param stdXY Standard deviation of position measurement (inches)
         * @param stdHeading Standard deviation of heading measurement (degrees)
         */
        void injectMeasurement(double x, double y, double headingDeg,
                              double stdXY = 2.0, double stdHeading = 5.0);

    private:
        Config config;                           /**< MCL configuration parameters */
        std::vector<Particle> particles;         /**< Particle population */
        std::default_random_engine rng{12345};   /**< Random number generator */
        double deltaX = 0;                       /**< X position change since last compute */
        double deltaY = 0;                       /**< Y position change since last compute */
        double deltaTheta = 0;                   /**< Heading change since last compute */

        /**
         * @brief Initialize particles randomly across the field
         *
         * Creates the initial particle population with uniform distribution
         * across the configured field dimensions.
         */
        void initParticles();

        /**
         * @brief Update particle positions based on motion model
         *
         * Propagates each particle forward based on the robot's movement
         * as reported by odometry, adding appropriate process noise.
         */
        void motionUpdate();

        /**
         * @brief Normalize particle weights to sum to 1.0
         *
         * Ensures particle weights form a valid probability distribution
         * by scaling them so they sum to 1.0.
         */
        void normalize();

        /**
         * @brief Resample particles based on their weights
         *
         * Creates a new particle set by sampling from the current particles
         * with probability proportional to their weights. This concentrates
         * particles in high-probability regions.
         */
        void resample();

        /**
         * @brief Update robot pose estimate from particle distribution
         *
         * Calculates the robot's position and heading as the weighted average
         * of all particles, providing the best estimate of the current pose.
         */
        void updateEstimate();
    };

} // namespace ls
