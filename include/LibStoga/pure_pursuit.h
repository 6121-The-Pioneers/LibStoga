/**
 * @file pure_pursuit.h
 * @author Lord Nick
 * @brief the file that contains all the code for the pure-persuit algorithm.
 * @version 0.1
 * @date 2024-11-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef PURE_PURSUIT_LS_H
#define PURE_PURSUIT_LS_H

#include <vector>
#include <array>
#include "odom.h"

namespace ls {
    struct pure_persuit_parameters_t {
        std::vector<std::array<double, 2>> waypoints;
        double lookahead;
        double error_tolerance = 1;
        double turn_sensitivity = 1;
        double speed = 127;
    };

    class PurePursuit {
    public:
        /**
         * @brief Construct a new Pure Pursuit object
         * 
         * @param target_x the array of x coordinates of the targets
         * @param target_y the array of y coordinates of the targets
         * @param error_tolerance the error tolerance it should have to get to a point.
         */
        PurePursuit(std::vector<double>& target_x, std::vector<double>& target_y, ls::AbstractOdom& odom, double lookahead, double error_tolerance = 1, double turn_sensitivity = 1, double speed = 127);

        /**
         * @brief Given the points and other required parameters, move the robot.
         * This function will block excecution until robot is done with all required movements.
         * 
         */
        void move();

    private:
        double error_tolerance;
        double speed;
        double lookahead;
        double turn_sensitivity;

        double x;
        double y;
        double heading;
        int waypoint;

        std::vector<double> target_x;
        std::vector<double> target_y;
        ls::AbstractOdom* odom;

        double rx;
        double ry;
        double rh;
    };

    /**
     * @brief A better implementation of pure persuit, comparable to lemlib's version:
     * 
     * Plan:
     * - get steering angle using following equation: https://youtu.be/qYR7mmcwT2w?t=225 (L is lookahead distance)
     * - speed is a constant set beforehand (only plus minus that is allowed)
     * - based on turn decrease speed proportionaly
     * - use PID for turns.
     * 
     */
};

#endif // PURE_PURSUIT_LS_H