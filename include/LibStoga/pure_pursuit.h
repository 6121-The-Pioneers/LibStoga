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
#include "odom.h"

namespace ls {
    class PurePursuit {
    public:
        /**
         * @brief Construct a new Pure Pursuit object
         * 
         * @param target_x the array of x coordinates of the targets
         * @param target_y the array of y coordinates of the targets
         * @param error_tolerance the error tolerance it should have to get to a point.
         */
        PurePursuit(std::vector<double>& target_x, std::vector<double>& target_y, ls::AbstractOdom& odom, double speed, double error_tolerance = 1);

        /**
         * @brief Given the points and other required parameters, move the robot.
         * This function will block excecution until robot is done with all required movements.
         * 
         */
        void move();

    private:
        double error_tolerance;
        double speed;

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
}

#endif // PURE_PURSUIT_LS_H