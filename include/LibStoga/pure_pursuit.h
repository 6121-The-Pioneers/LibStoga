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

//// TODO ================================================================================================================= (Lord Nick)

#ifndef PURE_PURSUIT_LS_H
#define PURE_PURSUIT_LS_H

#include <vector>
#include <array>
#include "odom.h"

namespace ls {
    /**
     * @brief Structure that represents all parameters for PurePursuit.
     * 
     * Order: waypoints (vector<Position>), lookahead (double), error_tolerance (double), turn_sensitivity (double), speed (int)
     */
    struct pure_persuit_parameters_t {
        std::vector<Position> waypoints;
        double lookahead;
        double error_tolerance = 1;
        double turn_sensitivity = 1;
        int speed = 127;
    };

    struct Point
    {
        Position position;
        bool forwards;
        unsigned int delayAfterPoint;
        double error_tolerance;
        
        explicit Point(Position pos, bool fowards, unsigned int delayAfterPoint, double error_tolerance);
        void Point::operator=(Point other);
    };
    

    /**
     * @brief Class that contains all Pure Pursuit related stuff
     * 
     * Keep in mind that this is not the real pure pursuit algorithm.
     */
    class PurePursuit {
    public:
        /**
         * @brief Construct a new Pure Pursuit object
         * 
         * @param path the path the bot will follow
         * @param error_tolerance the error tolerance it should have to get to a point.
         */
        explicit PurePursuit(std::vector<Point>& path, ls::AbstractOdom& odom, double lookahead, double error_tolerance = 1, double turn_sensitivity = 1, double speed = 127);

        /**
         * @brief Given the points and other required parameters, move the
         * robot. This function will block excecution until robot is done with
         * all required movements.
         *
         * @param forward the SPID instance for the forward part of drive PID
         * @param turn the SPID instance for the turn part of drive PID
         * @param chassis chassis object containing left and right drivetrain
         */
        void move(ls::SmartPID& forward, ls::SmartPID& turn, ls::Chassis& chassis);

    private:
        double error_tolerance;
        double speed;
        double lookahead;
        double turn_sensitivity;
        int waypoint;

        ls::AbstractOdom* odom;
        std::vector<Point> path;

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