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
         * @param _odom the odom object for tracking position
         * @param _forward the SPID instance for the forward part of drive PID
         * @param _turn the SPID instance for the turn part of drive PID
         * @param _chassis chassis object containing left and right drivetrain
         * @param _turn_sensitivity the sensity the robot should have for accurate turns.
         */
        explicit PurePursuit(ls::AbstractOdom& _odom, ls::SmartPID& _forward, ls::SmartPID& _turn, ls::Chassis& _chassis, double _turn_sensitivity = 1);

        /**
         * @brief Given the point to go to, moves the robot to that point.
         * This function will block excecution until robot is done with movement.
         *
         * @param _point the point to move to
         */
        void moveToPoint(Point& _point);

        /**
         * @brief Given the path to follow, moves the robot throughout the path.
         * This function will block excecution until robot is done with all movements.
         *
         * @param _path the path to follow
         */
        void followPath(std::vector<Point>& path);

        /**
         * @brief Given the point to turn to, turns the robot to that point.
         * This function will block excecution until robot is done with movement.
         *
         * @param _point the point to turn to
         */
        void turnToPoint(Point& _point);

    private:
        double turn_sensitivity;
        int waypoint;

        ls::AbstractOdom* odom;
        ls::SmartPID* forward;
        ls::SmartPID* turn;
        ls::Chassis* chassis;

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