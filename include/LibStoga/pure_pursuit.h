// /**
//  * @file pure_pursuit.h
//  * @author Lord Nick
//  * @brief the file that contains all the code for the pure-persuit algorithm.
//  * @version 0.1
//  * @date 2024-11-12
//  * 
//  * @copyright Copyright (c) 2024
//  * 
//  */
// #ifndef PURE_PURSUIT_LS_H
// #define PURE_PURSUIT_LS_H

// #include <vector>
// #include <array>
// #include "odom.h"

// namespace ls {
//     /**
//      * @brief The struct that represents all moving motors and parts of 6121D
//      * 
//      */
//     struct HighStakesRobot {
//         bool mogo_state = false;
//         pros::MotorGroup* right;
//         pros::MotorGroup* left;
//         pros::MotorGroup* intake;
//         pros::MotorGroup* wallstake;
//         pros::ADIDigitalOut* mogo;

//         HighStakesRobot(pros::MotorGroup& right, pros::MotorGroup& left, pros::MotorGroup& intake, pros::MotorGroup& wallstake, pros::ADIDigitalOut& mogo);
//     };

//     struct pure_persuit_parameters_t {
//         std::vector<Point> waypoints;
//         double lookahead;
//         double error_tolerance = 1;
//         double turn_sensitivity = 1;
//         double speed = 127;
//     };

//     class PurePursuit {
//     public:
//         /**
//          * @brief Construct a new Pure Pursuit object
//          * 
//          * @param path the path the bot will follow
//          * @param error_tolerance the error tolerance it should have to get to a point.
//          */
//         PurePursuit(std::vector<Point>& path, ls::AbstractOdom& odom, ls::HighStakesRobot& robot, double lookahead, double error_tolerance = 1, double turn_sensitivity = 1, double speed = 127);

//         /**
//          * @brief Given the points and other required parameters, move the
//          * robot. This function will block excecution until robot is done with
//          * all required movements.
//          *
//          */
//         void move();

//         /**
//          * @brief performs an action that is not relating to moving on the field.
//          * 
//          * @param action action to be performed.
//          */
//         void performAction(Actions action);

//     private:
//         double error_tolerance;
//         double speed;
//         double lookahead;
//         double turn_sensitivity;
//         int waypoint;

//         ls::AbstractOdom* odom;
//         std::vector<Point> path;
//         ls::HighStakesRobot* robot;

//         double rx;
//         double ry;
//         double rh;
//     };

//     enum class Actions {
//         NONE = 0,
//         INTAKE_STOP = 1,
//         INTAKE = 2,
//         OUTTAKE = 3,
//         MOGO_UP = 4,
//         MOGO_DOWN = 5,
//         MOGO_TOGGLE = 6,
//         WALLSTAKE_INACTIVE = 7,
//         WALLSTAKE_ACTIVE = 8,
//         WALLSTAKE_SCORE = 9,
//     };

//     struct Point {
//         Point(double x, double y, bool foward, unsigned int delayAfterPoint = 5);
//         double x;
//         double y;
//         bool fowards;
//         unsigned int delayAfterPoint = 5; // delay starts after moving to point
//         Actions action = Actions::NONE; // action starts before moving to point

//         void operator=(Point other);
//     };

//     /**
//      * @brief A better implementation of pure persuit, comparable to lemlib's version:
//      * 
//      * Plan:
//      * - get steering angle using following equation: https://youtu.be/qYR7mmcwT2w?t=225 (L is lookahead distance)
//      * - speed is a constant set beforehand (only plus minus that is allowed)
//      * - based on turn decrease speed proportionaly
//      * - use PID for turns.
//      * 
//      */
// };

// #endif // PURE_PURSUIT_LS_H