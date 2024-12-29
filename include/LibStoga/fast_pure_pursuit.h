// /**
//  * @file fast_pure_pursuit.h
//  * @author Rishit Varshney
//  * @brief Contains code realted to the Fast Pure Pursuit (FPP) Algorithm
//  * @version 0.1
//  * @date 2024-11-18
//  * 
//  * @copyright Copyright (c) 2024
//  * 
//  */
// #ifndef FAST_PURE_PURSUIT_LS_H
// #define FAST_PURE_PURSUIT_LS_H

// #include "odom.h"
// #include "spid.h"
// #include "pure_pursuit.h"
// #include "chassis.h"

// namespace ls {
//     /**
//      * @brief contains all parameters to FastPurePursuit.
//      *
//      * Order: odom (AbstractOdom*), chassis (Chassis*), turn values (smart_pid_parameters_t), turn sensitivity (double), inertia_radius (double), acceptable_goal_error (double), max_speed (int), path (vector<Position>) 
//      */
//     struct fast_pure_persuit_parameters_t {
//         ls::AbstractOdom* odom;
//         ls::Chassis* chassis;
//         ls::smart_pid_parameters_t turn;
        
//         double turn_sensitivity;
//         double inertia_radius;
//         double acceptable_goal_error;
//         int max_speed;

//         std::vector<Position> path;
//     };

//     /**
//      * @brief Represents the Fast Pure Persuit algorithm. 
//      * 
//      * Aims to speed up classic pure persuit while keeping some of its accuracy benifits.
//      */
//     class FastPurePursuit {
//     private:
//         ls::AbstractOdom* odom;
//         ls::Chassis* chassis;
//         std::unique_ptr<ls::SmartPID> turn;
        
//         double turn_sensitivity;
//         double inertia_radius;
//         double acceptable_goal_error;
//         int max_speed;

//         std::vector<Position> path;
//     public:
//         /**
//          * @brief Construct a new Fast Pure Pursuit object
//          * 
//          * @param parameters parameters structure for this class
//          */
//         explicit FastPurePursuit(fast_pure_persuit_parameters_t& parameters);

//         /**
//          * @brief Completes entire autonomous path
//          * 
//          * Will block all code untill movement is finished.
//          */
//         void move();
//     };
// };


// #endif // FAST_PURE_PURSUIT_LS_H