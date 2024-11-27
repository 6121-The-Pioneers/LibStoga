/**
 * @file fast_pure_pursuit.h
 * @author Rishit Varshney
 * @brief 
 * @version 0.1
 * @date 2024-11-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef FAST_PURE_PURSUIT_LS_H
#define FAST_PURE_PURSUIT_LS_H

#include "odom.h"
#include "spid.h"
#include "pure_pursuit.h"

namespace ls {
    struct Chassis {
        pros::MotorGroup* right;
        pros::MotorGroup* left;

        Chassis(pros::MotorGroup& _right, pros::MotorGroup& _left);
        void operator=(Chassis other);
    };

    struct fast_pure_persuit_parameters_t {
        ls::AbstractOdom* odom;
        ls::Chassis* chassis;
        // ls::Components components;
        ls::smart_pid_parameters_t turn;
        
        double turn_sensitivity;
        double inertia_radius;
        double acceptable_goal_error;
        int max_speed;

        std::vector<ls::Point> path;
    };

    class FastPurePursuit {
    private:
        ls::AbstractOdom* odom;
        ls::Chassis* chassis;
        // ls::Components components;
        std::unique_ptr<ls::SmartPID> turn;
        
        double turn_sensitivity;
        double inertia_radius;
        double acceptable_goal_error;
        int max_speed;

        std::vector<ls::Point> path;
    public:
        /**
         * @brief Construct a new Fast Pure Pursuit object
         * 
         * @param parameters parameters structure for this class
         */
        FastPurePursuit(fast_pure_persuit_parameters_t& parameters);

        /**
         * @brief Completes entire autonomous path
         * 
         * Will block all code untill movement is finished.
         */
        void move();
    };
};


#endif // FAST_PURE_PURSUIT_LS_H