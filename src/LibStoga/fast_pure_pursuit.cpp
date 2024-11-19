#include "fast_pure_pursuit.h"

ls::Chassis::Chassis(pros::MotorGroup& _right, pros::MotorGroup& _left)
{
    right = &_right;
    left = &_left;
}

// ls::FastPurePursuit::FastPurePursuit(fast_pure_persuit_parameters_t &parameters)
// {
//     odom = parameters.odom;
//     chassis = parameters.chassis;
//     // ls::Components components;
//     // turn;
        
//     // turn_sensitivity;
//     // inertia_radius;
//     // acceptable_goal_error;
//     // max_speed;

//     // path;
// }
