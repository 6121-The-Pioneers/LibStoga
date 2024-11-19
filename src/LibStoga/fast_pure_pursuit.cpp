#include "fast_pure_pursuit.h"

ls::Chassis::Chassis(pros::MotorGroup& _right, pros::MotorGroup& _left)
{
    right = &_right;
    left = &_left;
}