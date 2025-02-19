#include "chassis.h"

namespace ls {
    Chassis::Chassis(pros::MotorGroup& _right, pros::MotorGroup& _left) {
        right = &_right;
        left = &_left;
    }

    void Chassis::operator=(Chassis other) {
        right = other.right;
        left = other.left;
    }
}