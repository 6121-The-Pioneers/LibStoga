#include "chassis.h"
#include "timer.hpp"
#include "geometry.h"

namespace ls {
    Chassis::Chassis(pros::MotorGroup& _right, pros::MotorGroup& _left, ls::AbstractOdom& _odom, ls::PID& _lateral_control, ls::PID& _angular_control) {
        right = &_right;
        left = &_left;
        odom = &_odom;
        lateral_control = &_lateral_control;
        angular_control = &_angular_control;
    }

    void Chassis::operator=(Chassis other) {
        right = other.right;
        left = other.left;
        odom = other.odom;
        lateral_control = other.lateral_control;
        angular_control = other.angular_control;
    }

    void Chassis::moveToPoint(double X, double Y, unsigned int timeout, bool reverse) { // TODO make curveless toPoint complete on next weekend
        ls::Timer timer(timeout);
        

        while (!timer.isDone()) {
            odom->compute();
            
            // calculate amount of change:
            double deltaX = X - odom->getX();
            double deltaY = Y - odom->getY();
            double distance = sqrt(deltaX*deltaX + deltaY*deltaY);

            ls::Angle goalAngle(radiansToDegrees(atan2(deltaY, deltaX)));
            goalAngle.normalize();

            ls::Angle deltaAngle = odom->getPosition().theta.minimumAngleDifference(goalAngle);

            // set power:


            pros::delay(5);
        }
    }
}