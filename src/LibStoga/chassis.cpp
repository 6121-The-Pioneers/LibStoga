#include "chassis.h"
#include "timer.hpp"
#include "geometry.h"
#include "odom.h"
#include "spid.h"
#include <algorithm>

#define LOOP_DELAY 5

namespace ls {
    Chassis::Chassis(pros::MotorGroup& _right, pros::MotorGroup& _left, ls::AbstractOdom& _odom, ls::AbstractPID& _lateral_control, ls::AbstractPID& _angular_control, double _turn_sensitivity, double _threshold_lateral, double _threshold_angular, unsigned int _threshold_timeout) {
        right = &_right;
        left = &_left;
        odom = &_odom;
        lateral_control = &_lateral_control;
        angular_control = &_angular_control;
        turn_sensitivity = _turn_sensitivity;
        threshold_lateral = _threshold_lateral;
        threshold_angular = _threshold_angular;
        threshold_timeout = _threshold_timeout;
    }

    void Chassis::operator=(Chassis other) {
        right = other.right;
        left = other.left;
        odom = other.odom;
        lateral_control = other.lateral_control;
        angular_control = other.angular_control;
    }

    void Chassis::moveToPointLinear(double X, double Y, unsigned int timeout, bool reverse) { // TODO make curveless toPoint complete on next weekend
        ls::Timer timer(timeout);
        ls::Position goal(X, Y, 0); // IGNORE THETA
        unsigned int exit_timer = 0;

        while (!timer.isDone()) {
            odom->compute();
            ls::Position current_position = odom->getPosition();
            
            // calculate amount of change:
            double distance = goal.distanceFromPointSigned(current_position);
            Angle theta_final = current_position.angleToPosition(goal);

            pros::lcd::print(0, "d to target: %f", distance);
            pros::lcd::print(2, "th to target: %f", theta_final);
            
            if (reverse) {
                theta_final += Angle(180);
                theta_final = theta_final.normalize();
            }
            else {
                distance *= -1;
            }

            Angle d_theta = current_position.theta.minimumAngleDifference(theta_final); // order might be subject to change
            pros::lcd::print(3, "dth to target: %f", d_theta);

            // calculate power and PID outputs:
            double power = lateral_control->update(distance) * move_priority(d_theta);
            double turn = angular_control->update(d_theta.getAngle());
            pros::lcd::print(5, "move_priority: %f", move_priority(d_theta));

            if (fabs(distance) <= threshold_lateral) {
                exit_timer += LOOP_DELAY;
            }

            if (exit_timer > threshold_timeout) {
                break;
            }

            // set power: (UNCOMMENT WHEN CONFIDENT IT WORKS)
            right->move(power + turn); // order might be subject to change
            left->move(power - turn); // order might be subject to change

            pros::delay(LOOP_DELAY);

        }

        right->move(0);
        left->move(0);
    }

    void Chassis::turnToPoint(double X, double Y, unsigned int timeout, bool reverse) {
        ls::Timer timer(timeout);
        ls::Position goal(X, Y, 0); // IGNORE THETA
        odom->compute();
        Angle theta_final = odom->getPosition().angleToPosition(goal);

        if (reverse) {
            theta_final += Angle(180);
            theta_final = theta_final.normalize();
        }

        unsigned int exit_timer = 0;

        while (!timer.isDone()) {
            odom->compute();
            ls::Position current_position = odom->getPosition();

            // calculate amount of change:
            Angle d_theta = odom->getPosition().theta.minimumAngleDifference(theta_final); // order might be subject to change

            // calculate power and PID outputs:
            double turn = angular_control->update(d_theta.getAngle());

            if (fabs(d_theta.getAngle()) <= threshold_angular) {
                exit_timer += LOOP_DELAY;
            }

            if (exit_timer > threshold_timeout) {
                break;
            }

            // set power:
            right->move(-turn); // order might be subject to change
            left->move(turn); // order might be subject to change

            odom->compute();
            pros::delay(LOOP_DELAY);
        }

        right->move(0);
        left->move(0);
    }

    double Chassis::move_priority(Angle& angle) const {
        return std::clamp(cos(turn_sensitivity * angle.convertToRadians()), 0.0, 1.0);
    }
}