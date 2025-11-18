#include "chassis.h"
#include "timer.hpp"
#include "geometry.h"
#include "odom.h"
#include "spid.h"
#include <algorithm>

#define LOOP_DELAY 1

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

    void Chassis::moveToPoint(double X, double Y, unsigned int timeout, bool reverse) {
        
        // --- LemLib-inspired advanced moveToPoint ---
        ls::Timer timer(timeout);
        ls::Position goal(X, Y, 0); // IGNORE THETA
        unsigned int exit_timer = 0;
        Angle d_theta(0);
        bool close = false;
        double prevLateralOut = 0;
        double prevAngularOut = 0;
        double maxLateralSpeed = 127;
        double minLateralSpeed = 20;
        double maxAngularSpeed = 127;

        while (!timer.isDone()) {
            odom->compute();
            ls::Position current_position = odom->getPosition();

            double distance = goal.distanceFromPointSigned(current_position) * move_priority(d_theta);
            Angle theta_final = current_position.angleToPosition(goal);

            if (reverse) {
                theta_final += Angle(180);
                theta_final = theta_final.normalize();
            } else {
                distance *= -1;
            }

            d_theta = current_position.theta.minimumAngleDifference(theta_final);

            // Dynamic speed adjustment when close
            if (!close && fabs(distance) < 7.5) {
                close = true;
                maxLateralSpeed = std::max(fabs(prevLateralOut), 50.0);
                maxAngularSpeed = std::max(fabs(prevAngularOut), 50.0);
            }



            // PID outputs

            double lateralOut = lateral_control->update(distance);
            lateralOut = std::clamp(lateralOut, -maxLateralSpeed, maxLateralSpeed);
            if (!close && reverse) lateralOut = std::clamp(lateralOut, -maxLateralSpeed, -minLateralSpeed);
            else if (!close && !reverse) lateralOut = std::clamp(lateralOut, minLateralSpeed, maxLateralSpeed);
            prevLateralOut = lateralOut;

            double angularOut = close ? 0 : angular_control->update(d_theta.getAngle());
            angularOut = std::clamp(angularOut, -maxAngularSpeed, maxAngularSpeed);
            prevAngularOut = angularOut;

            // Robust exit conditions
            if (fabs(distance) <= threshold_lateral) {
                exit_timer += LOOP_DELAY;
            }
            if (exit_timer > threshold_timeout && close) {
                break;
            }

            // Set power
            right->move(lateralOut + angularOut);
            left->move(lateralOut - angularOut);

            pros::delay(LOOP_DELAY);
        }

        right->move(0);
        left->move(0);
    }

    void Chassis::turnToPoint(double X, double Y, unsigned int timeout, bool reverse) {
        ls::Timer timer(timeout);
        ls::Position goal(X, Y, 0); // IGNORE THETA
        unsigned int exit_timer = 0;

        while (!timer.isDone()) {
            odom->compute();
            ls::Position current_position = odom->getPosition();

            // calculate amount of change:
            Angle theta_final = current_position.angleToPosition(goal);

            if (reverse) {
                theta_final += Angle(180);
                theta_final = theta_final.normalize();
            }

            Angle d_theta = current_position.theta.minimumAngleDifference(theta_final); // order might be subject to change

            // calculate power and PID outputs:
            double turn = angular_control->update(d_theta.getAngle());

            if (fabs(current_position.theta.getAngle()) <= threshold_angular) {
                exit_timer += LOOP_DELAY;
            }

            if (exit_timer > threshold_timeout) {
                break;
            }

            // set power:
            right->move(turn); // order might be subject to change
            left->move(-turn); // order might be subject to change

            pros::delay(LOOP_DELAY);

        }

        right->move(0);
        left->move(0);
    }

    double Chassis::move_priority(Angle& angle) const {
        return std::clamp(cos(turn_sensitivity * angle.convertToRadians()), 0.0, 1.0);
    }
}