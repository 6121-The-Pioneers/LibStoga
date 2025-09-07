#include "chassis.h"
#include "timer.hpp"
#include "geometry.h"
#include "odom.h"
#include "spid.h"
#include "pure_pursuit.h"
#include "util/clamp.hpp"
#include "auton_tuner.h"
#include <cmath>
#include <algorithm>

#define LOOP_DELAY 1

// Uncomment to enable verbose motion debug output (LCD + std::cout)
//#define LS_DEBUG_MOTION

#ifdef LS_DEBUG_MOTION
#include <iostream>
#include <cstdarg>
#include <cstdio>
namespace {
    inline void ls_debug_printf(const char* fmt, ...) {
        char buf[160];
        va_list args; va_start(args, fmt);
        vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        std::cout << "[MOTION] " << buf << std::endl;
    }
}
#define DBG_PRINT(line, fmt, ...) do { pros::lcd::print(line, fmt, __VA_ARGS__); ls_debug_printf(fmt, __VA_ARGS__); } while(0)
#else
#define DBG_PRINT(line, fmt, ...)
#endif

namespace ls {
    Chassis::Chassis(std::shared_ptr<pros::MotorGroup> _right, std::shared_ptr<pros::MotorGroup> _left, std::shared_ptr<ls::AbstractOdom> _odom, std::shared_ptr<ls::AbstractPID> _lateral_control, std::shared_ptr<ls::AbstractPID> _angular_control, const ChassisConfig& cfg) 
    : right(_right), left(_left), odom(_odom), lateral_control(_lateral_control), angular_control(_angular_control) {
        turn_sensitivity = cfg.turnSensitivity;
        threshold_lateral = cfg.lateralThreshold;
        threshold_angular = cfg.angularThreshold;
        threshold_timeout = cfg.dwellTimeMs;
        enable_cancel = cfg.enableCancel;
        cancel_button = cfg.cancelButton;
        enable_adaptive_pid = cfg.enableAdaptivePID;
        controller = nullptr;

        // Initialize adaptive PID controllers if enabled
        if (enable_adaptive_pid) {
            // Get current gains from the base controllers
            PIDGains lateral_gains = lateral_control->getGains();
            PIDGains angular_gains = angular_control->getGains();

            // Initialize adaptive controllers with current gains
            lateral_adaptive_controller = std::make_unique<AdaptivePIDController>(
                lateral_control, lateral_gains);
            angular_adaptive_controller = std::make_unique<AdaptivePIDController>(
                angular_control, angular_gains);

            std::cout << "Adaptive PID enabled for chassis" << std::endl;
            std::cout << "Base Lateral: P=" << lateral_gains.kP << ", I=" << lateral_gains.kI << ", D=" << lateral_gains.kD << std::endl;
            std::cout << "Base Angular: P=" << angular_gains.kP << ", I=" << angular_gains.kI << ", D=" << angular_gains.kD << std::endl;
        }
    }

    void Chassis::setController(pros::Controller& cont) {
        controller = &cont;
    }

    void Chassis::updateAdaptivePID() {
        if (!enable_adaptive_pid) return;

        if (lateral_adaptive_controller) {
            lateral_adaptive_controller->update();
        }
        if (angular_adaptive_controller) {
            angular_adaptive_controller->update();
        }
    }

    std::shared_ptr<ls::AbstractOdom> Chassis::getOdom() {
        return odom;
    }

    std::shared_ptr<AbstractPID> Chassis::getLateralControl() {
        return lateral_control;
    }

    std::shared_ptr<AbstractPID> Chassis::getAngularControl() {
        return angular_control;
    }

    void Chassis::moveToPoint(double X, double Y, unsigned int timeout, bool reverse) {
        if (!odom || !lateral_control || !angular_control || !right || !left) return; // safety
        ls::Timer timer(timeout);
        ls::Position goal(X, Y, 0); // IGNORE THETA
        unsigned int exit_timer = 0;
        double initial_distance = odom->getPosition().distanceFromPoint(goal);
        int log_counter = 0;

        while (!timer.isDone()) {
            if (enable_cancel && controller && controller->get_digital(cancel_button)) {
                std::cout << "Motion cancelled by user" << std::endl;
                break;
            }
            odom->compute();
            ls::Position current_position = odom->getPosition();
            
            // calculate amount of change:
            double distance = goal.distanceFromPointSigned(current_position);
            Angle theta_final = current_position.angleToPosition(goal);
            
            if (reverse) {
                theta_final += Angle(180);
                theta_final = theta_final.normalize();
            }
            else {
                distance *= -1;
            }

            double percent_complete = (1.0 - (fabs(distance) / initial_distance)) * 100.0;
            if (static_cast<int>(percent_complete / 10) > log_counter) {
                log_counter = static_cast<int>(percent_complete / 10);
                std::cout << "Motion " << log_counter * 10 << "% complete" << std::endl;
            }

            Angle d_theta = current_position.theta.minimumAngleDifference(theta_final); // order might be subject to change

            // calculate power and PID outputs:
            double raw_power = lateral_control->update(distance);
            double power = raw_power * move_priority(d_theta);
            double turn = angular_control->update(d_theta.getAngle());

            if (std::isnan(distance) || std::isnan(d_theta.getAngle())) {
                DBG_PRINT(6, "NaN detected dist=%f ang=%f", distance, d_theta.getAngle());
                break;
            }
            if (fabs(distance) <= threshold_lateral) {
                exit_timer += LOOP_DELAY;
            } else {
                exit_timer = 0; // reset dwell timer when leaving threshold band
            }

            if (exit_timer > threshold_timeout) {
                break;
            }

            // set power (clamped):
            right->move(clampPower(power + turn)); // order might be subject to change
            left->move(clampPower(power - turn)); // order might be subject to change

            DBG_PRINT(5, "MT d=%.2f ang=%.2f p=%.2f t=%.2f", distance, d_theta.getAngle(), power, turn);

            pros::delay(LOOP_DELAY);

        }

        right->move(0);
        left->move(0);
    }

    void Chassis::turnToPoint(double X, double Y, unsigned int timeout, bool reverse) {
        if (!odom || !angular_control || !right || !left) return; // safety
        ls::Timer timer(timeout);
        ls::Position goal(X, Y, 0); // IGNORE THETA
        unsigned int exit_timer = 0;
        odom->compute();
        Angle initial_angle_difference = odom->getPosition().theta.minimumAngleDifference(odom->getPosition().angleToPosition(goal));
        int log_counter = 0;

        while (!timer.isDone()) {
            if (enable_cancel && controller && controller->get_digital(cancel_button)) {
                std::cout << "Motion cancelled by user" << std::endl;
                break;
            }
            odom->compute();
            ls::Position current_position = odom->getPosition();

            // calculate amount of change:
            Angle theta_final = current_position.angleToPosition(goal);

            if (reverse) {
                theta_final += Angle(180);
                theta_final = theta_final.normalize();
            }

            Angle d_theta = current_position.theta.minimumAngleDifference(theta_final); // order might be subject to change

            double percent_complete = (1.0 - (fabs(d_theta.getAngle()) / fabs(initial_angle_difference.getAngle()))) * 100.0;
            if (static_cast<int>(percent_complete / 10) > log_counter) {
                log_counter = static_cast<int>(percent_complete / 10);
                std::cout << "Motion " << log_counter * 10 << "% complete" << std::endl;
            }

            // calculate power and PID outputs:
            double turn = angular_control->update(d_theta.getAngle());

            if (std::isnan(d_theta.getAngle())) {
                DBG_PRINT(6, "NaN turn dAng");
                break;
            }
            if (fabs(d_theta.getAngle()) <= threshold_angular) {
                exit_timer += LOOP_DELAY;
            } else {
                exit_timer = 0;
            }

            if (exit_timer > threshold_timeout) {
                break;
            }

            // set power:
            right->move(clampPower(-turn));
            left->move(clampPower(turn));

            DBG_PRINT(4, "TP dAng=%.2f turn=%.2f", d_theta.getAngle(), turn);

            pros::delay(LOOP_DELAY);

        }

        right->move(0);
        left->move(0);
    }

    double Chassis::move_priority(Angle& angle) const {
        return std::clamp(cos(turn_sensitivity * angle.convertToRadians()), 0.0, 1.0);
    }

    void Chassis::followPath(const Path &path, double lookahead, unsigned int timeout) {
        if (path.empty()) return;
        PurePursuit pp(*odom, lookahead);
        pp.setPath(path);
        ls::Timer timer(timeout);
        while(!timer.isDone() && !pp.isFinished()) {
            odom->compute();
            auto out = pp.update();
            if (out.done) break;
            double linear = out.linear;
            double angular = out.angular;
            // convert linear/angular to left/right (simplified differential drive model)
            double leftPower = linear + angular;
            double rightPower = linear - angular;
            left->move(clampPower(leftPower));
            right->move(clampPower(rightPower));
            pros::delay(10);
        }
        left->move(0);
        right->move(0);
    }

double Chassis::driveRelative(double distance, unsigned int timeout) {
    if (!odom || !lateral_control) return -1; // safety
    ls::Timer timer(timeout);
    unsigned int exit_timer = 0;
    odom->resetAll(); // Reset odom to properly measure relative distance
    odom->compute(); // Update odometry after reset
    double initial_dist = odom->getPosition().Y; // Get initial Y position after reset
    int log_counter = 0;
    double total_error = 0.0;

    while(!timer.isDone()) {
        if (enable_cancel && controller && controller->get_digital(cancel_button)) {
            std::cout << "Motion cancelled by user" << std::endl;
            break;
        }
        odom->compute();
        double current_dist = odom->getPosition().Y; // Assuming Y is the forward direction
        double error = distance - (current_dist - initial_dist);
        total_error += fabs(error) * (LOOP_DELAY / 1000.0); // Integrate error over time

        // Debug output for odometry diagnosis
        static int debug_counter = 0;
        if (debug_counter++ % 10 == 0) { // Print every 10 iterations
            DBG_PRINT(4, "ODOM: X=%.2f Y=%.2f Ang=%.2f", odom->getPosition().X, odom->getPosition().Y, odom->getAngle());
            DBG_PRINT(5, "DRV: curr=%.2f init=%.2f err=%.2f", current_dist, initial_dist, error);
        }

        double percent_complete = (1.0 - (fabs(error) / fabs(distance))) * 100.0;
        if (static_cast<int>(percent_complete / 10) > log_counter) {
            log_counter = static_cast<int>(percent_complete / 10);
            std::cout << "Motion " << log_counter * 10 << "% complete" << std::endl;
        }

        double power = lateral_control->update(error);

        if (fabs(error) <= threshold_lateral) {
            exit_timer += LOOP_DELAY;
        } else {
            exit_timer = 0;
        }

        if (exit_timer > threshold_timeout) break;

        right->move(clampPower(power));
        left->move(clampPower(power));

        DBG_PRINT(3, "DRV rel=%.2f err=%.2f pwr=%.2f", distance, error, power);
        pros::delay(LOOP_DELAY);
    }
    right->move(0);
    left->move(0);
    return total_error;
}

double Chassis::turnRelative(double deltaAngle, unsigned int timeout) {
    if (!odom) return -1; // safety
    odom->compute();
    double target = odom->getAngle() + deltaAngle;
    while (target >= 360) target -= 360;
    while (target < 0) target += 360;
    DBG_PRINT(2, "TRN rel=%.2f tgt=%.2f", deltaAngle, target);
    return turnToAngle(target, timeout);
}

double Chassis::turnToAngle(double angle, unsigned int timeout) {
    if (!odom || !angular_control || !right || !left) return -1; // safety
    ls::Timer timer(timeout);
    unsigned int exit_timer = 0;
    Angle target(angle);
    odom->compute();
    Angle initial_angle_difference = odom->getPosition().theta.minimumAngleDifference(target);
    int log_counter = 0;
    double total_error = 0.0;

    while(!timer.isDone()) {
        if (enable_cancel && controller && controller->get_digital(cancel_button)) {
            std::cout << "Motion cancelled by user" << std::endl;
            break;
        }
        odom->compute();
        Angle current(odom->getAngle());
        Angle d = current.minimumAngleDifference(target);
        total_error += fabs(d.getAngle()) * (LOOP_DELAY / 1000.0); // Integrate error over time

        double percent_complete = (1.0 - (fabs(d.getAngle()) / fabs(initial_angle_difference.getAngle()))) * 100.0;
        if (static_cast<int>(percent_complete / 10) > log_counter) {
            log_counter = static_cast<int>(percent_complete / 10);
            std::cout << "Motion " << log_counter * 10 << "% complete" << std::endl;
        }

        double turn = angular_control->update(d.getAngle());
        if (fabs(d.getAngle()) <= threshold_angular) {
            exit_timer += LOOP_DELAY;
        } else {
            exit_timer = 0;
        }
        if (exit_timer > threshold_timeout) break;
        right->move(clampPower(turn));
        left->move(clampPower(-turn));
        DBG_PRINT(1, "TA dAng=%.2f out=%.2f", d.getAngle(), turn);
        pros::delay(LOOP_DELAY);
    }
    right->move(0);
    left->move(0);
    return total_error;
}    void Chassis::arcade(double forward, double turn) {
        if (!right || !left) return;
        double left_power = forward + turn;
        double right_power = forward - turn;
        right->move(clampPower(right_power));
        left->move(clampPower(left_power));
    }
}