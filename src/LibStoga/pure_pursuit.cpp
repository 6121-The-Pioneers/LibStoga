#include "LibStoga/pure_pursuit.h"
#include "LibStoga/pid.h"
#include "LibStoga/spid.h"
#include "LibStoga/geometry.h"
#include "LibStoga/error.h"
#include "LibStoga/chassis.h"
#include <math.h>
#include "pure_pursuit.h"
#include "api.h"

namespace ls {
    PurePursuit::PurePursuit(ls::AbstractOdom& _odom, ls::PID* _forward, ls::PID* _turn, ls::Chassis* _chassis, double _turn_sensitivity) {
        odom = &_odom;
        turn_sensitivity = _turn_sensitivity;
        this->forward = forward;
        this->turn = turn;
        this->chassis = chassis;
    }

    void PurePursuit::moveToPoint(Point point)
    {
        while (true) {
            //// update odom:
            odom->compute();

            //// define useful variables:
            ls::Position current_waypoint(point.position.X, point.position.Y, 0);
            rx = point.position.X - odom->getX();
            ry = point.position.Y - odom->getY();
            ls::Angle rel_heading(radiansToDegrees(atan2(rx, ry)));
            
            //// get power ratio - how much power needs to be proportionally decreased based on turn.
            double power_ratio;
            if (rel_heading.getAngle() >= 90 || rel_heading.getAngle() <= -90) {
               power_ratio = 0;
            } else {
               power_ratio = cos(turn_sensitivity * rel_heading.convertToRadians());
            }
            
            //// check to see if we can advance:
            if (abs(rx) <= point.error_tolerance && abs(ry) <= point.error_tolerance) {
                forward->reset();
                turn->reset();
                pros::delay(point.delayAfterPoint);
                break;
            }
            
            //// Make the robot move based on calculations: 
            double distance;
            double turn_output;
            double power_output;
            if (point.forwards) {
                distance = odom->getPosition().distanceFromPointSigned(current_waypoint);
                turn_output = turn->update(rel_heading.getAngle());
            } else {
                distance = -odom->getPosition().distanceFromPointSigned(current_waypoint);
                turn_output = turn->update(rel_heading.getAngle() + 180);
            }

            power_output = power_ratio * forward->update(distance);

            chassis->right->move(power_output - turn_output);
            chassis->left->move(power_output + turn_output);

            //// delay to make sure brain safe from overheating:
            pros::delay(5);
        }
    }

    void PurePursuit::turnToPoint(Point point)
    {
        while (true) {
            //// update odom:
            odom->compute();

            //// define useful variables:
            ls::Position current_waypoint(point.position.X, point.position.Y, 0);
            ls::Angle rel_heading(radiansToDegrees(atan2(rx, ry)));
            
            //// check to see if we can advance:
            if (abs(rel_heading.getAngle()) <= point.error_tolerance) {
                turn->reset();
                pros::delay(point.delayAfterPoint);
                break;
            }
            
            //// Make the robot move based on calculations: 
            double turn_output;
            turn_output = turn->update(rel_heading.getAngle());

            chassis->right->move(-turn_output);
            chassis->left->move(turn_output);

            //// delay to make sure brain safe from overheating:
            pros::delay(5);
        }
    }

    void PurePursuit::followPath(std::vector<Point>& path) {
        while (true) {
            //// update odom:
            odom->compute();

            //// define useful variables:
            ls::Position current_waypoint(path[waypoint].position.X, path[waypoint].position.Y, 0);
            rx = path[waypoint].position.X-odom->getX();
            ry = path[waypoint].position.Y-odom->getY();
            ls::Angle rel_heading(radiansToDegrees(atan2(rx, ry)));
            
            //// get power ratio - how much power needs to be proportionally decreased based on turn.
            double power_ratio;
            if (rel_heading.getAngle() >= 90 || rel_heading.getAngle() <= -90) {
               power_ratio = 0;
            } else {
               power_ratio = cos(turn_sensitivity * rel_heading.convertToRadians());
            }
            
            //// check to see if we can advance:
            if (abs(rx) <= path[waypoint].error_tolerance && abs(ry) <= path[waypoint].error_tolerance && waypoint < path.size() - 1) {
                forward->reset();
                turn->reset();
                waypoint++;
                pros::delay(path[waypoint].delayAfterPoint);
                continue;
            } else if (waypoint >= path.size() - 1) {
                break;
            }
            
            //// Make the robot move based on calculations: 
            double distance;
            double turn_output;
            double power_output;
            if (path[waypoint].forwards) {
                distance = odom->getPosition().distanceFromPointSigned(current_waypoint);
                turn_output = turn->update(rel_heading.getAngle());
            } else {
                distance = -odom->getPosition().distanceFromPointSigned(current_waypoint);
                turn_output = turn->update(rel_heading.getAngle() + 180);
            }

            power_output = power_ratio * forward->update(distance);

            chassis->right->move(power_output - turn_output);
            chassis->left->move(power_output + turn_output);

            //// delay to make sure brain safe from overheating:
            pros::delay(5);
        }
    }

    Point::Point(Position pos, bool fowards, unsigned int delayAfterPoint, double error_tolerance) {
        this->position = pos;
        this->forwards = fowards;
        this->delayAfterPoint = delayAfterPoint;
        this->error_tolerance = error_tolerance;
    }

    void Point::operator=(Point other)
    {
        this->position = other.position;
        this->forwards = other.forwards;
        this->delayAfterPoint = other.delayAfterPoint;
        this->error_tolerance = other.error_tolerance;
    }
};

