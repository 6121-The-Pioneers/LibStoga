#include "LibStoga/pure_pursuit.h"
#include "LibStoga/pid.h"
#include "LibStoga/spid.h"
#include "LibStoga/geometry.h"
#include "LibStoga/error.h"
#include <math.h>
#include "pure_pursuit.h"
#include "api.h"

namespace ls {

    PurePursuit::PurePursuit(std::vector<Point>& _path, ls::AbstractOdom& _odom, double _lookahead, double _error_tolerance, double _turn_sensitivity, double _speed) {
        path = _path;
        error_tolerance = _error_tolerance;
        odom = &_odom;
        speed = _speed;
        lookahead = _lookahead;
        turn_sensitivity = _turn_sensitivity;
    }

    void PurePursuit::move() {
        /**
         * TODO:
         * 
         * - /have SPID for movement = turning and forward motion.
         * - have it live to an array of path.
         * 
         */
        ls::SmartPID forward(0.4, 130, 0.1, 127, 0.1);
        ls::SmartPID turn(0.4, 90, 0.1, 127, 0.1);

        while (true) {
            //// update odom:
            odom->compute();

            //// define useful variables:
            ls::Position current_waypoint(path[waypoint].x, path[waypoint].y, 0);
            rx = path[waypoint].x-odom->getX();
            ry = path[waypoint].y-odom->getY();
            ls::Angle rel_heading(radiansToDegrees(atan2(rx, ry)));
            
            //// get power ratio - how much power needs to be proportionally decreased based on turn.
            double power_ratio;
            if (rel_heading.getAngle() >= 90 || rel_heading.getAngle() <= -90) {
               power_ratio = 0;
            } else {
               power_ratio = cos(turn_sensitivity * rel_heading.convertToRadians());
            }
            
            //// check to see if we can advance:
            if (abs(rx) <= error_tolerance && abs(ry) <= error_tolerance && waypoint < path.size() - 1) {
                forward.reset();
                turn.reset();
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
            if (path[waypoint].fowards) {
                distance = odom->getPosition().distanceFromPointSigned(current_waypoint);
            } else {
                distance = -odom->getPosition().distanceFromPointSigned(current_waypoint);
            }
            turn_output = turn.update(rel_heading.getAngle());
            power_output = power_ratio * forward.update(distance);

            //// delay to make sure brain safe from overheating:
            pros::delay(5);
        }
    }

    Point::Point(double x, double y, bool fowards, unsigned int delayAfterPoint) {
        this->x = x;
        this->y = y;
        this->fowards = fowards;
        this->delayAfterPoint = delayAfterPoint;
    }

    void Point::operator=(Point other)
    {
        this->x = other.x;
        this->y = other.y;
        this->fowards = other.fowards;
        this->delayAfterPoint = other.delayAfterPoint;
    }
};

