#include "LibStoga/pure_pursuit.h"
#include "LibStoga/pid.h"
#include "LibStoga/spid.h"
#include "LibStoga/geometry.h"
#include "LibStoga/error.h"
#include <math.h>

namespace ls {

    PurePursuit::PurePursuit(std::vector<double>& _target_x, std::vector<double>& _target_y, ls::AbstractOdom& _odom, double _lookahead, double _error_tolerance, double _turn_sensitivity, double _speed) {
        if (target_x.size() != target_y.size()) {
            THROW_ERROR("X and Y target arrays are unequal in length, please double check your points.");
        }
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
        ls::SmartPID forward(0.4, 130, 0.1, 127);
        ls::SmartPID turn(0.4, 90, 0.1, 127);

        while (true) {
            //// update odom:
            odom->compute();

            //// define useful variables:
            ls::Position current_waypoint(path[waypoint].x, path[waypoint].y, 0);
            rx = path[waypoint].x-x;
            ry = path[waypoint].y-y;
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
                continue;
            } else if (waypoint >= path.size() - 1) {
                break;
            }
            
            //// Make the robot move based on calculations: 
            double distance = odom->getPosition().distanceFromPointSigned(current_waypoint);
            double turn_output = turn.update(rel_heading.getAngle());
            double power_output = power_ratio * forward.update(distance);

            // delay to make sure brain safe from overheating:
            pros::delay(5);
        }
    }

    point::point(double x, double y, bool fowards) {
        this.x = x;
        this.y = y;
        this.fowards = fowards;
    }


}