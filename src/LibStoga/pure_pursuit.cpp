#include "LibStoga/pure_pursuit.h"
#include "LibStoga/pid.h"
#include "LibStoga/geometry.h"
#include "LibStoga/error.h"
#include <math.h>

namespace ls {
    
    PurePursuit::PurePursuit(std::vector<double>& _target_x, std::vector<double>& _target_y, ls::AbstractOdom& _odom, double _speed, double _error_tolerance) {
        if (target_x.size() != target_y.size()) {
            THROW_ERROR("X and Y target arrays are unequal in length, please double check your points.");
        }
        target_x = _target_x;
        target_y = _target_y;
        error_tolerance = _error_tolerance;
        odom = &_odom;
        speed= _speed;
    }

    void PurePursuit::move() {
        /**
         * TODO:
         * 
         * - x & y updated by odom.
         * - have SPID for movement = turning and forward motion.
         * - have it live to an array of path.
         * 
         */

        while (true) {
            double current_target_x = target_x[waypoint];
            double current_target_y = target_y[waypoint];
            
            float rx = current_target_x - x;
            float ry = current_target_y - y;

            float rh = radiansToDegrees(atan2(rx,ry)); // sensor fuse with IMU
            float xv = speed * sin(degreesToRadians(heading));
            float yv = speed * cos(degreesToRadians(heading));
            
            if (heading > 360) {
                heading -= 360;
            } else if (heading < 0) {
                heading += 360;
            }
            
            if (rh < 0) {
                rh += 360;
            } else if (rh > 360) {
                rh -= 360;
            }
            
            if (abs(rx) <= error_tolerance && abs(ry) <= error_tolerance && waypoint < target_x.size() - 1) {
                speed = 0;
                waypoint++;
            } else if (waypoint >= target_x.size() - 1) {
                break;
            }
            
            //// TODO: 
            x += xv;
            y += yv;

            // delay to make sure brain safe from overheating:
            pros::delay(5);
        }
    }


}