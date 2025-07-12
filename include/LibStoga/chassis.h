/** ✅
 * @file chassis.h
 * @author Rishit Varshney
 * @brief contains organizational elements related to the physical robot itself.
 * @version 0.1
 * @date 2024-11-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef LS_CHASSIS_H
#define LS_CHASSIS_H

#include "api.h"
#include "odom.h"
#include "pid.h"

namespace ls {
    /**
     * @brief Represents the chassis of the robot. Only supports tank drive configuration (TODO)
     * 
     */
    class Chassis 
    {
    private:
        pros::MotorGroup* right;
        pros::MotorGroup* left;
        ls::AbstractOdom* odom;

        ls::PID* lateral_control;
        ls::PID* angular_control;
    public:
        /**
         * @brief Construct a new Chassis object
         * 
         * @param _right the right motors
         * @param _left the left motors
         */
        explicit Chassis(pros::MotorGroup& _right, pros::MotorGroup& _left, ls::AbstractOdom& _odom, ls::PID& _lateral_control, ls::PID& _angular_control);
        
        /**
         * @brief equates this chassis structure to another one, by using the same motors.
         * 
         * @param other chassis to equate
         */
        void operator=(Chassis other);
        
        
        void moveToPoint(double X, double Y, unsigned int timeout, bool reverse = false);


        void turnToPoint(double X, double Y, unsigned int timeout);
    }; 
}



#endif // LS_CHASSIS_H