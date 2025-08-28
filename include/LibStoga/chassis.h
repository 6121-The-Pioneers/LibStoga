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

        ls::AbstractPID* lateral_control;
        ls::AbstractPID* angular_control;
        
        double turn_sensitivity;
        double threshold_lateral;
        double threshold_angular;
        unsigned int threshold_timeout;
    public:
        /**
         * @brief Construct a new Chassis object
         * 
         * @param _right the right motors
         * @param _left the left motors
         * @param _odom odom of the chassis
         * @param _lateral_control PID class denoting forward motion
         * @param _angular_control PID class denoting turn motion
         * @param _turn_sensitivity how much turns should be prioritized (1 is default, >1 is prioritize turn by magnitude, <1 is prioritize movement by magnitude (NOT RECOMMENDED), negatives do not matter, 0 is no lateral movement)
         * @param _threshold_lateral threshold at which if sitting for _threshold_timeout ms moveTo functions can exit
         * @param _threshold_angular threshold at which if sitting for _threshold_timeout ms turnTo functions can exit
         * @param _threshold_timeout amount of time time that must be spent within threshold to exit
         */
        explicit Chassis(pros::MotorGroup& _right, pros::MotorGroup& _left, ls::AbstractOdom& _odom, ls::AbstractPID& _lateral_control, ls::AbstractPID& _angular_control, double _turn_sensitivity = 1, double _threshold_lateral = 1, double _threshold_angular = 5, unsigned int _threshold_timeout = 500);
        
        /**
         * @brief equates this chassis structure to another one, by using the same motors.
         * 
         * @param other chassis to equate
         */
        void operator=(Chassis other);
        
        /** TODO
        * @brief moves chassis to the following point in the cartesian coordinate plane with latest odom reset point as (0, 0)
        * 
        * @param ... (figure it out once done with function)
        */
        void moveToPoint(double X, double Y, unsigned int timeout, bool reverse = false);

        /** TODO
        * @brief turns chassis to face the following point in the cartesian coordinate plane with latest odom reset point as (0, 0)
        * 
        * @param ... (figure it out once done with function)
        */
        void turnToPoint(double X, double Y, unsigned int timeout, bool reverse = false);

    private:
        
        /**
        * Calculate priority from [0, 1] as percentage of which the turn motion has priority over the move motion. Usefull for moveToPoint linear
        */
        double move_priority(Angle& angle) const;
    }; 
}



#endif // LS_CHASSIS_H