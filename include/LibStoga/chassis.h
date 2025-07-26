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
        * Linear in movement. Does not support curves.
        * 
        * @param ... (figure it out once done with function)
        */
        void moveToPointLinear(double X, double Y, unsigned int timeout, bool reverse = false);

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