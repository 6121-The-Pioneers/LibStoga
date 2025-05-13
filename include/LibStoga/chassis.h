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

namespace ls {
    /**
     * @brief Represents the chassis of the robot. Only supports tank drive configuration (TODO)
     * 
     */
    struct Chassis 
    {
        pros::MotorGroup* right;
        pros::MotorGroup* left;
        
        /**
         * @brief Construct a new Chassis object
         * 
         * @param _right the right motors
         * @param _left the left motors
         */
        explicit Chassis(pros::MotorGroup& _right, pros::MotorGroup& _left);
        
        /**
         * @brief equates this chassis structure to another one, by using the same motors.
         * 
         * @param other chassis to equate
         */
        void operator=(Chassis other);
    }; 
}



#endif // LS_CHASSIS_H