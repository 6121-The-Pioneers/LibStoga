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
#include "auton_tuner.h"
#include <memory>

#include "pros/misc.hpp" // For controller button enums

namespace ls {
    struct ChassisConfig {
        double turnSensitivity{1.0};
        double lateralThreshold{1.0};
        double angularThreshold{5.0};
        unsigned int dwellTimeMs{500};
        bool enableCancel{true};
        pros::controller_digital_e_t cancelButton{pros::E_CONTROLLER_DIGITAL_A};
        bool enableAdaptivePID{false};
    };
    /**
     * @brief Represents the chassis of the robot. Only supports tank drive configuration (TODO)
     * 
     */
    class Chassis 
    {
    private:
        std::shared_ptr<pros::MotorGroup> right;
        std::shared_ptr<pros::MotorGroup> left;
        std::shared_ptr<ls::AbstractOdom> odom;

        std::shared_ptr<ls::AbstractPID> lateral_control;
        std::shared_ptr<ls::AbstractPID> angular_control;
        
        double turn_sensitivity;
        double threshold_lateral;
        double threshold_angular;
        unsigned int threshold_timeout;
        bool enable_cancel;
        pros::controller_digital_e_t cancel_button;
        bool enable_adaptive_pid;
        pros::Controller* controller;

        // Adaptive PID controllers
        std::unique_ptr<AdaptivePIDController> lateral_adaptive_controller;
        std::unique_ptr<AdaptivePIDController> angular_adaptive_controller;
    public:
        /**
         * @brief Set the Controller object
         * 
         * @param cont The controller to use for cancellation
         */
        void setController(pros::Controller& cont);

        /**
         * @brief Get the Controller object
         * 
         * @return pros::Controller* 
         */
        pros::Controller* getController() { return controller; }

        /**
         * @brief Get a pointer to the odom system
         * 
         * @return ls::AbstractOdom* 
         */
        std::shared_ptr<ls::AbstractOdom> getOdom();
        
        /**
         * @brief Get the Lateral Control object
         * 
         * @return std::shared_ptr<AbstractPID> 
         */
        std::shared_ptr<AbstractPID> getLateralControl();

        /**
         * @brief Get the Angular Control object
         * 
         * @return std::shared_ptr<AbstractPID> 
         */
        std::shared_ptr<AbstractPID> getAngularControl();

        /**
         * @brief Update adaptive PID controllers if enabled
         * Should be called periodically during autonomous operation
         */
        void updateAdaptivePID();

        /**
         * @brief Construct a new Chassis object
         * 
         * @param _right the right motors
         * @param _left the left motors
         * @param _odom odom of the chassis
         * @param _lateral_control PID class denoting forward motion
         * @param _angular_control PID class denoting turn motion
         * @param cfg Configuration struct
         */
    explicit Chassis(std::shared_ptr<pros::MotorGroup> _right, std::shared_ptr<pros::MotorGroup> _left, std::shared_ptr<ls::AbstractOdom> _odom, std::shared_ptr<ls::AbstractPID> _lateral_control, std::shared_ptr<ls::AbstractPID> _angular_control, const ChassisConfig& cfg = {});
        
        Chassis(const Chassis&) = default;
        Chassis& operator=(const Chassis&) = default;
        Chassis(Chassis&&) = default;
        Chassis& operator=(Chassis&&) = default;
        
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

        /**
         * @brief Turns the chassis to a specific angle.
         * 
         * @param angle The target angle in degrees.
         * @param timeout The maximum time to complete the turn.
         */
        double turnToAngle(double angle, unsigned int timeout);

    /**
     * @brief Drive forward a relative distance from the current pose.
     * Positive distance drives forward; negative will target a point behind (robot may rotate to reach it).
     * @param distance inches to drive (forward +)
     * @param timeout ms timeout
     */
    double driveRelative(double distance, unsigned int timeout);

    /**
     * @brief Turn by a relative delta angle (bearing frame).
     * @param deltaAngle angle change in degrees (positive clockwise)
     * @param timeout ms timeout
     */
    double turnRelative(double deltaAngle, unsigned int timeout);

        /**
         * @brief Controls the chassis with tank drive logic.
         * 
         * @param left_y The input for the left side motors.
         * @param right_y The input for the right side motors.
         */
        void tank_drive(double left_y, double right_y);

        /**
         * @brief Controls the chassis with arcade drive logic.
         * 
         * @param forward The input for forward/backward movement.
         * @param turn The input for turning movement.
         */
        void arcade(double forward, double turn);

    /**
     * @brief Follow a path using Pure Pursuit controller.
     * @param path list of waypoints (inches)
     * @param lookahead lookahead distance (inches)
     * @param timeout ms timeout
     */
    void followPath(const Path& path, double lookahead, unsigned int timeout);

    private:
        
        /**
        * Calculate priority from [0, 1] as percentage of which the turn motion has priority over the move motion. Usefull for moveToPoint linear
        */
        double move_priority(Angle& angle) const;
    }; 
}



#endif // LS_CHASSIS_H