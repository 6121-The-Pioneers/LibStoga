#include "config.h"
#include "pid.h"
#include "odom.h"
#include "chassis.h"
#include "pure_pursuit.h"
#include "pros/motors.hpp"
#include <memory>

namespace ls {

    ChassisBuilder& ChassisBuilder::withDrivetrain(const DrivetrainConfig& cfg) {
        drivetrainConfig = cfg;
        return *this;
    }

    ChassisBuilder& ChassisBuilder::withOdom(const OdomConfig& cfg) {
        odomConfig = cfg;
        return *this;
    }

    ChassisBuilder& ChassisBuilder::withController(const ChassisControllerConfig& cfg) {
        controllerConfig = cfg;
        return *this;
    }

    std::shared_ptr<Chassis> ChassisBuilder::build() {
        // Motor groups
        auto leftGroup = std::make_shared<pros::MotorGroup>(drivetrainConfig.leftMotors);
        auto rightGroup = std::make_shared<pros::MotorGroup>(drivetrainConfig.rightMotors);

        // Odom selection
        std::shared_ptr<AbstractOdom> odom;
        if (odomConfig.useThreeWheel) {
            odom = std::make_shared<ThreeWheelOdom>(odomConfig.threeWheelParams);
        } else {
            odom = std::make_shared<ImuOdom>(odomConfig.imuParams);
        }

        // PIDs
        auto drivePID = std::make_shared<PID>(controllerConfig.driveGains);
        auto turnPID = std::make_shared<PID>(controllerConfig.turnGains);

        // Chassis Config
        ChassisConfig cc;
        cc.turnSensitivity = controllerConfig.turnSensitivity;
        cc.lateralThreshold = controllerConfig.tolerances.lateralThresh;
        cc.angularThreshold = controllerConfig.tolerances.angularThresh;
        cc.dwellTimeMs = controllerConfig.tolerances.dwellMs;
        cc.enableAdaptivePID = controllerConfig.enableAdaptivePID;

        // Create Chassis
        auto chassis = std::make_shared<Chassis>(rightGroup, leftGroup, odom, drivePID, turnPID, cc);
        
        // Note: PurePursuit is not part of the builder for now to keep it simple.
        // It can be constructed separately if needed.

        return chassis;
    }
}
