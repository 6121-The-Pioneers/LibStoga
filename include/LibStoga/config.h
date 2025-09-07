#pragma once
#include <memory>
#include <vector>

#include "pros/motors.hpp"
#include "odom.h"
#include "gains.h"
#include "pure_pursuit.h"

namespace ls {
    class Chassis; // Forward declaration


    // Movement completion and sensitivity thresholds for primitive motions
    struct ChassisTolerances {
        double lateralThresh{1};      // inches to consider drive complete
        double angularThresh{5};      // degrees to consider turn complete
        unsigned int dwellMs{300};    // time must remain inside threshold
    };

    struct DrivetrainConfig {
        std::vector<int8_t> leftMotors;
        std::vector<int8_t> rightMotors;
        double trackWidth;
        double wheelDiameter;
        double rpm;
    };

    struct OdomConfig {
        bool useThreeWheel;
        threewheel_odom_parameters_t threeWheelParams;
        imu_odom_parameters_t imuParams;
    };

    struct ChassisControllerConfig {
        PIDGains driveGains;
        PIDGains turnGains;
        ChassisTolerances tolerances;
        double turnSensitivity{1.0};
        bool enableAdaptivePID{false};  // Enable adaptive PID that adjusts to battery/surface/load
    };

    class ChassisBuilder {
    public:
        ChassisBuilder& withDrivetrain(const DrivetrainConfig& cfg);
        ChassisBuilder& withOdom(const OdomConfig& cfg);
        ChassisBuilder& withController(const ChassisControllerConfig& cfg);
        std::shared_ptr<Chassis> build();

    private:
        DrivetrainConfig drivetrainConfig;
        OdomConfig odomConfig;
        ChassisControllerConfig controllerConfig;
    };
}
