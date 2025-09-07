// Demonstrates usage of the new unified configuration & builder.
#include "main.h"
#include "LibStoga/libstoga.h"
#include "LibStoga/auton_tuner.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "LibStoga/system_health.h"
#include "LibStoga/data_logger.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Global shared pointers produced by the builder
static std::shared_ptr<ls::Chassis> gChassis;
static std::shared_ptr<ls::AbstractOdom> gOdom;

static void initializeSystems() {
    // Drivetrain Configuration
    ls::DrivetrainConfig drivetrain {
        .leftMotors = {-2},
        .rightMotors = {1},
        .trackWidth = 11.0, // Tune this value
        .wheelDiameter = 3.25, // Tune this value
        .rpm = 600
    };

    // Odometry Configuration
    ls::OdomConfig odom {
        .useThreeWheel = false,
        .imuParams = {
            {13, 2.75, true},   // Vertical tracking wheel - REVERSED
            5.5,               // Distance from tracking center to vertical wheel
            {3, 2, true},      // Horizontal tracking wheel - REVERSED
            5.5,               // Distance from tracking center to horizontal wheel
            20                 // imu port
        }
    };

    // Chassis Controller Configuration
    ls::ChassisControllerConfig controller {
        .driveGains = {4.f, 0.f, 0.f, 10.f, true, 1000.f},
        .turnGains = {1.f, 0.f, 0.f, 10.f, true, 1000.f},
        .tolerances = {1.0, 5.0, 300},
        .turnSensitivity = 1.0,
        .enableAdaptivePID = false  // Set to true to enable adaptive PID that adjusts to battery/surface/load changes
    };

    // Build the chassis
    gChassis = ls::ChassisBuilder()
        .withDrivetrain(drivetrain)
        .withOdom(odom)
        .withController(controller)
        .build();

    // The odom object is now owned by the chassis, but we can get a reference if needed
    gOdom = gChassis->getOdom();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	initializeSystems();
    gChassis->setController(master);
	if (gOdom) {
        gOdom->resetAll();
        std::cout << "Odometry initialized and reset" << std::endl;
    }

    // Initialize and start health monitoring
    auto& healthMonitor = ls::getHealthMonitor();
    healthMonitor.registerMotor(2, &gChassis->getLeftMotorVec()[0]);
    healthMonitor.registerMotor(1, &gChassis->getRightMotorVec()[0]);
    healthMonitor.startMonitoring();

    // Initialize data logger
    auto& dataLogger = ls::getDataLogger();
    if (dataLogger.isSDAvailable()) {
        std::cout << "SD card available. Data logging enabled." << std::endl;
    } else {
        std::cout << "SD card not found. Data logging disabled." << std::endl;
    }
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    // Example of saving a tuning profile
    auto& dataLogger = ls::getDataLogger();
    if (dataLogger.isSDAvailable()) {
        ls::DataLogger::TuningProfile profile;
        profile.name = "default";
        profile.driveGains = gChassis->getLateralControl()->getGains();
        profile.turnGains = gChassis->getAngularControl()->getGains();
        profile.timestamp = pros::millis();
        profile.notes = "Initial tuning from competition init.";
        dataLogger.saveTuningProfile(profile);
    }
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    if (!gChassis) return;

    // Example autonomous routine with adaptive PID updates
    // If adaptive PID is enabled, call updateAdaptivePID() periodically during motion
    std::cout << "Starting autonomous routine..." << std::endl;

    // Example movement with adaptive PID updates (if enabled)
    gChassis->driveRelative(24, 3000);
    if (gChassis->isAdaptivePIDEnabled()) {
        gChassis->updateAdaptivePID();
    }

    gChassis->turnToAngle(90, 2000);
    if (gChassis->isAdaptivePIDEnabled()) {
        gChassis->updateAdaptivePID();
    }

    std::cout << "Autonomous routine complete." << std::endl;
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
    if (!gChassis) return;
	while(true) {
		// Arcade drive using controller inputs
		gChassis->arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
		                 master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			ls::autoTune(gChassis);
		}

        // Health monitoring check
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            auto& healthMonitor = ls::getHealthMonitor();
            auto issues = healthMonitor.getIssues();
            if (issues.empty()) {
                std::cout << "System health: HEALTHY" << std::endl;
            } else {
                for (const auto& issue : issues) {
                    std::cout << "Health issue: " << issue.description << std::endl;
                }
            }
        }

		// Test odometry sensors without moving
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			std::cout << "Testing odometry sensors (don't move robot)..." << std::endl;
			gChassis->getOdom()->resetAll();
			for (int i = 0; i < 5; i++) {
				gChassis->getOdom()->compute();
				std::cout << "Pos: " << gChassis->getOdom()->getPosition() << std::endl;
				pros::delay(1000);
			}
			std::cout << "Odometry test complete - check if values change when stationary" << std::endl;
		}

		// Test tracking wheel deltas
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			std::cout << "Testing tracking wheel deltas..." << std::endl;
			gChassis->getOdom()->resetAll();
			pros::delay(100); // Let reset complete
			for (int i = 0; i < 10; i++) {
				gChassis->getOdom()->compute();
				std::cout << "Delta X: " << gChassis->getOdom()->getDeltaX() 
				         << " Delta Y: " << gChassis->getOdom()->getDeltaY() 
				         << " Delta Angle: " << gChassis->getOdom()->getDeltaAngle().getAngle() << std::endl;
				pros::delay(200);
			}
			std::cout << "Delta test complete" << std::endl;
		}

		// Example of a chassis movement
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			std::cout << "Starting driveRelative test 15in" << std::endl;
			std::cout << "[DEBUG] Before drive: pos=" << gChassis->getOdom()->getPosition() << "\n";
			
			// Test odometry by manually driving and checking readings
			gChassis->getOdom()->resetAll();
			gChassis->getOdom()->compute();
			std::cout << "[DEBUG] After reset: pos=" << gChassis->getOdom()->getPosition() << "\n";
			
			gChassis->driveRelative(15, 3000);
			std::cout << "[DEBUG] After drive: pos=" << gChassis->getOdom()->getPosition() << "\n";
			std::cout << "driveRelative complete" << std::endl;
		}

		pros::delay(20);
	}
}


