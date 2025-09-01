#include "main.h"
#include "LibStoga/libstoga.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <map>

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup right({1, 9});
pros::MotorGroup left({-2, -6});

ls::imu_odom_parameters_t odom_params = {
	{
		13,
		1.375,
		true
	},
	0,
	{
		3,
		1.375,
		false
	},
	0,
	20
};
ls::ImuOdom odom(odom_params);

//ls::PID lateral_control (
//	4,
//	0,
//	0,
//	10,
//	0
//);
//ls::PID turn_control (
//	1,
//	0,
//	0,
//	10,
//	0
//);
ls::SmartPID lateral_control(1);
ls::SmartPID turn_control(0.5);
ls::Chassis chassis(right, left, odom, lateral_control, turn_control, 1);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	odom.resetAll();
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
	// touch ladder and call it a match.
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
	// relative coordinate PID testing (reverse parrallel odom to fix jittering if any)
	/*ls::SmartPID pid(1);
	
	while (true) {
		odom.compute();
		double output = pid.update(20 - odom.getY());
		right.move(output);
		left.move(output);
	
		pros::lcd::print(0, "X: %f Y: %f angle: %f", odom.getX(), odom.getY(), odom.getAngle());
		pros::delay(10);
	}*/

	chassis.moveToPoint(0, -40, 10000, true);

	master.print(0, 0, "DONE");

}


