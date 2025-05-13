#include "main.h"
#include "LibStoga/libstoga.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <map>

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup right({19, 9, 20});
pros::MotorGroup left({-8, -14, -10});
ls::Chassis chassis{right, left};

ls::imu_odom_parameters_t odom_params = {
	{
		7,
		1.375,
		false
	},
	0,
	{
		17,
		1.375,
		false
	},
	3.5,
	15
};
ls::ImuOdom odom(odom_params);

ls::PID lateral_control (
	1,
	0,
	0,
	10,
	0
);
ls::PID turn_control (
	1,
	0,
	0,
	10,
	0
);

ls::BMA movement(odom, &lateral_control, &turn_control, &chassis, 1);

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






















	// ///// CLAWBOT SHIEZA (Dont worry about this)
	// pros::Motor left(1);
	// pros::Motor right(-2);
	// pros::Motor lift(3);
	// pros::Motor claw(8);

	// lift.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	// claw.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	// right.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	// left.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	// claw.move(127);
	// pros::delay(1000);
	// claw.move(0);
	// lift.move_absolute(300, 50);

	// for (int i = 0; i < 50; i++) {
	// 	left.move(i - 3);
	// 	right.move(i);
	// 	pros::delay(150);
	// }

	// // left.move(127/1 - 1);
	// // right.move(127/1);
	// // pros::delay(500);
	// pros::delay(1000);
	// left.move(50 - 4);
	// right.move(50);
	// pros::delay(6000);
	// left.move(50 - 2);
	// right.move(50);
	// pros::delay(21000 - 1000 - 6000);

	// left.move(0);
	// right.move(0);

	// claw.move(-127);
	// pros::delay(1000);
	// // claw.move(0);

	// lift.move(127);
	// pros::delay(2000);
	// lift.move(0);

	// right.move(-50);
	// left.move(50);
	// pros::delay(500);
	// right.move(0);
	// left.move(0);
	
	// right.move(50);
	// left.move(50);
	// pros::delay(500);
	// left.move(0);
	// right.move(0);
	
}


