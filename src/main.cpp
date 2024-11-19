#include "main.h"
#include "LibStoga/libstoga.h"
#include "pros/motors.hpp"
#include "pros/llemu.hpp"
#include <cmath>
#include "LibStoga/pure_pursuit.h"

ls::imu_odom_parameters_t odom_params = {
	{
		13,
		1.375,
		false
	},
	0,
	{
		12,
		1.375,
		false
	},
	2,
	5
};

pros::Imu imu(5);

ls::ImuOdom odom(odom_params);
pros::MotorGroup right({6, 7, 8});
pros::MotorGroup left({-20, -10, -9});
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup intake({1, -2});
pros::MotorGroup wallstake({69});

pros::ADIDigitalOut mogo('A');

// std::vector<ls::Point> path = {
// };

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	imu.reset(true);
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
void competition_initialize() {}

/**
 * @brief If all else fails, this auton will move straight to the ladder.
 * 
 */
void backup_autonomous_ladder_touch() {
	right.move(-127/2);
	left.move(-127/2);
	pros::delay(1500);
	right.move(0);
	left.move(0);
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
	backup_autonomous_ladder_touch();
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
	bool is_mogo_on = false;
	constexpr double TURN_SENSITIVITY = 0.85;
	constexpr int  SECOND_STAGE_SPEED = 100;

	right.set_brake_mode_all(MOTOR_BRAKE_COAST);
	left.set_brake_mode_all(MOTOR_BRAKE_COAST);
	intake.set_brake_mode(MOTOR_BRAKE_COAST);

	while (true) {
		int angle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		int power = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		right.move(power - 0.85 * angle);
		left.move(power + 0.85 * angle);

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
			is_mogo_on = !is_mogo_on;
		}
		mogo.set_value(is_mogo_on);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			intake.move(-SECOND_STAGE_SPEED);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intake.move(SECOND_STAGE_SPEED);
		} else {
			intake.move(0);
		}

		pros::delay(20);
	}
	
}