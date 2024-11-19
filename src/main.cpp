#include "main.h"
#include "LibStoga/libstoga.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>
// #include "LibStoga/pure_pursuit.h"

ls::imu_odom_parameters_t odom_params = {
	{
		20,
		1.375,
		false
	},
	99999999,
	{
		1,
		1.375,
		false
	},
	100000,
	15
};

pros::Imu imu(15);
ls::ImuOdom odom(odom_params);
pros::MotorGroup right({6, 7, 8});
pros::MotorGroup left({-20, -10, -9});
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::adi::DigitalOut mogo('A');
pros::MotorGroup intake({1, -2});
pros::MotorGroup wallstake({69});
// ls::HighStakesRobot robot(right, left, intake, wallstake, mogo);

// std::vector<ls::Point> path = {

// };

// ls::PurePursuit persuit(path, odom, robot, 10, 2, 1, 127);

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
	// ///// TODO: MAKE THIS FORMAT WORK:
	// // chassis.moveTo(0, 0, 5000);
	// // chassis.moveTo(38.109, -32.738, 5000);
	// ls::Angle goal(90);
	// ls::SmartPID spid(0.01, 4, 0.0024, 40);

	// right.set_brake_mode_all(MOTOR_BRAKE_COAST);
	// left.set_brake_mode_all(MOTOR_BRAKE_COAST);
	// intake.set_brake_mode(MOTOR_BRAKE_COAST);

	// while (true)
	// {
	// 	odom.compute();
	// 	ls::Angle current(odom.getAngle());
	// 	ls::Angle difference = goal.minimumAngleDifference(current);

	// 	double output = spid.update(difference.getAngle());

	// 	master.print(0, 0, "%f", odom.getAngle());
	// 	right.move(-output);
	// 	left.move(output);
	// 	pros::delay(1);
	// }
	// persuit.move();
	right.move(-127/2);
	left.move(-127/2);
	pros::delay(2000);
	right.move(0);
	left.move(0);
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
	unsigned long jam_time = 0;
	bool is_jammed = false;
	const unsigned long JAM_LIMIT = 100;
	bool is_mogo_on = false;	

	right.set_brake_mode_all(MOTOR_BRAKE_COAST);
	left.set_brake_mode_all(MOTOR_BRAKE_COAST);
	intake.set_brake_mode(MOTOR_BRAKE_COAST);

	while (true) {
		odom.compute();
		int angle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		int power = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		right.move(power - 0.85 * angle);
		left.move(power + 0.85 * angle);

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
			is_mogo_on = !is_mogo_on;
		}
		mogo.set_value(is_mogo_on);


		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			intake.move(-127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intake.move(127);
		} else {
			intake.move(0);
		}

		pros::lcd::print(0, "%i", (int)is_mogo_on);

		pros::delay(20);
	}
	
}