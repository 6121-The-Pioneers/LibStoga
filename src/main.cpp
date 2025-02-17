#include "main.h"
#include "LibStoga/libstoga.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>
// #include "LibStoga/pure_pursuit.h"

ls::PID wallstakePID(1, 0, 0, 10, 0); // change numbers later
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

pros::Imu imu(5);
pros::Optical racism(3);
ls::ImuOdom odom(odom_params);
pros::MotorGroup right({19, 9, 20});
pros::MotorGroup left({-8, -14, -10});
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup intake({18, -6});
pros::MotorGroup wallstake({-16});
pros::adi::DigitalOut mogo('H');

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
	racism.set_led_pwm(100);
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
void competition_initialize() {

}

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
	// backup_autonomous_ladder_touch();
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
	pros::lcd::print(0, "Loading...");
	bool is_mogo_on = false;
	constexpr double TURN_SENSITIVITY = 0.85;
	constexpr int  SECOND_STAGE_SPEED = 105;

	right.set_brake_mode_all(MOTOR_BRAKE_COAST);
	left.set_brake_mode_all(MOTOR_BRAKE_COAST);
	intake.set_brake_mode(MOTOR_BRAKE_COAST);

	int goal_wallstake_angle = 0;
	wallstake.set_encoder_units_all(pros::MotorEncoderUnits::degrees);

	while (true) {
		odom.compute();
		int angle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		int power = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		right.move(power - angle);
		left.move(power + angle);

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

		if (racism.get_hue() >= 200.0 && racism.get_hue() <= 220.0) {
			pros::delay(100);
			intake.move(0);
			pros::delay(500);
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			goal_wallstake_angle = 0;
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			goal_wallstake_angle = 60;
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			goal_wallstake_angle = 400;
		}

		// set PID for wallstake
		int wallstake_pid_output = wallstakePID.update(goal_wallstake_angle - wallstake.get_position(0));
		wallstake.move(wallstake_pid_output);

		pros::lcd::print(0, "%f", odom.getX());
		pros::lcd::print(1, "%f", odom.getY());
		pros::lcd::print(2, "%f", odom.getAngle());

		pros::delay(5);
	}
	
}