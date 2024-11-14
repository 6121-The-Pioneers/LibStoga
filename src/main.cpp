#include "main.h"
#include "LibStoga/libstoga.h"
#include "pros/motors.hpp"
#include "pros/llemu.hpp"
#include <cmath>

ls::imu_odom_parameters_t odom_params = {
	{
		20,
		2.75,
		false
	},
	0.49,
	{
		1,
		2.75,
		false
	},
	2.5,
	15
};

pros::Imu imu(15);
ls::ImuOdom odom(odom_params);

pros::MotorGroup right({10, 17, 18});
pros::MotorGroup left({-14, -13, -12});
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut mogo('A');
pros::MotorGroup intake({19, -11});

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	imu.reset(true);
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
	///// TODO: MAKE THIS FORMAT WORK:
	// chassis.moveTo(0, 0, 5000);
	// chassis.moveTo(38.109, -32.738, 5000);
	ls::Angle goal(90);
	ls::SmartPID spid(0.1, 90, 0.1, 127);

	right.set_brake_mode_all(MOTOR_BRAKE_COAST);
	left.set_brake_mode_all(MOTOR_BRAKE_COAST);
	intake.set_brake_mode(MOTOR_BRAKE_COAST);
	
	while (true)
	{
		odom.compute();
		ls::Angle current(odom.getAngle());
		ls::Angle difference = goal.minimumAngleDifference(current);

		double output = spid.update(difference.getAngle());
		right.move(-output);
		left.move(output);
	}

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

	right.set_brake_mode_all(MOTOR_BRAKE_COAST);
	left.set_brake_mode_all(MOTOR_BRAKE_COAST);
	intake.set_brake_mode(MOTOR_BRAKE_COAST);

	while (true) {
		odom.compute();
		int angle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		int power = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		right.move(power - angle);
		left.move(power + angle);

		mogo.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1));
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			intake.move(-127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intake.move(127);
		} else {
			intake.move(0);
		}

		pros::lcd::print(0, "(%f, %f), %f", odom.getX(), odom.getY(), odom.getAngle());

		pros::delay(20);
	}
	
}