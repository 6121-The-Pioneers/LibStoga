/*
* Contains the Abstract Odom class for the Odometry hierarchy
*/

#ifndef ODOM_ABSTRACT_LS_H
#define ODOM_ABSTRACT_LS_H

#include <initializer_list>
#include "api.h"

namespace ls {
	class AbstractOdom {
	protected:
		double X;
		double Y;
		double angle;

		explicit AbstractOdom(): X(0), Y(0), angle(0) {};
	public:
		/*
		* Initialize this AbstractOdom with a list of pins. 
		* Depending on the specific subclass, the meaning of this and the specific order of the pins may be different.
		* 
		* Ex. The 3 Wheel Odom class can have an instance where the 1st one is center, 2nd is right, 3rd is left
		*     But for 2 Wheel Odom + IMU, it might have an instance where 1st one is IMU, 2nd is perpendicular, 3rd is parralel.
		* 
		* If items on ports have to be reversed, it can be given as a negative number
		* 
		* @param ports the ports that this Odom object uses
		*/
		virtual void initialize(std::initializer_list<int8_t> ports) = 0;
		/*
		* Computes the new coordinations and angle of the robot since the program has started.
		* Assumes that (0, 0) is where the robot starts at reboot.
		* Also assumes that initial posture is 0 degrees (bearing)
		* 
		* Must give X, Y, and angle new values after this call is over.
		* 
		* @throws std::bad_function_call if object has not been initialized properly.
		*/
		virtual void compute() = 0;
		/*
		* Gets the x-coordinate value of the robot since program started.
		* Assumes (0, 0) is when the robot started.
		* 
		* @returns the x-coordinate value
		* @throws std::bad_function_call if object is not initialized properly.
		*/
		virtual double getX();
		/*
		* Gets the y-coordinate value of the robot since program started.
		* Assumes (0, 0) is when the robot started.
		*
		* @returns the y-coordinate value
		* @throws std::bad_function_call if object is not initialized properly.
		*/
		virtual double getY();
		/*
		* Gets the bearing angle of the robot since program started.
		* Assumes 0 degrees is initial orientation where .
		*
		* @returns the x-coordinate value
		* @throws std::bad_function_call if object is not initialized properly.
		*/
		virtual double getAngle();
		/*
		* Reset the X coordinate of the robot.
		* Make the current position X = 0
		*/
		virtual void resetX() = 0;
		/*
		* Reset the Y coordinate of the robot.
		* Make the current position Y = 0
		*/
		virtual void resetY() = 0;
		/*
		* Reset the angle of the robot.
		* Make the current angle = 0
		*/
		virtual void resetAngle() = 0;
		/*
		* Reset the positioning and angles of the robot.
		* makes the current position (0, 0) and the current angle = 0;
		*/
		virtual void resetAll();
		/*
		* Gets the change in X since the previous call of this method.
		* Does not call resetX() or reset the coordinates in any way.
		* 
		* Will return 0 on first call.
		* 
		* @returns the delta in the x-coordinate value
		* @throws std::bad_function_call if object is not initialized properly.
		*/
		virtual double getDeltaX() = 0;
		/*
		* Gets the change in Y since the previous call of this method.
		* Does not call resetY() or reset the coordinates in any way.
		*
		* Will return 0 on first call.
		*
		* @returns the delta in the y-coordinate value
		* @throws std::bad_function_call if object is not initialized properly.
		*/
		virtual double getDeltaY() = 0;
		/*
		* Gets the change in angle since the previous call of this method.
		* Does not call resetAngle() or reset the angle in any way.
		*
		* Will return 0 on first call.
		*
		* @returns the delta in the angle
		* @throws std::bad_function_call if object is not initialized properly.
		*/
		virtual double getDeltaAngle() = 0;
	};

	class ThreeWheelOdom: public AbstractOdom {
	private:
		double prevR;
		double prevL;
		double prevC;

		pros::Rotation right;
		pros::Rotation left;
		pros::Rotation center;
	public:
		explicit ThreeWheelOdom();

		explicit ThreeWheelOdom(std::initializer_list<int8_t> ports);

		void initialize(std::initializer_list<int8_t> ports) override;

		void compute() override;

		double getX() override;

		double getY() override;

		double getAngle() override;

		void resetX() override;

		void resetY() override;
		
		void resetAngle() override;

		double getDeltaX() override;
		
		double getDeltaY() override;
		
		double getDeltaAngle() override;
	};
};

#endif // !ODOM_ABSTRACT_LS_H
