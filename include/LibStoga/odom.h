/*
* Contains the Abstract Odom class for the Odometry hierarchy
*/

#ifndef ODOM_ABSTRACT_LS_H
#define ODOM_ABSTRACT_LS_H

#include <initializer_list>
#include <exception>
#include <vector>
#include "tracking.h"
#include "geometry.h"
#include "api.h"

namespace ls {
	/*
	* A compact representation for x, y, and theta.
	* contains coordinates and helper methods accordingly.
	*/
	struct Position {
		double X;
		double Y;
		Angle theta; // by default in bearing mode (North is angle when robot starts).
		/*
		* Default constructor.
		*/
		explicit Position(): X(0), Y(0), theta(Angle(0)) {}
		/**
		* Constructs a position object in terms x, y, z.
		* 
		* @param x the x value.
		* @param y the y value.
		* @param theta the theta value (degrees & bearing).
		*/
		explicit Position(double x, double y, double t): X(x), Y(y), theta(t) {}
		/**
		* Returns the distance from this point to the given point.
		* Only returns a positive value.
		*
		* @param pos second point to compare to.
		* @return distance from point
		*/
		double distanceFromPoint(Position& pos) const;
		/**
		* Returns a signed value from this point to the other point.
		* 
		* will return a negative value if point is behind the robot.
		* check isbehind() for info about 'behind'
		*
		* @param pos second point to compare to.
		* @return signed distance from point.
		*/
		double distanceFromPointSigned(Position& pos) const;
		/**
		* returns if a point is behind (-1), in front of (1), or niether (0) in terms of the robot.
		* behind and front of check if a give point is in front of the robot or behind it.
		* 
		* determined by getting angle to point and comparing.
		* niether implies thats points are too close to eachother to determine. 
		* 
		* @param pos second point to compare to.
		* @returns behind (-1), in front of (1), or niether (0)
		*/
		int isBehind(Position& pos) const;
		/**
		 * @brief Gets the of the resulting line of this position to the other given position.
		 * Will return in terms of a bearing.
		 * Will return inf if the positions are way to close or equal to be calculated properly.
		 * 
		 * @param pos the other position.
		 * @return Angle in [0, 360)
		 */
		Angle angleToPosition(Position& pos) const;
		/**
		 * @brief Gets the of the resulting line of this position to the other given position.
		 * Will return in terms of a bearing, but will not be bounded from [0, 360)
		 * Will return inf if the positions are way to close or equal to be calculated properly.
		 * 
		 * @param pos the other position.
		 * @return Angle not bounded to [0, 360)
		 */
		Angle angleToPositionSigned(Position& pos) const;
	};

	class AbstractOdom {
	protected:
		Position pos;
		explicit AbstractOdom(): pos(Position()) {};
	public:
		/**
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
		/**
		* Computes the new coordinations and angle of the robot since the program has started.
		* Assumes that (0, 0) is where the robot starts at reboot.
		* Also assumes that initial posture is 0 degrees (bearing)
		* 
		* Must give X, Y, and angle new values after this call is over.
		* 
		* @throws std::bad_function_call if object has not been initialized properly.
		*/
		virtual void compute() = 0;
		/**
		* Gets the x-coordinate value of the robot since program started.
		* Assumes (0, 0) is when the robot started.
		* 
		* @returns the x-coordinate value
		* @throws std::bad_function_call if object is not initialized properly.
		*/
		virtual double getX();
		/**
		* Gets the y-coordinate value of the robot since program started.
		* Assumes (0, 0) is when the robot started.
		*
		* @returns the y-coordinate value
		* @throws std::bad_function_call if object is not initialized properly.
		*/
		virtual double getY();
		/**
		* Gets the bearing angle of the robot since program started.
		* Assumes 0 degrees is initial orientation when program starts.
		*
		* @returns the x-coordinate value
		* @throws std::bad_function_call if object is not initialized properly.
		*/
		virtual double getAngle();
		/**
		* Gets the Position of this odom object in terms of the Position object.
		*/
		virtual Position getPosition();
		/**
		* Reset the X coordinate of the robot.
		* Make the current position X = 0
		*/
		virtual void resetX() = 0;
		/**
		* Reset the Y coordinate of the robot.
		* Make the current position Y = 0
		*/
		virtual void resetY() = 0;
		/**
		* Reset the angle of the robot.
		* Make the current angle = 0
		*/
		virtual void resetAngle() = 0;
		/**
		* Reset the positioning and angles of the robot.
		* makes the current position (0, 0) and the current angle = 0;
		*/
		virtual void resetAll();
		/**
		* Gets the change in X since the previous call of this method.
		* Does not call resetX() or reset the coordinates in any way.
		* 
		* Will return 0 on first call.
		* 
		* @returns the delta in the x-coordinate value
		* @throws std::bad_function_call if object is not initialized properly.
		*/
		virtual double getDeltaX() = 0;
		/**
		* Gets the change in Y since the previous call of this method.
		* Does not call resetY() or reset the coordinates in any way.
		*
		* Will return 0 on first call.
		*
		* @returns the delta in the y-coordinate value
		* @throws std::bad_function_call if object is not initialized properly.
		*/
		virtual double getDeltaY() = 0;
		/**
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
};

#endif // !ODOM_ABSTRACT_LS_H
