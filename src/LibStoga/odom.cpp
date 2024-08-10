#include "odom.h"
#include "tracking.h"

/**
 * @brief Contains all source code for abstract classes or composed ones
 * Basically code that powers abstraction.
 * 
 */
namespace ls {
    void AbstractOdom::compute()
    {
		pos.X += getDeltaX();
		pos.Y += getDeltaY();
		pos.theta += getDeltaAngle();
    }

    double AbstractOdom::getX()
	{
		return pos.X;
	}

	double AbstractOdom::getY()
	{
		return pos.Y;
	}

	double AbstractOdom::getAngle()
	{
		return pos.theta.getAngle();
	}

	Position AbstractOdom::getPosition()
	{
		return pos;
	}

	void AbstractOdom::resetX()
	{
		pos.X = 0;
	}

	void AbstractOdom::resetY()
	{
		pos.Y = 0;
	}

	void AbstractOdom::resetAngle()
	{
		pos.theta = 0;
	}

	void AbstractOdom::resetAll()
	{
		resetX();
		resetY();
		resetAngle();
	}

	double Position::distanceFromPoint(Position &pos) const
	{
		const double dx = pos.X - X;
		const double dy = pos.Y - Y;
		return sqrt(dx*dx + dy*dy);
	}

	double Position::distanceFromPointSigned(Position &pos) const
	{
		return isBehind(pos) * distanceFromPoint(pos);
	}

	int Position::isBehind(Position &pos) const
	{
		Angle a = angleToPosition(pos);
		const Angle lower_bound = theta.getAngle() - 90;
		const Angle upper_bound = theta.getAngle() + 90;

		if (a.getAngle() == infinity()) {
			return 0;
		}
		else if (a.getAngle() > lower_bound.normalize() && a.getAngle() < upper_bound.normalize()) {
			return 1;
		} 
		else {
			return -1;
		}
	}

	Angle Position::angleToPosition(Position &pos) const
	{
		Angle tor = (90 - atan2(pos.Y-Y, pos.X-X) * 57.2957795131);
		tor = tor.normalize();
		return tor;
	}

	Angle Position::angleToPositionSigned(Position &pos) const
	{
		return (90 - atan2(pos.Y-Y, pos.X-X) * 57.2957795131);
	}
};

/**
 * @brief Code for ThreeWheelOdom class.
 * 
 */
namespace ls {
	ThreeWheelOdom::ThreeWheelOdom(double center_to_right, double center_to_left, double center_to_back)
		: centerToRight(center_to_right), centerToLeft(center_to_left), centerToBack(center_to_back) {}

    ThreeWheelOdom::ThreeWheelOdom(double center_to_right, double center_to_left, double center_to_back, TrackingWheel &r, TrackingWheel &l, TrackingWheel &b)
		: centerToRight(center_to_right), centerToLeft(center_to_left), centerToBack(center_to_back)
    {
		right = std::make_unique<TrackingWheel>(r);
		left = std::make_unique<TrackingWheel>(l);
		back = std::make_unique<TrackingWheel>(b);
		deltaB = 0;
		deltaL = 0;
		deltaR = 0;
    }
	void ThreeWheelOdom::initialize(std::initializer_list<uint8_t> ports)
    {
		if (ports.size() != 3) {
			throw std::invalid_argument("initializer list must only have 3 elements (right, left, back).");
		}
		int index = 0;
		for (uint8_t i : ports) {
			if (abs(i) > 24) {
				throw std::invalid_argument("ports must be in between [-24, 0) U (0, 24].");
			}

			if (index == 0) {
				right = std::make_unique<TrackingWheel>(i);
			} else if (index == 1) {
				left = std::make_unique<TrackingWheel>(i);
			} else {
				back = std::make_unique<TrackingWheel>(i);
			}
		}
		
    }

	double ThreeWheelOdom::getDeltaX()
    {
		return 2 * sin(getDeltaAngle().convertToRadians() / 2) * ((deltaB / getDeltaAngle().convertToRadians()) + centerToBack);
        // return 0.0 if no movement
    }

    double ThreeWheelOdom::getDeltaY()
    {
		return 2 * sin(getDeltaAngle().convertToRadians() / 2) * ((deltaR / getDeltaAngle().convertToRadians()) + centerToRight);
        // return 0.0 if no movement
    }

    Angle ThreeWheelOdom::getDeltaAngle()
    {
		double angleRadian = (deltaL - deltaR) / (centerToRight + centerToLeft);
		double angleDegrees = angleRadian * 57.2958; // convert to degrees
        return Angle(angleDegrees);
    }

    void ThreeWheelOdom::compute()
    {
		deltaL = left.get()->getLinearDeltaDistance();
		deltaR = right.get()->getLinearDeltaDistance();
		deltaB = back.get()->getLinearDeltaDistance();
		AbstractOdom::compute();
    }
	
};

/**
 * @brief Code for ImuOdom class.
 * 
 */
namespace ls {
	ImuOdom::ImuOdom(double center_to_horiz, double center_to_vert)
		: centerToHoriz(center_to_horiz), centerToVert(center_to_vert) {}

    ImuOdom::ImuOdom(double center_to_horiz, double center_to_vert, TrackingWheel &h, TrackingWheel &v, pros::Imu &i)
		: centerToHoriz(center_to_horiz), centerToVert(center_to_vert)
    {
		horiz = std::make_unique<TrackingWheel>(h);
		vert = std::make_unique<TrackingWheel>(v);
		IMU = std::make_unique<pros::Imu>(i);
		deltaH = 0;
		deltaV = 0;
		prevRotation = 0;
    }

	void ImuOdom::initialize(std::initializer_list<uint8_t> ports)
    {
		if (ports.size() != 3) {
			throw std::invalid_argument("initializer list must only have 3 elements (horiz, vert, IMU).");
		}
		int index = 0;
		for (uint8_t i : ports) {
			if (abs(i) > 24) {
				throw std::invalid_argument("ports must be in between [-24, 0) U (0, 24].");
			}

			if (index == 0) {
				horiz = std::make_unique<TrackingWheel>(i);
			} else if (index == 1) {
				vert = std::make_unique<TrackingWheel>(i);
			} else {
				IMU = std::make_unique<pros::Imu>(i);
			}
		}	
    } 

	double ImuOdom::getDeltaX()
    {
    	return 2 * sin(getDeltaAngle().convertToRadians() / 2) * ((deltaH / getDeltaAngle().convertToRadians()) + centerToHoriz);
    }

    double ImuOdom::getDeltaY()
    {
		return 2 * sin(getDeltaAngle().convertToRadians() / 2) * ((deltaV / getDeltaAngle().convertToRadians()) + centerToVert);
    }

    Angle ImuOdom::getDeltaAngle()
    {
		double curRotation = IMU.get()->get_rotation();
		double deltaRotation = curRotation - prevRotation;
		Angle angleDegrees = Angle(deltaRotation) ; // convert to Angle
		prevRotation = curRotation;
        return Angle(deltaRotation);
    }

    void ImuOdom::compute()
    {
		deltaH = horiz.get()->getLinearDeltaDistance();
		deltaV = vert.get()->getLinearDeltaDistance();
		AbstractOdom::compute();
    }
	
};