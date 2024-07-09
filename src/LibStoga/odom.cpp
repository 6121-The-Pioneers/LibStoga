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

	//// ========================================= TODO: =========================================
    // for Gavin to implement:
	
	double ThreeWheelOdom::getDeltaX()
    {
		return 2 * sin(deltaRadians / 2) * ((deltaB / deltaRadians) + center_to_back);
        // return 0.0;
    }

    double ThreeWheelOdom::getDeltaY()
    {
		return 2 * sin(deltaRadians / 2) * ((deltaR / deltaRadians) + center_to_right);
        // return 0.0;
    }

    Angle ThreeWheelOdom::getDeltaAngle()
    {
		double angleRadian = (deltaL - deltaR) / (center_to_right + center_to_left);
		double angleDegrees = angleRadian * 57.2958;
        return Angle(angleDegrees);
    }

	voic compute() {
		double deltaL = left.getLinearDeltaDistance();
		double deltaR = right.getLinearDeltaDistance();
		double deltaB = back.getLinearDeltaDistance();
		AbstractOdom::compute();

	}
};