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
		Angle angle = getDeltaAngle();
		pos.X += getDeltaX();
		pos.Y += getDeltaY();
		pos.theta += angle;
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

		if (a.getAngle() == infinity()) {
			return 0;
		}

		const Angle lower_bound = theta.getAngle() - 90;
		const Angle upper_bound = theta.getAngle() + 90;
		
		if (a.getAngle() > lower_bound.normalize() && a.getAngle() < upper_bound.normalize()) {
			return 1;
		} 
		else {
			return -1;
		}
	}

	Angle Position::angleToPosition(Position &pos) const
	{
		double raw_angle = atan2(pos.Y - Y, pos.X - X);
		return Angle(90 - (raw_angle > 0 ? raw_angle : (6.28318530718 + raw_angle)) * 57.2957795131);
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

	ThreeWheelOdom::ThreeWheelOdom(threewheel_odom_parameters_t& param) 
	{
		right = std::make_unique<TrackingWheel>(param.right.port, param.right.radius, param.right.reversed);
		left = std::make_unique<TrackingWheel>(param.left.port, param.left.radius, param.left.reversed);
		back = std::make_unique<TrackingWheel>(param.back.port, param.back.radius, param.back.reversed);
		deltaB = param.center_to_back;
		deltaL = param.center_to_left;
		deltaR = param.center_to_right;
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
		if (deltaT == 0) {
			return 0;
		} else {
			return 2 * sin(degreesToRadians(deltaT) / 2) * ((deltaB / degreesToRadians(deltaT)) + centerToBack);
		}
    }

    double ThreeWheelOdom::getDeltaY()
    {
		if (deltaT == 0) {
			return 0;
		}
		else {
			return 2 * sin(degreesToRadians(deltaT) / 2) * ((deltaR / degreesToRadians(deltaT)) + centerToRight);
		}
	}

    Angle ThreeWheelOdom::getDeltaAngle()
    {
		double angleRadian = (deltaL - deltaR) / (centerToRight + centerToLeft);
		deltaT = angleRadian * 57.2958; // convert to degrees
        return Angle(deltaT);
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
		: centerToHoriz(center_to_horiz), centerToVert(center_to_vert) {
		prevRotation = 0;
	}

    ImuOdom::ImuOdom(double center_to_horiz, double center_to_vert, TrackingWheel &h, TrackingWheel &v, pros::Imu &i)
		: centerToHoriz(center_to_horiz), centerToVert(center_to_vert)
    {
		horiz = std::make_unique<TrackingWheel>(h);
		vert = std::make_unique<TrackingWheel>(v);
		IMU = std::make_unique<pros::Imu>(i);
		prevRotation = 0;
    }

	ImuOdom::ImuOdom(imu_odom_parameters_t& param)
		: centerToHoriz(param.center_to_perpendicular), centerToVert(param.center_to_parallel)
	{
		horiz = std::make_unique<TrackingWheel>(param.perpendicular.port, param.perpendicular.radius, param.perpendicular.reversed);
		vert = std::make_unique<TrackingWheel>(param.parallel.port, param.parallel.radius, param.parallel.reversed);
		IMU = std::make_unique<pros::Imu>(param.imu_port);
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
	
	double ImuOdom::getDeltaX() // rework
    {
		return 0;
    }

    double ImuOdom::getDeltaY() // rework
    {
		return 0;
    }

    Angle ImuOdom::getDeltaAngle() // good
    {
		double curRotation = IMU->get_heading();
		prevRotation = curRotation;
        return Angle(curRotation == prevRotation ? 1e-6 : (curRotation - prevRotation));
    }

    void ImuOdom::compute() // rework
    {
		pos.theta = IMU->get_heading();
		Angle mathAngle = Angle(90 - pos.theta.getAngle()); // use non-polar angle for math
		Angle deltaAngle = getDeltaAngle();

		// calculate respective radiuses for odom math
		double r0 = (vert->getLinearDeltaDistance() - centerToVert * deltaAngle.convertToRadians()) / deltaAngle.convertToRadians();
		double r1 = (horiz->getLinearDeltaDistance() - centerToHoriz * deltaAngle.convertToRadians()) / deltaAngle.convertToRadians();
		
		// calculate relative coordinates:
		double relX = r0 * sin(deltaAngle.convertToRadians()) - r1 * (1 - cos(deltaAngle.convertToRadians()));
		double relY = r1 * sin(deltaAngle.convertToRadians()) + r0 * (1 - cos(deltaAngle.convertToRadians()));

		// rotate and increment to global coordinates:
		pos.X += relX * cos(mathAngle.convertToRadians()) - relY * sin(mathAngle.convertToRadians());
		pos.Y += relY * cos(mathAngle.convertToRadians()) + relX * sin(mathAngle.convertToRadians());
    }

    void ImuOdom::resetAll()
    {
		horiz->reset();
		vert->reset();
		IMU->reset(true);
		pos.X = 0;
		pos.Y = 0;
		pos.theta = 0;
		AbstractOdom::resetAll();
    }
};