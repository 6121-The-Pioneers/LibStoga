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

	void AbstractOdom::resetAngle(bool block_exec)
	{
		pos.theta = 0;
	}

	void AbstractOdom::resetAll(bool block_exec)
	{
		resetX();
		resetY();
		resetAngle(block_exec);
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
		prev_pos.X = 0; prev_pos.Y = 0; prev_pos.theta = Angle(0);
	};

    ImuOdom::ImuOdom(double center_to_horiz, double center_to_vert, TrackingWheel &h, TrackingWheel &v, pros::Imu &i)
		: centerToHoriz(center_to_horiz), centerToVert(center_to_vert)
    {
		horiz = std::make_unique<TrackingWheel>(h);
		vert = std::make_unique<TrackingWheel>(v);
		IMU = std::make_unique<pros::Imu>(i);
		deltaH = 0;
		deltaV = 0;
		prev_pos.X = 0; prev_pos.Y = 0; prev_pos.theta = Angle(0);
    }

	ImuOdom::ImuOdom(imu_odom_parameters_t& param) 
	{
		horiz = std::make_unique<TrackingWheel>(param.perpendicular.port, param.perpendicular.radius, param.perpendicular.reversed);
		vert = std::make_unique<TrackingWheel>(param.parallel.port, param.parallel.radius, param.parallel.reversed);
		IMU = std::make_unique<pros::Imu>(param.imu_port);
		deltaH = param.center_to_perpendicular;
		deltaV = param.center_to_parallel;
		prev_pos.X = 0; prev_pos.Y = 0; prev_pos.theta = Angle(0);
	}

	double ImuOdom::unwrapHeading(double cur, double prev) {
		double diff = cur - prev;
		if (diff > 180.0)  diff -= 360.0;   // crossed 360 -> 0
		if (diff < -180.0) diff += 360.0;   // crossed 0 -> 360
		return diff;
	}
	
	double ImuOdom::getDeltaX()
	{
		return pos.X - prev_pos.X;
	}

    double ImuOdom::getDeltaY()
	{
		return pos.Y - prev_pos.Y;
	}

    Angle ImuOdom::getDeltaAngle()
	{
		double curHeading = IMU->get_heading();          // 0–360
		double dHeading = unwrapHeading(curHeading, prevRotation);
		prevRotation = curHeading;
		deltaT = dHeading;
		return Angle(dHeading);
	}

    void ImuOdom::compute()
	{
		// save last pos for deltaX/deltaY queries
		prev_pos = pos;

		// wheel deltas
		double dSv = vert->getLinearDeltaDistance();
		double dSh = horiz->getLinearDeltaDistance();

		// IMU delta (degrees -> radians)
		double curHeading = IMU->get_heading();
		double dHeading = unwrapHeading(curHeading, prevRotation);
		prevRotation = curHeading;
		double dTheta = degreesToRadians(dHeading);

		// orientation at motion midpoint
		double theta_mid = pos.theta.convertToRadians() + dTheta * 0.5;

		// wheel offset corrections
		double dSf = dSv - centerToVert * dTheta;
		double dSl = dSh + centerToHoriz * dTheta;

		// local -> global transform
		double dX = dSl * cos(theta_mid) + dSf * sin(theta_mid);
		double dY = dSf * cos(theta_mid) - dSl * sin(theta_mid);

		// update global position
		pos.X += dX;
		pos.Y += dY;
		pos.theta += Angle(dHeading); // Angle wrapper expects degrees
	}

	void ImuOdom::resetAll(bool block_exec) {
		horiz->reset();
		vert->reset();
		resetAngle(block_exec);
		AbstractOdom::resetAll();
		prevRotation = 0.0;
		deltaH = 0.0;
		deltaV = 0.0;
		deltaT = 0.0;
	}

	void ImuOdom::resetAngle(bool block_exec) {
		IMU->reset(block_exec);
		AbstractOdom::resetAngle();
	}
};

