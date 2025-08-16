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

	ImuOdom::ImuOdom(imu_odom_parameters_t& param) 
	{
		horiz = std::make_unique<TrackingWheel>(param.perpendicular.port, param.perpendicular.radius, param.perpendicular.reversed);
		vert = std::make_unique<TrackingWheel>(param.parallel.port, param.parallel.radius, param.parallel.reversed);
		IMU = std::make_unique<pros::Imu>(param.imu_port);
		deltaH = param.center_to_perpendicular;
		deltaV = param.center_to_parallel;
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
		// if (deltaT == 0) {
		// 	return 0;	
		// }
		// else {
		// 	return 2 * sin(degreesToRadians(deltaT) / 2.0) * ((deltaH / degreesToRadians(deltaT)) + centerToHoriz);
		// }
		return horiz->getLinearDeltaDistance() * sin(degreesToRadians(IMU->get_heading())) - centerToHoriz * deltaT;
    }

    double ImuOdom::getDeltaY() // rework
    {
		// if (deltaT == 0) {
		// 	return 0;
		// } else {
		// 	return 2 * sin(degreesToRadians(deltaT) / 2.0) * ((deltaV / degreesToRadians(deltaT)) + centerToVert);
		// }
		return vert->getLinearDeltaDistance() * cos(degreesToRadians(IMU->get_heading())) - centerToVert * deltaT;
    }

    Angle ImuOdom::getDeltaAngle() // good
    {
		double curRotation = IMU->get_heading();
		deltaT = curRotation - prevRotation;
		prevRotation = curRotation;
        return Angle(deltaT);
    }

    void ImuOdom::compute() // rework
    {
		// deltaH = horiz.get()->getLinearDeltaDistance();
		// deltaV = vert.get()->getLinearDeltaDistance();
		// AbstractOdom::compute();
		pos.theta += getDeltaAngle();
		pos.X += getDeltaX();
		pos.Y += getDeltaY();
		
    }

    void ImuOdom::resetAll()
    {
		horiz->reset();
		vert->reset();
		IMU->reset(true);
		AbstractOdom::resetAll();
    }
};




/*

// Chat GPT Suggested Code for better Odom, implement this later:

#include "pros/rtos.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include <atomic>
#include <cmath>
#include <functional>
#include <mutex>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Odom {
public:
  struct Config {
	double wheel_diam_in = 2.75;
	double wheel_gear_ratio = 1.0;
	double rv = 0.0;
	double rh = 0.0;
	int period_ms = 10;
  };

  Odom(pros::Rotation& vert_rot, pros::Rotation& horiz_rot, pros::Imu& imu, const Config& cfg = {})
	: vert_reader_([&] { return rotationToLinear(vert_rot, cfg_); }),
	  horiz_reader_([&] { return rotationToLinear(horiz_rot, cfg_); }),
	  imu_(&imu), cfg_(cfg) {}

  Odom(std::function<double()> vertical_distance_supplier,
	   std::function<double()> horizontal_distance_supplier,
	   pros::Imu& imu, const Config& cfg = {})
	: vert_reader_(std::move(vertical_distance_supplier)),
	  horiz_reader_(std::move(horizontal_distance_supplier)),
	  imu_(&imu), cfg_(cfg) {}

  void reset(double x_in = 0.0, double y_in = 0.0) {
	std::scoped_lock lk(mtx_);
	x_ = x_in;
	y_ = y_in;
	double h = safeImuHeadingLocked();
	theta_rad_ = deg2rad(90.0 - h);
	last_bearing_deg_ = h;
	last_vert_ = vert_reader_ ? vert_reader_() : 0.0;
	last_horiz_ = horiz_reader_ ? horiz_reader_() : 0.0;
  }

  void start() {
	if (running_) return;
	running_ = true;
	task_ = std::make_unique<pros::Task>([this] { this->loop_(); });
  }

  void stop() {
	running_ = false;
	if (task_) { task_->remove(); task_.reset(); }
  }

  double x() const { std::scoped_lock lk(mtx_); return x_; }
  double y() const { std::scoped_lock lk(mtx_); return y_; }
  double heading_bearing_deg() const {
	std::scoped_lock lk(mtx_);
	return wrapDeg360(90.0 - rad2deg(theta_rad_));
  }

private:
  std::function<double()> vert_reader_;
  std::function<double()> horiz_reader_;
  pros::Imu* imu_ = nullptr;
  Config cfg_{};
  mutable std::mutex mtx_;
  double x_ = 0.0, y_ = 0.0;
  double theta_rad_ = 0.0;
  double last_vert_ = 0.0;
  double last_horiz_ = 0.0;
  double last_bearing_deg_ = 0.0;
  std::unique_ptr<pros::Task> task_;
  std::atomic<bool> running_{false};

  static double deg2rad(double d) { return d * M_PI / 180.0; }
  static double rad2deg(double r) { return r * 180.0 / M_PI; }

  static double rotationToLinear(pros::Rotation& rot, const Config& cfg) {
	double deg = rot.get_position();
	double revs = (deg / 360.0) / cfg.wheel_gear_ratio;
	double circumference = M_PI * cfg.wheel_diam_in;
	return revs * circumference;
  }

  static double wrapDeg180(double d) {
	while (d <= -180.0) d += 360.0;
	while (d > 180.0) d -= 360.0;
	return d;
  }

  static double wrapDeg360(double d) {
	while (d < 0.0) d += 360.0;
	while (d >= 360.0) d -= 360.0;
	return d;
  }

  bool imuHealthy() const { return imu_ && imu_->is_installed() && !imu_->is_calibrating(); }

  double safeImuHeadingLocked() const {
	if (imuHealthy()) {
	  double h = imu_->get_heading();
	  if (!std::isnan(h) && h >= 0.0 && h < 360.0) return h;
	}
	return last_bearing_deg_;
  }

  void loop_() {
	reset(x(), y());
	while (running_) {
	  double vert_now = vert_reader_ ? vert_reader_() : 0.0;
	  double horiz_now = horiz_reader_ ? horiz_reader_() : 0.0;
	  double dSv_meas = vert_now - last_vert_;
	  double dSh_meas = horiz_now - last_horiz_;
	  last_vert_ = vert_now;
	  last_horiz_ = horiz_now;

	  double dTheta = 0.0;
	  if (imuHealthy()) {
		double bearing_now = imu_->get_heading();
		if (!std::isnan(bearing_now) && bearing_now >= 0.0 && bearing_now < 360.0) {
		  double dBearing = wrapDeg180(bearing_now - last_bearing_deg_);
		  last_bearing_deg_ = bearing_now;
		  dTheta = -deg2rad(dBearing);
		}
	  } else {
		double cnt = 0.0;
		double sum = 0.0;
		if (std::abs(cfg_.rv) > 1e-6) { sum += (dSv_meas / cfg_.rv); cnt += 1.0; }
		if (std::abs(cfg_.rh) > 1e-6) { sum += (-dSh_meas / cfg_.rh); cnt += 1.0; }
		if (cnt > 0.0) dTheta = sum / cnt;
	  }

	  double theta_mid;
	  {
		std::scoped_lock lk(mtx_);
		theta_mid = theta_rad_ + dTheta * 0.5;
	  }

	  double dSf = dSv_meas - cfg_.rv * dTheta;
	  double dSl = dSh_meas + cfg_.rh * dTheta;
	  double dX = dSl * std::cos(theta_mid) + dSf * std::sin(theta_mid);
	  double dY = dSf * std::cos(theta_mid) - dSl * std::sin(theta_mid);

	  {
		std::scoped_lock lk(mtx_);
		x_ += dX;
		y_ += dY;
		theta_rad_ += dTheta;
	  }

	  pros::delay(cfg_.period_ms);
	}
  }
};

// pros::Imu imu(9);
// pros::Rotation vRot(1);
// pros::Rotation hRot(2);
// Odom::Config cfg;
// cfg.wheel_diam_in = 2.75;
// cfg.rv = 1.50;
// cfg.rh = 0.75;
// Odom odom(vRot, hRot, imu, cfg);
// void initialize() { imu.reset(); while (imu.is_calibrating()) pros::delay(20); odom.reset(0.0, 0.0); odom.start(); }
// void opcontrol() { while (true) { double x = odom.x(); double y = odom.y(); double hdg = odom.heading_bearing_deg(); pros::delay(20); } }




*/