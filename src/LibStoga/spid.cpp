#include "spid.h"
#include <cmath>

// instead of getting output, get realtime vector magnitude from odom.
namespace ls {
    SmartPID::SmartPID(double cc, double w, double lc, double max)
        : kp(0), kd(0), P(0), D(0), prev_val(0), correction_constant(fabs(cc)), windup(fabs(w)), learning_constant(fabs(lc)), max_value(fabs(max)) {}

    SmartPID::SmartPID(smart_pid_parameters_t &parameters): kp(0), kd(0), P(0), D(0), prev_val(0)
    {
        correction_constant = fabs(parameters.correction_constant);
        windup = fabs(parameters.windup);
        learning_constant = fabs(parameters.learning_constant);
        max_value = fabs(parameters.max_value);
    }

    double SmartPID::update(const double e) {
        P = e;
        update_components(e);
        update_constants(e);
        return kp * P + kd * D;
    }

    void SmartPID::reset()
    {
        P = 0;
        D = 0;
        prev_val = 0;
    }

    void SmartPID::update_components(const double e) {
        P = e;
        D = (e - prev_val);
        prev_val = e;
    }

	// uses sigmoid function to stop learning when close to target.
    void SmartPID::update_constants(const double e) {
        double theta = kp * P + kd * D;
        double Y = get_expected(e);
        double constant = learning_constant * 2 * (theta - Y);
        double CKp = constant * P;
        double CKd = constant * D;
        
        if (fabs(e) > windup) {
            kp -= CKp * learn_sigmoid(e);
            kd -= CKd * learn_sigmoid(e);
        }

       
        if (isnanf(kp) || isnanf(kd) || isinff(kp) || isinff(kd)) {
            kp = 0;
            kd = 0;
            reset();
        }
    }

    // uses logistic growth over ReLU for smooth motion.
    double SmartPID::get_expected(const double e) const {
        return (double)max_value / (1 + exp(-correction_constant * e)) - (double)max_value / 2;
    }

    // special sigmoid function for learning.
    double SmartPID::learn_sigmoid(double x) const {
        return 1.0 / (1.0 + std::exp(-2 * correction_constant * x)) - 0.5; // make it twice as strong to stop learning near sub-0 error values
    }

}

