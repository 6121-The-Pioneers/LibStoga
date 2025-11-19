#include "spid.h"
#include <cmath>

// instead of getting output, get realtime vector magnitude from odom.
namespace ls {
    SmartPID::SmartPID(std::function<double()> actual, double cc, double w, double lc, double max)
        : kP(1), kI(0), kD(0), P(0), I(0), D(0), prev_val(0), correction_constant(fabs(cc)), windup(fabs(w)), learning_constant(fabs(lc)), max_value(fabs(max)), actual_output(actual) {}

    SmartPID::SmartPID(smart_pid_parameters_t &parameters): kP(1), kI(0), kD(0), P(0), I(0), D(0), prev_val(0)
    {
        actual_output = parameters.actual_output;
        correction_constant = fabs(parameters.correction_constant);
        windup = fabs(parameters.windup);
        learning_constant = fabs(parameters.learning_constant);
        max_value = fabs(parameters.max_value);
    }

    double SmartPID::update(const double e) {
        update_components(e);
        update_constants(e);
        return kP * P + kI * I + kD * D;
    }

    void SmartPID::reset()
    {
        P = 0;
        I = 0;
        D = 0;
        prev_val = 0;
    }

    void SmartPID::update_components(const double e) {
        P = e;
        D = (e - prev_val);
        prev_val = e;

        if (fabs(e) < windup) {
            I += e;
        }
        if (fabs(I) >= windup) {
            I = 0;
        }
    }

    // uses sigmoid function to stop learning when close to target.
    void SmartPID::update_constants(const double e) {
        control_output = kP * P + kI * I + kD * D;
        
        double mu = update_mu();
        double Y = get_expected(e);
        
        double constant = learning_constant * mu * mu * 2 * (control_output - Y);
        
        double CKp = constant * P;
        double CKi = constant * I;
        double CKd = constant * D;
        
        if (fabs(e) > windup) {
            kP -= CKp * learn_sigmoid(e);
            kI -= CKi * learn_sigmoid(e);
            kD -= CKd * learn_sigmoid(e);
        }
       
        if (isnanf(kP) || isnanf(kD) || isnanf(kI) || isinff(kP) || isinff(kD) || isinff(kI)) {
            kP = 1;
            kD = 0;
            kI = 0;
            reset();
        }
    }

    // uses logistic growth over ReLU for smooth motion.
    double SmartPID::get_expected(const double e) const {
        // return max_value / (1.0 + std::exp(-correction_constant * e)) - max_value / 2.0;
        return max_value * tanh(correction_constant * e * e * e);
    }

    // special sigmoid function for learning.
    double SmartPID::learn_sigmoid(double x) const {
        return 1.0 / (1.0 + std::exp(-2 * correction_constant * x)) - 0.5; // make it twice as strong to stop learning near sub-0 error values
    }

    // update mu using current control output and the actual output
    double SmartPID::update_mu() {
        if (actual_output() == 0.0) {
            return 1.7976931348623157E+308;
        }
        return control_output / actual_output();
    }

}
