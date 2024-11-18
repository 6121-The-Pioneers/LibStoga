#include "spid.h"
#include <cmath>

static double max_val = 1.7976931348623157E+308;

static inline double abs(double val) {
    if (val < 0) return -val;
    return val;
}

namespace ls {
    SmartPID::SmartPID(double cc, double w, double lc, double max, double damp)
        : kp(0), ki(0), kd(0), P(0), I(0), D(0), prev_val(0), correction_constant(cc), windup(w), learning_constant(lc), max_value(max), damp(damp) {}

    double SmartPID::update(const double e) {
        P = e;
        update_components(e);
        update_constants(e);

        double tor = kp * P + ki * I + kd * D;
        tor -= damp * velocity;
        velocity = tor + prev_velocity;
        
        return tor;
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
        if (abs(I) > windup || abs(I) < 0.5) {
            I = 0;
        }
        I += e;
        D = (e - prev_val);
        prev_val = e;
    }

    void SmartPID::update_constants(const double e) {
        double theta = kp * P + ki * I + kd * D;
        double Y = get_expected(e);
        double constant = learning_constant * 2 * (theta - Y);
                
        double CKp = constant * P;
        double CKi = constant * I;
        double CKd = constant * D;
                
        kp -= CKp > max_value ? max_value : CKp;
        ki -= CKi > max_value ? max_value : CKi;
        kd -= CKd > max_value ? max_value : CKd;

        if (isnanf(kp) || isnanf(ki) || isnanf(kd) || isinff(kp) || isinff(ki) || isinff(kd)) {
            kp = 0;
            ki = 0;
            kd = 0;
            reset();
        }
    }

    double SmartPID::get_expected(const double e) { 
        double Y = correction_constant * e;
        if (abs(Y) > max_value) {
            Y = max_value;
        }
        return Y;
    }

}

