#include "pid.h"
#include <algorithm> 
#include <stdexcept> 

namespace ls {
    PID::PID(float kP, float kI, float kD, float windupRange, bool signFlipReset, float integralLimit)
        : kP(kP),
        kI(kI),
        kD(kD),
        windupRange(windupRange),
        signFlipReset(signFlipReset),
        integral(0),
        prevError(0),
        integralLimit(integralLimit) {
       
        if (kP < 0 || kI < 0 || kD < 0) {
            throw std::invalid_argument("PID constants must be non-negative");
        }
        if (windupRange < 0) {
            throw std::invalid_argument("Windup range must be non-negative");
        }
    }
    
    PID::PID(const PIDGains& gains)
        : PID(gains.kP, gains.kI, gains.kD, gains.windup, gains.resetOnZeroCross, gains.integralLimit) {
    }
    
    inline int sgn(float val) {
        return (0 < val) - (val < 0);
    }

    double PID::update(const double error) {
        
    unsigned long now = pros::millis();
    double dt = (lastTimeMs == 0) ? 0.01 : (now - lastTimeMs) / 1000.0; // seconds
    lastTimeMs = now;

    integral += error * dt;
    integral = std::clamp(integral, -std::min(windupRange, integralLimit), std::min(windupRange, integralLimit));
        if (sgn(error) != sgn(prevError) && signFlipReset) integral = 0;

    const float derivative = dt > 0 ? (error - prevError) / dt : 0;
        prevError = error;

        
        return error * kP + integral * kI + derivative * kD;
    }

    void PID::reset() {
        integral = 0;
        prevError = 0;
        lastTimeMs = 0;
    }

    void PID::setGains(const PIDGains& gains) {
        kP = gains.kP;
        kI = gains.kI;
        kD = gains.kD;
        windupRange = gains.windup;
        signFlipReset = gains.resetOnZeroCross;
        integralLimit = gains.integralLimit;
        reset();
    }

    PIDGains PID::getGains() const {
        return {kP, kI, kD, windupRange, signFlipReset, integralLimit};
    }
}
