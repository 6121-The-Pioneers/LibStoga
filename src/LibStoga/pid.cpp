#include "pid.h"
#include <algorithm> 
#include <stdexcept> 

namespace ls {
    PID::PID(float kP, float kI, float kD, float windupRange, bool signFlipReset)
        : kP(kP),
        kI(kI),
        kD(kD),
        windupRange(windupRange),
        signFlipReset(signFlipReset),
        integral(0),
        prevError(0) {
       
        if (kP < 0 || kI < 0 || kD < 0) {
            throw std::invalid_argument("PID constants must be non-negative");
        }
        if (windupRange < 0) {
            throw std::invalid_argument("Windup range must be non-negative");
        }
    }
    
    inline int sgn(float val) {
        return (0 < val) - (val < 0);
    }

    double PID::update(const double error) {
        
        integral += error;
        integral = std::clamp(integral, -windupRange, windupRange);
        if (sgn(error) != sgn(prevError) && signFlipReset) integral = 0;

        
        const float derivative = error - prevError;
        prevError = error;

        
        return error * kP + integral * kI + derivative * kD;
    }

    void PID::reset() {
        integral = 0;
        prevError = 0;
    }
} 
