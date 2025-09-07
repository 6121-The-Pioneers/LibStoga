#ifndef LS_GAINS_H
#define LS_GAINS_H

namespace ls {
    /**
     * @brief A struct to hold PID gains.
     */
    struct PIDGains {
        float kP{0.f};
        float kI{0.f};
        float kD{0.f};
        float windup{10.f};
        bool resetOnZeroCross{true};
        float integralLimit{1000.f};
    };
}

#endif // LS_GAINS_H
