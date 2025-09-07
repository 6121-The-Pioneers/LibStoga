#pragma once
#include <algorithm>
#include <cmath>

namespace ls {
    // Generic clamp (C++17 already has std::clamp, but this short alias keeps our namespace consistent)
    template <typename T>
    inline T clamp(const T& v, const T& lo, const T& hi) {
        return std::clamp(v, lo, hi);
    }

    // Clamp motor / joystick power to V5 typical range [-127, 127]
    inline double clampPower(double v) { return clamp(v, -127.0, 127.0); }

    // Deadband helper: zero out small magnitude values
    inline double deadband(double v, double db) { return (std::fabs(v) < db) ? 0.0 : v; }

    // Scale a value from one range to another, optionally clamping result
    inline double scale(double v, double inMin, double inMax, double outMin, double outMax, bool clampResult = true) {
        if (inMax - inMin == 0) return outMin; // avoid div/0
        double t = (v - inMin) / (inMax - inMin);
        double out = outMin + t * (outMax - outMin);
        if (clampResult) return clamp(out, std::min(outMin, outMax), std::max(outMin, outMax));
        return out;
    }

    // Sign helper (-1, 0, 1)
    template <typename T>
    inline int sign(T v) { return (v > 0) - (v < 0); }
}
