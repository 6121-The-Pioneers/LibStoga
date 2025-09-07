#ifndef LS_AUTON_TUNER_H
#define LS_AUTON_TUNER_H

#include <memory>
#include <vector>
#include "gains.h"
#include "pid.h"

namespace ls {
    class Chassis; // Forward declaration

    // Adaptive PID controller that continuously adjusts gains during operation
    class AdaptivePIDController {
    private:
        std::shared_ptr<AbstractPID> pid;
        PIDGains base_gains;           // Original tuned gains
        PIDGains current_gains;        // Currently adapted gains
        PIDGains battery_compensation; // Battery voltage compensation factors
        PIDGains surface_compensation; // Surface condition compensation factors

        // Adaptation parameters
        const float ADAPTATION_RATE = 0.02f;     // How aggressively to adapt (0.01 = conservative, 0.1 = aggressive)
        const float BATTERY_THRESHOLD_LOW = 11.0f;  // Voltage below which to start compensating
        const float BATTERY_THRESHOLD_HIGH = 12.8f; // Full battery voltage
        const int PERFORMANCE_WINDOW = 10;       // Number of movements to average for performance
        const float MAX_GAIN_MULTIPLIER = 2.0f;  // Maximum gain increase allowed
        const float MIN_GAIN_MULTIPLIER = 0.3f;  // Minimum gain decrease allowed

        // Performance tracking
        std::vector<double> recent_errors;
        std::vector<double> recent_settling_times;
        double last_battery_voltage = 12.8f;
        uint32_t last_adaptation_time = 0;
        const uint32_t ADAPTATION_INTERVAL = 5000; // Adapt every 5 seconds

        // Surface detection
        enum SurfaceType { UNKNOWN, SMOOTH, ROUGH, CARPET };
        SurfaceType current_surface = UNKNOWN;
        uint32_t surface_detection_time = 0;

    public:
        AdaptivePIDController(std::shared_ptr<AbstractPID> pid_controller, PIDGains base)
            : pid(pid_controller), base_gains(base), current_gains(base) {

            // Initialize compensation factors
            battery_compensation = {1.0f, 1.0f, 1.0f};  // Start with no compensation
            surface_compensation = {1.0f, 1.0f, 1.0f};

            // Note: Logging removed from constructor to avoid including iostream in header
        }

        void update();
        void recordMovement(double final_error, double settling_time);
        PIDGains getCurrentGains() const { return current_gains; }

    private:
        void updateBatteryCompensation(double current_voltage);
        void detectSurfaceConditions();
        void updateSurfaceCompensation();
        void applyAdaptiveGains();
    };

    /**
     * @brief Automatically tunes the PID controllers for the chassis by running a star pattern.
     *
     * @param chassis A shared pointer to the chassis object.
     */
    void autoTune(std::shared_ptr<Chassis> chassis);

    /**
     * @brief Updates the adaptive PID controllers during operation.
     * Call this periodically (e.g., every 50-100ms) during autonomous or driver control.
     */
    void updateAdaptivePID();

    /**
     * @brief Records movement performance data for adaptive learning.
     *
     * @param lateral_error Final position error for lateral movement (inches)
     * @param lateral_settling_time Time to settle for lateral movement (ms)
     * @param angular_error Final angle error for angular movement (degrees)
     * @param angular_settling_time Time to settle for angular movement (ms)
     */
    void recordMovementPerformance(double lateral_error, double lateral_settling_time,
                                  double angular_error, double angular_settling_time);

    /**
     * @brief Gets the current adapted lateral PID gains.
     *
     * @return Current lateral PID gains
     */
    PIDGains getCurrentLateralGains();

    /**
     * @brief Gets the current adapted angular PID gains.
     *
     * @return Current angular PID gains
     */
    PIDGains getCurrentAngularGains();

} // namespace ls

#endif // LS_AUTON_TUNER_H