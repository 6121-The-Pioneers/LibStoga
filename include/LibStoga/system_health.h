#pragma once
#include <vector>
#include <deque>
#include <memory>
#include <functional>
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "Timer.hpp"

namespace ls {

/**
 * @brief System health monitoring for detecting various robot issues
 */
class SystemHealthMonitor {
public:
    enum HealthStatus {
        HEALTHY,           // Everything normal
        WARNING,           // Minor issues detected
        CRITICAL,          // Major issues requiring attention
        FAILURE           // System failure
    };

    enum IssueType {
        TUG_OF_WAR,       // Robot being pushed/stalled
        BROWNOUT,         // Low battery voltage
        BROKEN_DRIVE,     // Drive motor failure
        STALL,            // Motor stall detection
        OVERCURRENT,      // Motor overcurrent
        TEMPERATURE,      // Motor overheating
        ODOMETRY_DRIFT,   // Odometry position drift
        SENSOR_FAILURE    // Sensor communication failure
    };

    struct HealthIssue {
        IssueType type;
        HealthStatus severity;
        std::string description;
        uint32_t timestamp;
        std::string recommendation;
    };

    struct MotorHealth {
        int port;
        double current;
        double temperature;
        double power;
        int rpm;
        bool isStalled;
        uint32_t lastUpdate;
    };

    struct SystemMetrics {
        double batteryVoltage;
        double batteryCurrent;
        std::vector<MotorHealth> motorHealth;
        double odometryDrift;
        uint32_t timestamp;
    };

    SystemHealthMonitor() : enabled_(true), monitoringTask_(nullptr) {}

    ~SystemHealthMonitor() {
        stopMonitoring();
    }

    /**
     * @brief Enable or disable health monitoring
     */
    void setEnabled(bool enabled) {
        enabled_ = enabled;
        if (enabled) {
            startMonitoring();
        } else {
            stopMonitoring();
        }
    }

    /**
     * @brief Start background monitoring task
     */
    void startMonitoring() {
        if (!enabled_ || monitoringTask_ != nullptr) return;

        monitoringTask_ = new pros::Task([this]() {
            while (enabled_) {
                updateMetrics();
                checkForIssues();
                pros::delay(100); // Check every 100ms
            }
        });
    }

    /**
     * @brief Stop monitoring task
     */
    void stopMonitoring() {
        enabled_ = false;
        if (monitoringTask_ != nullptr) {
            monitoringTask_->remove();
            delete monitoringTask_;
            monitoringTask_ = nullptr;
        }
    }

    /**
     * @brief Get current system health status
     */
    HealthStatus getOverallHealth() const {
        if (issues_.empty()) return HEALTHY;

        HealthStatus worst = HEALTHY;
        for (const auto& issue : issues_) {
            if (issue.severity > worst) {
                worst = issue.severity;
            }
        }
        return worst;
    }

    /**
     * @brief Get list of current health issues
     */
    std::vector<HealthIssue> getIssues() const {
        return issues_;
    }

    /**
     * @brief Get current system metrics
     */
    SystemMetrics getMetrics() const {
        return currentMetrics_;
    }

    /**
     * @brief Register a motor for monitoring
     */
    void registerMotor(int port, pros::Motor* motor) {
        monitoredMotors_.push_back({port, motor});
    }

    /**
     * @brief Set callback for health status changes
     */
    void setHealthCallback(std::function<void(HealthStatus, const std::vector<HealthIssue>&)> callback) {
        healthCallback_ = callback;
    }

    /**
     * @brief Manually trigger health check
     */
    void forceHealthCheck() {
        updateMetrics();
        checkForIssues();
    }

    /**
     * @brief Clear resolved issues
     */
    void clearResolvedIssues() {
        auto it = std::remove_if(issues_.begin(), issues_.end(),
            [](const HealthIssue& issue) {
                // Remove issues older than 30 seconds
                return (pros::millis() - issue.timestamp) > 30000;
            });
        issues_.erase(it, issues_.end());
    }

private:
    bool enabled_;
    pros::Task* monitoringTask_;
    SystemMetrics currentMetrics_;
    std::vector<HealthIssue> issues_;
    std::vector<std::pair<int, pros::Motor*>> monitoredMotors_;
    std::function<void(HealthStatus, const std::vector<HealthIssue>&)> healthCallback_;

    // Rolling buffers for trend analysis
    std::deque<double> voltageHistory_;
    std::deque<double> currentHistory_;
    std::deque<double> positionHistory_;

    void updateMetrics() {
        currentMetrics_.timestamp = pros::millis();
        currentMetrics_.batteryVoltage = pros::battery::get_voltage() / 1000.0;
        currentMetrics_.batteryCurrent = pros::battery::get_current() / 1000.0;

        // Update motor health
        currentMetrics_.motorHealth.clear();
        for (const auto& motorPair : monitoredMotors_) {
            if (motorPair.second) {
                MotorHealth health;
                health.port = motorPair.first;
                health.current = motorPair.second->get_current_draw() / 1000.0;
                health.temperature = motorPair.second->get_temperature();
                health.power = motorPair.second->get_power();
                health.rpm = motorPair.second->get_actual_velocity();
                health.lastUpdate = currentMetrics_.timestamp;

                // Stall detection
                health.isStalled = (std::abs(health.power) > 0.1 && std::abs(health.rpm) < 5);

                currentMetrics_.motorHealth.push_back(health);
            }
        }

        // Maintain rolling history (last 50 readings = 5 seconds)
        voltageHistory_.push_back(currentMetrics_.batteryVoltage);
        currentHistory_.push_back(currentMetrics_.batteryCurrent);

        if (voltageHistory_.size() > 50) voltageHistory_.pop_front();
        if (currentHistory_.size() > 50) currentHistory_.pop_front();
    }

    void checkForIssues() {
        std::vector<HealthIssue> newIssues;

        // Check battery voltage (brownout detection)
        if (currentMetrics_.batteryVoltage < 6.5) {
            newIssues.push_back({
                BROWNOUT,
                currentMetrics_.batteryVoltage < 6.0 ? CRITICAL : WARNING,
                "Low battery voltage detected",
                currentMetrics_.timestamp,
                "Check battery charge or reduce motor load"
            });
        }

        // Check for voltage drops (tug of war detection)
        if (voltageHistory_.size() >= 10) {
            double avgVoltage = 0.0;
            for (double v : voltageHistory_) avgVoltage += v;
            avgVoltage /= voltageHistory_.size();

            double voltageDrop = avgVoltage - currentMetrics_.batteryVoltage;
            if (voltageDrop > 0.5) { // 0.5V drop indicates heavy load
                newIssues.push_back({
                    TUG_OF_WAR,
                    WARNING,
                    "Sudden voltage drop detected - possible tug of war",
                    currentMetrics_.timestamp,
                    "Reduce motor power or check for external resistance"
                });
            }
        }

        // Check motor health
        for (const auto& motor : currentMetrics_.motorHealth) {
            // Overcurrent detection
            if (motor.current > 2.5) { // 2.5A threshold
                newIssues.push_back({
                    OVERCURRENT,
                    motor.current > 3.0 ? CRITICAL : WARNING,
                    "Motor " + std::to_string(motor.port) + " overcurrent",
                    currentMetrics_.timestamp,
                    "Check motor wiring and reduce load"
                });
            }

            // Temperature warning
            if (motor.temperature > 50.0) { // 50°C threshold
                newIssues.push_back({
                    TEMPERATURE,
                    motor.temperature > 60.0 ? CRITICAL : WARNING,
                    "Motor " + std::to_string(motor.port) + " overheating",
                    currentMetrics_.timestamp,
                    "Allow motor to cool or reduce usage"
                });
            }

            // Stall detection
            if (motor.isStalled) {
                newIssues.push_back({
                    BROKEN_DRIVE,
                    CRITICAL,
                    "Motor " + std::to_string(motor.port) + " appears stalled",
                    currentMetrics_.timestamp,
                    "Check motor connection and mechanical issues"
                });
            }
        }

        // Add new issues to the list
        for (const auto& issue : newIssues) {
            // Check if similar issue already exists
            bool exists = false;
            for (const auto& existing : issues_) {
                if (existing.type == issue.type &&
                    existing.severity == issue.severity &&
                    (currentMetrics_.timestamp - existing.timestamp) < 5000) { // Within 5 seconds
                    exists = true;
                    break;
                }
            }

            if (!exists) {
                issues_.push_back(issue);
            }
        }

        // Limit issue history
        if (issues_.size() > 20) {
            issues_.erase(issues_.begin(), issues_.begin() + issues_.size() - 20);
        }

        // Notify callback if health changed
        if (!newIssues.empty() && healthCallback_) {
            healthCallback_(getOverallHealth(), issues_);
        }
    }
};

/**
 * @brief Global health monitor instance
 */
inline SystemHealthMonitor& getHealthMonitor() {
    static SystemHealthMonitor monitor;
    return monitor;
}

} // namespace ls