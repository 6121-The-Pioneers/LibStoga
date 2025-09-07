/** @file system_health.h
 * @brief Comprehensive system health monitoring for LibStoga robotics framework
 *
 * This file provides a complete system health monitoring system that continuously
 * tracks robot health, detects issues, and provides diagnostic information.
 * The system monitors motors, battery, sensors, and overall system performance
 * to help identify problems before they cause failures.
 *
 * Key Features:
 * - Real-time motor health monitoring (current, temperature, RPM)
 * - Battery voltage and current monitoring with brownout detection
 * - Stall detection and overcurrent protection
 * - Tug-of-war detection through voltage analysis
 * - Configurable health thresholds and alerts
 * - Background monitoring with PROS task integration
 * - Health status callbacks for user notification
 * - Rolling history buffers for trend analysis
 *
 * Monitored Parameters:
 * - **Battery**: Voltage, current, brownout detection
 * - **Motors**: Current draw, temperature, RPM, stall detection
 * - **System**: Overall health status, issue tracking
 * - **Trends**: Historical data analysis for anomaly detection
 *
 * @author 6121D (The Pioneers)
 * @version 3.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup diagnostics
 */

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
 * @brief Comprehensive system health monitoring and diagnostics
 *
 * The SystemHealthMonitor provides real-time monitoring of all critical robot
 * systems, detecting issues before they cause failures. It runs as a background
 * task and provides both programmatic access to health data and callback-based
 * notifications for critical issues.
 *
 * The monitor tracks:
 * - Motor health (current, temperature, RPM, stall conditions)
 * - Battery status (voltage, current, brownout detection)
 * - System performance trends
 * - Odometry drift detection
 * - Sensor communication failures
 *
 * Usage Example:
 * @code
 * // Get global health monitor
 * auto& health = getHealthMonitor();
 *
 * // Register motors for monitoring
 * health.registerMotor(1, &leftFrontMotor);
 * health.registerMotor(2, &leftBackMotor);
 * health.registerMotor(3, &rightFrontMotor);
 * health.registerMotor(4, &rightBackMotor);
 *
 * // Set up health callback
 * health.setHealthCallback([](SystemHealthMonitor::HealthStatus status,
 *                            const std::vector<SystemHealthMonitor::HealthIssue>& issues) {
 *     if (status >= SystemHealthMonitor::WARNING) {
 *         // Handle health issues
 *         pros::lcd::print(0, "Health Issue: %s", issues.back().description.c_str());
 *     }
 * });
 *
 * // Start monitoring
 * health.startMonitoring();
 *
 * // Check health in main loop
 * auto metrics = health.getMetrics();
 * if (metrics.batteryVoltage < 7.0) {
 *     // Low battery warning
 * }
 * @endcode
 */
class SystemHealthMonitor {
public:
    /**
     * @brief Overall system health status levels
     */
    enum HealthStatus {
        HEALTHY,    /**< All systems normal */
        WARNING,    /**< Minor issues detected, monitor closely */
        CRITICAL,   /**< Major issues requiring immediate attention */
        FAILURE     /**< System failure, robot may be inoperable */
    };

    /**
     * @brief Types of health issues that can be detected
     */
    enum IssueType {
        TUG_OF_WAR,       /**< Robot being pushed or stalled externally */
        BROWNOUT,         /**< Low battery voltage detected */
        BROKEN_DRIVE,     /**< Drive motor failure or disconnection */
        STALL,            /**< Motor stall detection */
        OVERCURRENT,      /**< Motor drawing excessive current */
        TEMPERATURE,      /**< Motor overheating */
        ODOMETRY_DRIFT,   /**< Odometry position drift detected */
        SENSOR_FAILURE    /**< Sensor communication failure */
    };

    /**
     * @brief Detailed information about a specific health issue
     */
    struct HealthIssue {
        IssueType type;           /**< Type of health issue */
        HealthStatus severity;    /**< Severity level of the issue */
        std::string description;  /**< Human-readable description */
        uint32_t timestamp;       /**< When the issue was detected (PROS millis) */
        std::string recommendation; /**< Suggested action to resolve the issue */
    };

    /**
     * @brief Health information for a single motor
     */
    struct MotorHealth {
        int port;           /**< Motor port number */
        double current;     /**< Current draw in amperes */
        double temperature; /**< Motor temperature in Celsius */
        double power;       /**< Motor power output (0-1) */
        int rpm;            /**< Motor RPM */
        bool isStalled;     /**< Whether motor appears to be stalled */
        uint32_t lastUpdate; /**< Timestamp of last health update */
    };

    /**
     * @brief Complete system health metrics snapshot
     */
    struct SystemMetrics {
        double batteryVoltage;        /**< Battery voltage in volts */
        double batteryCurrent;        /**< Battery current draw in amperes */
        std::vector<MotorHealth> motorHealth; /**< Health data for all monitored motors */
        double odometryDrift;         /**< Odometry drift measurement */
        uint32_t timestamp;           /**< When metrics were collected */
    };

    /**
     * @brief Default constructor
     */
    SystemHealthMonitor();

    /**
     * @brief Destructor - stops monitoring task
     */
    ~SystemHealthMonitor();

    /**
     * @brief Enable or disable health monitoring
     *
     * @param enabled Whether to enable monitoring
     */
    void setEnabled(bool enabled);

    /**
     * @brief Start background monitoring task
     *
     * Begins continuous health monitoring in a separate PROS task.
     * The task runs every 100ms checking all monitored systems.
     */
    void startMonitoring();

    /**
     * @brief Stop monitoring task
     *
     * Stops the background monitoring task and cleans up resources.
     */
    void stopMonitoring();

    /**
     * @brief Get current overall system health status
     *
     * @return Current health status (HEALTHY, WARNING, CRITICAL, or FAILURE)
     */
    HealthStatus getOverallHealth() const;

    /**
     * @brief Get list of current health issues
     *
     * @return Vector of all currently active health issues
     */
    std::vector<HealthIssue> getIssues() const;

    /**
     * @brief Get current system metrics
     *
     * @return Complete snapshot of current system health metrics
     */
    SystemMetrics getMetrics() const;

    /**
     * @brief Register a motor for health monitoring
     *
     * Adds a motor to the monitoring system. The motor will be checked
     * for current draw, temperature, RPM, and stall conditions.
     *
     * @param port Motor port number
     * @param motor Pointer to PROS motor object
     */
    void registerMotor(int port, pros::Motor* motor);

    /**
     * @brief Set callback for health status changes
     *
     * Registers a callback function that will be called whenever the
     * system health status changes or new issues are detected.
     *
     * @param callback Function to call on health changes
     */
    void setHealthCallback(std::function<void(HealthStatus, const std::vector<HealthIssue>&)> callback);

    /**
     * @brief Manually trigger a health check
     *
     * Forces an immediate health check outside the normal monitoring cycle.
     * Useful for one-time health assessments or debugging.
     */
    void forceHealthCheck();

    /**
     * @brief Clear resolved health issues
     *
     * Removes health issues that are older than 30 seconds, assuming
     * they have been resolved or are no longer relevant.
     */
    void clearResolvedIssues();

private:
    bool enabled_;                                              /**< Whether monitoring is enabled */
    pros::Task* monitoringTask_;                               /**< Background monitoring task */
    SystemMetrics currentMetrics_;                             /**< Current health metrics */
    std::vector<HealthIssue> issues_;                          /**< List of active issues */
    std::vector<std::pair<int, pros::Motor*>> monitoredMotors_; /**< Motors being monitored */
    std::function<void(HealthStatus, const std::vector<HealthIssue>&)> healthCallback_; /**< Health change callback */

    // Rolling buffers for trend analysis (last 5 seconds of data)
    std::deque<double> voltageHistory_;   /**< Battery voltage history */
    std::deque<double> currentHistory_;   /**< Battery current history */
    std::deque<double> positionHistory_;  /**< Position history for drift detection */

    /**
     * @brief Update all system health metrics
     *
     * Collects current data from all monitored systems including
     * battery status, motor health, and sensor readings.
     */
    void updateMetrics();

    /**
     * @brief Check for health issues based on current metrics
     *
     * Analyzes the current metrics against thresholds to detect
     * potential problems and creates appropriate health issues.
     */
    void checkForIssues();
};

/**
 * @brief Get global system health monitor instance
 *
 * Returns a reference to the global SystemHealthMonitor instance.
 * This provides a singleton pattern for easy access throughout the codebase.
 *
 * @return Reference to global health monitor
 */
inline SystemHealthMonitor& getHealthMonitor() {
    static SystemHealthMonitor monitor;
    return monitor;
}

} // namespace ls