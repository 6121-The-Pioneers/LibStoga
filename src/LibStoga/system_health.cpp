#include "LibStoga/system_health.h"
#include "pros/misc.hpp"

namespace ls {

SystemHealthMonitor::SystemHealthMonitor() : enabled_(false), monitoringTask_(nullptr) {}

SystemHealthMonitor::~SystemHealthMonitor() {
    stopMonitoring();
}

void SystemHealthMonitor::setEnabled(bool enabled) {
    enabled_ = enabled;
}

void SystemHealthMonitor::startMonitoring() {
    if (enabled_ || monitoringTask_ != nullptr) return;
    enabled_ = true;
    monitoringTask_ = new pros::Task([this]() {
        while (this->enabled_) {
            this->forceHealthCheck();
            pros::delay(100); // Check every 100ms
        }
    });
}

void SystemHealthMonitor::stopMonitoring() {
    if (!enabled_ || monitoringTask_ == nullptr) return;
    enabled_ = false;
    monitoringTask_->remove();
    delete monitoringTask_;
    monitoringTask_ = nullptr;
}

SystemHealthMonitor::HealthStatus SystemHealthMonitor::getOverallHealth() const {
    // Implement logic to determine overall health from issues
    if (!issues_.empty()) {
        for (const auto& issue : issues_) {
            if (issue.severity == FAILURE) return FAILURE;
            if (issue.severity == CRITICAL) return CRITICAL;
        }
        return WARNING;
    }
    return HEALTHY;
}

std::vector<SystemHealthMonitor::HealthIssue> SystemHealthMonitor::getIssues() const {
    return issues_;
}

SystemHealthMonitor::SystemMetrics SystemHealthMonitor::getMetrics() const {
    return currentMetrics_;
}

void SystemHealthMonitor::registerMotor(int port, pros::Motor* motor) {
    monitoredMotors_.push_back({port, motor});
}

void SystemHealthMonitor::setHealthCallback(std::function<void(HealthStatus, const std::vector<HealthIssue>&)> callback) {
    healthCallback_ = callback;
}

void SystemHealthMonitor::forceHealthCheck() {
    updateMetrics();
    checkForIssues();
    clearResolvedIssues();
}

void SystemHealthMonitor::clearResolvedIssues() {
    // Implement logic to clear old/resolved issues
}

void SystemHealthMonitor::updateMetrics() {
    currentMetrics_.batteryVoltage = pros::battery::get_voltage() / 1000.0;
    currentMetrics_.batteryCurrent = pros::battery::get_current() / 1000.0;
    currentMetrics_.timestamp = pros::millis();
    currentMetrics_.motorHealth.clear();
    for (auto& motorPair : monitoredMotors_) {
        MotorHealth health;
        health.port = motorPair.first;
        health.current = motorPair.second->get_current_draw() / 1000.0;
        health.temperature = motorPair.second->get_temperature();
        health.power = motorPair.second->get_power();
        health.rpm = motorPair.second->get_actual_velocity();
        health.isStalled = motorPair.second->get_actual_velocity() == 0; // Simplified stall detection
        health.lastUpdate = pros::millis();
        currentMetrics_.motorHealth.push_back(health);
    }
}

void SystemHealthMonitor::checkForIssues() {
    // Implement issue detection logic
}

} // namespace ls
