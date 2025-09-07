/**
 * @file examples.cpp
 * @brief Example usage of LibStoga's new features
 *
 * This file demonstrates how to use the new Phase 1 features:
 * - Joystick curves and configurable drive methods
 * - Optional SD card data persistence
 * - LemLib-style movement parameters
 * - System health monitoring
 */

#include "LibStoga/libstoga.h"
#include <iostream>

// Example 1: Joystick Curves and Configurable Drive Methods
void exampleDriverControl() {
    // Create a driver control configuration
    ls::DriverControl::DriveConfig config;
    config.type = ls::DriverControl::DriveType::CHEESY;
    config.throttleCurve = ls::JoystickCurve::CurveType::EXPONENTIAL;
    config.turnCurve = ls::JoystickCurve::CurveType::SIGMOID;
    config.throttleGain = 1.2;
    config.turnGain = 0.8;
    config.enableDeadzone = true;
    config.deadzoneThreshold = 5.0;

    ls::DriverControl driverControl(config);

    // In opcontrol loop:
    // pros::Controller controller(pros::E_CONTROLLER_MASTER);
    // double leftPower, rightPower;
    // driverControl.calculateDrivePowers(controller, leftPower, rightPower);
    // leftMotors.move(leftPower);
    // rightMotors.move(rightPower);

    std::cout << "Driver control configured with Cheesy Drive and custom curves\n";
}

// Example 2: Optional SD Card Data Persistence
void exampleDataPersistence() {
    ls::DataLogger& logger = ls::getDataLogger();

    if (logger.isSDAvailable()) {
        std::cout << "SD card available: " << logger.getSDStats() << "\n";

        // Save a tuning profile
        ls::DataLogger::TuningProfile profile;
        profile.name = "competition_tune";
        profile.driveGains = {12.0f, 0.05f, 4.0f};
        profile.turnGains = {18.0f, 0.0f, 6.5f};
        profile.timestamp = pros::millis();
        profile.notes = "Final competition tuning";

        if (logger.saveTuningProfile(profile)) {
            std::cout << "Tuning profile saved successfully\n";
        }

        // List available profiles
        auto profiles = logger.listTuningProfiles();
        std::cout << "Available profiles: ";
        for (const auto& name : profiles) {
            std::cout << name << " ";
        }
        std::cout << "\n";

        // Load a profile
        auto loadedProfile = logger.loadTuningProfile("competition_tune");
        if (loadedProfile) {
            std::cout << "Loaded profile: " << loadedProfile->name << "\n";
        }

    } else {
        std::cout << "SD card not available - data persistence disabled\n";
    }
}

// Example 3: LemLib-Style Movement Parameters
void exampleEnhancedMovements() {
    // Note: This would be implemented in an actual Chassis class
    // that inherits from EnhancedChassis

    // Example movement with advanced parameters
    ls::MoveToPointParams moveParams;
    moveParams.forwards = true;
    moveParams.maxSpeed = 100.0;
    moveParams.minSpeed = 20.0;
    moveParams.earlyExitRange = 2.0; // Exit when within 2 inches
    moveParams.lead = 0.7;
    moveParams.horizontalDrift = 3.0;

    // chassis.moveToPoint(24, 12, 3000, moveParams);

    ls::TurnToHeadingParams turnParams;
    turnParams.direction = ls::AngularDirection::AUTO;
    turnParams.maxSpeed = 80.0;
    turnParams.minSpeed = 15.0;
    turnParams.earlyExitRange = 3.0; // Exit when within 3 degrees

    // chassis.turnToHeading(90, 2000, turnParams);

    std::cout << "Enhanced movement parameters configured\n";
}

// Example 4: System Health Monitoring
void exampleHealthMonitoring() {
    ls::SystemHealthMonitor& monitor = ls::getHealthMonitor();

    // Register motors for monitoring (example ports)
    // monitor.registerMotor(1, &leftFrontMotor);
    // monitor.registerMotor(2, &leftBackMotor);
    // monitor.registerMotor(3, &rightFrontMotor);
    // monitor.registerMotor(4, &rightBackMotor);

    // Set up health callback
    monitor.setHealthCallback([](ls::SystemHealthMonitor::HealthStatus status,
                                const std::vector<ls::SystemHealthMonitor::HealthIssue>& issues) {
        std::cout << "Health status changed to: ";
        switch (status) {
            case ls::SystemHealthMonitor::HEALTHY: std::cout << "HEALTHY"; break;
            case ls::SystemHealthMonitor::WARNING: std::cout << "WARNING"; break;
            case ls::SystemHealthMonitor::CRITICAL: std::cout << "CRITICAL"; break;
            case ls::SystemHealthMonitor::FAILURE: std::cout << "FAILURE"; break;
        }
        std::cout << "\n";

        for (const auto& issue : issues) {
            std::cout << "Issue: " << issue.description << "\n";
            std::cout << "Recommendation: " << issue.recommendation << "\n";
        }
    });

    // Start monitoring
    monitor.setEnabled(true);

    // In main loop, you could check:
    // auto health = monitor.getOverallHealth();
    // auto issues = monitor.getIssues();
    // auto metrics = monitor.getMetrics();

    std::cout << "System health monitoring enabled\n";
}

// Example 5: Complete Integration Example
void exampleCompleteIntegration() {
    std::cout << "=== LibStoga Phase 1 Features Integration Example ===\n\n";

    // 1. Set up driver control
    exampleDriverControl();
    std::cout << "\n";

    // 2. Initialize data persistence
    exampleDataPersistence();
    std::cout << "\n";

    // 3. Configure enhanced movements
    exampleEnhancedMovements();
    std::cout << "\n";

    // 4. Start health monitoring
    exampleHealthMonitoring();
    std::cout << "\n";

    std::cout << "=== All Phase 1 features initialized successfully! ===\n";
    std::cout << "\nFeatures included:\n";
    std::cout << "✓ Joystick curves and configurable drive methods\n";
    std::cout << "✓ Optional SD card data persistence\n";
    std::cout << "✓ LemLib-style movement parameters\n";
    std::cout << "✓ System health monitoring (tug of war, brownouts, broken drive)\n";
    std::cout << "\nAll features are completely optional and won't affect existing code.\n";
}

/**
 * @brief Example main function showing integration
 *
 * In your actual robot code, you would call these functions in appropriate places:
 * - exampleDriverControl() in initialize() or opcontrol() setup
 * - exampleDataPersistence() when saving/loading tuning data
 * - exampleEnhancedMovements() when using advanced movement functions
 * - exampleHealthMonitoring() in initialize()
 */
int main() {
    exampleCompleteIntegration();
    return 0;
}