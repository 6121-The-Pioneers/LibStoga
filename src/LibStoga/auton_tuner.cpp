#include "LibStoga/auton_tuner.h"
#include "LibStoga/chassis.h"
#include "LibStoga/pid.h"
#include "LibStoga/gains.h"
#include "LibStoga/Timer.hpp"
#include "pros/rtos.hpp"
#include <iostream>
#include <vector>
#include <memory>
#include <limits>
#include <cmath>

namespace ls {

// Implementation of AdaptivePIDController methods
void AdaptivePIDController::update() {
    uint32_t current_time = pros::millis();

    // Only adapt periodically to avoid excessive computation
    if (current_time - last_adaptation_time < ADAPTATION_INTERVAL) {
        return;
    }
    last_adaptation_time = current_time;

    // Get current battery voltage
    double battery_voltage = pros::battery::get_voltage() / 1000.0; // Convert mV to V

    // Detect surface changes based on recent performance
    detectSurfaceConditions();

    // Calculate battery compensation
    updateBatteryCompensation(battery_voltage);

    // Calculate surface compensation
    updateSurfaceCompensation();

    // Apply combined compensations
    applyAdaptiveGains();

    last_battery_voltage = battery_voltage;
}

void AdaptivePIDController::recordMovement(double final_error, double settling_time) {
    recent_errors.push_back(final_error);
    recent_settling_times.push_back(settling_time);

    // Keep only recent performance data
    if (recent_errors.size() > PERFORMANCE_WINDOW) {
        recent_errors.erase(recent_errors.begin());
        recent_settling_times.erase(recent_settling_times.begin());
    }
}

void AdaptivePIDController::updateBatteryCompensation(double current_voltage) {
    // Battery compensation: lower voltage = higher gains needed
    float voltage_ratio = current_voltage / BATTERY_THRESHOLD_HIGH;

    // More aggressive compensation for low battery
    if (current_voltage < BATTERY_THRESHOLD_LOW) {
        float low_battery_factor = BATTERY_THRESHOLD_LOW / current_voltage;
        battery_compensation.kP = std::min(low_battery_factor * 1.3f, MAX_GAIN_MULTIPLIER);
        battery_compensation.kI = std::min(low_battery_factor * 1.1f, MAX_GAIN_MULTIPLIER);
        battery_compensation.kD = std::min(low_battery_factor * 1.2f, MAX_GAIN_MULTIPLIER);
    } else {
        // Gradual compensation for normal voltage range
        battery_compensation.kP = std::max(voltage_ratio * 0.9f, 0.7f);
        battery_compensation.kI = std::max(voltage_ratio * 0.95f, 0.8f);
        battery_compensation.kD = std::max(voltage_ratio * 0.85f, 0.6f);
    }

    // Smooth voltage changes to prevent oscillations
    static float last_p_comp = 1.0f, last_i_comp = 1.0f, last_d_comp = 1.0f;
    battery_compensation.kP = last_p_comp * 0.8f + battery_compensation.kP * 0.2f;
    battery_compensation.kI = last_i_comp * 0.8f + battery_compensation.kI * 0.2f;
    battery_compensation.kD = last_d_comp * 0.8f + battery_compensation.kD * 0.2f;

    last_p_comp = battery_compensation.kP;
    last_i_comp = battery_compensation.kI;
    last_d_comp = battery_compensation.kD;
}

void AdaptivePIDController::detectSurfaceConditions() {
    if (recent_errors.size() < 5) return; // Need enough data

    // Calculate average error and settling time
    double avg_error = 0.0, avg_settling = 0.0;
    for (size_t i = 0; i < recent_errors.size(); ++i) {
        avg_error += recent_errors[i];
        avg_settling += recent_settling_times[i];
    }
    avg_error /= recent_errors.size();
    avg_settling /= recent_settling_times.size();

    // Surface detection logic
    SurfaceType detected_surface = UNKNOWN;

    if (avg_error > 2.0 && avg_settling > 1500) {
        detected_surface = ROUGH;  // High error and long settling = rough surface
    } else if (avg_error < 0.5 && avg_settling < 800) {
        detected_surface = SMOOTH; // Low error and fast settling = smooth surface
    } else {
        detected_surface = CARPET; // Medium performance = carpet
    }

    // Only change surface type if consistently detected
    if (detected_surface != current_surface) {
        surface_detection_time = pros::millis();
    } else if (pros::millis() - surface_detection_time > 3000) { // 3 seconds of consistent detection
        if (current_surface != detected_surface) {
            current_surface = detected_surface;
            std::cout << "Surface changed to: " <<
                (current_surface == SMOOTH ? "SMOOTH" :
                 current_surface == ROUGH ? "ROUGH" : "CARPET") << std::endl;
        }
    }
}

void AdaptivePIDController::updateSurfaceCompensation() {
    // Surface compensation factors
    switch (current_surface) {
        case SMOOTH:
            surface_compensation.kP = 0.9f;  // Less aggressive on smooth surfaces
            surface_compensation.kI = 1.1f;  // More integral for steady state
            surface_compensation.kD = 0.8f;  // Less derivative on smooth
            break;
        case ROUGH:
            surface_compensation.kP = 1.3f;  // More aggressive on rough surfaces
            surface_compensation.kI = 0.8f;  // Less integral to avoid windup
            surface_compensation.kD = 1.2f;  // More derivative for rough
            break;
        case CARPET:
            surface_compensation.kP = 1.0f;  // Baseline for carpet
            surface_compensation.kI = 1.0f;
            surface_compensation.kD = 1.0f;
            break;
        default:
            surface_compensation = {1.0f, 1.0f, 1.0f}; // No compensation for unknown
            break;
    }
}

void AdaptivePIDController::applyAdaptiveGains() {
    // Combine all compensation factors
    PIDGains adapted_gains = base_gains;

    adapted_gains.kP *= battery_compensation.kP * surface_compensation.kP;
    adapted_gains.kI *= battery_compensation.kI * surface_compensation.kI;
    adapted_gains.kD *= battery_compensation.kD * surface_compensation.kD;

    // Apply safety limits
    adapted_gains.kP = std::max(MIN_GAIN_MULTIPLIER * base_gains.kP,
                               std::min(MAX_GAIN_MULTIPLIER * base_gains.kP, adapted_gains.kP));
    adapted_gains.kI = std::max(MIN_GAIN_MULTIPLIER * base_gains.kI,
                               std::min(MAX_GAIN_MULTIPLIER * base_gains.kI, adapted_gains.kI));
    adapted_gains.kD = std::max(MIN_GAIN_MULTIPLIER * base_gains.kD,
                               std::min(MAX_GAIN_MULTIPLIER * base_gains.kD, adapted_gains.kD));

    // Only update if gains changed significantly
    const float GAIN_CHANGE_THRESHOLD = 0.05f; // 5% change threshold
    if (fabs(adapted_gains.kP - current_gains.kP) > GAIN_CHANGE_THRESHOLD ||
        fabs(adapted_gains.kI - current_gains.kI) > GAIN_CHANGE_THRESHOLD ||
        fabs(adapted_gains.kD - current_gains.kD) > GAIN_CHANGE_THRESHOLD) {

        current_gains = adapted_gains;
        pid->setGains(current_gains);

        // Optional: Log adaptation (can be commented out for performance)
        // std::cout << "Adapted gains: P=" << current_gains.kP << ", I=" << current_gains.kI
        //           << ", D=" << current_gains.kD << std::endl;
    }
}

// Global adaptive controllers (one for lateral, one for angular)
static std::unique_ptr<AdaptivePIDController> lateral_adaptive_controller;
static std::unique_ptr<AdaptivePIDController> angular_adaptive_controller;

// Helper function to find ultimate gains using the safe Relay (Åström-Hägglund) method
static std::pair<double, double> findUltimateGainsRelay(std::shared_ptr<Chassis> chassis, bool isAngular) {
    const double relay_amplitude = 30.0; // Reduced from 60.0 to prevent excessive movement
    const int min_cycles = 5; // Minimum number of relay cycles to observe
    const int max_cycles = 20; // Maximum cycles to prevent infinite loop
    const unsigned int cycle_timeout = 3000; // Reduced from 5000 to 3 seconds per cycle

    std::shared_ptr<AbstractPID> pid = isAngular ? chassis->getAngularControl() : chassis->getLateralControl();
    pid->setGains({0.0f, 0.0f, 0.0f}); // Disable PID, use pure relay control

    std::vector<double> peaks;
    std::vector<unsigned long> peak_times;
    bool relay_state = true; // Start with positive relay output

    chassis->getOdom()->resetAll();

    for (int cycle = 0; cycle < max_cycles; ++cycle) {
        ls::Timer cycle_timer(cycle_timeout);
        double max_error = 0.0;
        unsigned long max_error_time = 0;

        while (!cycle_timer.isDone()) {
            chassis->getOdom()->compute();
            double current_pos = isAngular ? chassis->getOdom()->getAngle() : chassis->getOdom()->getPosition().Y;
            double error = relay_state ? relay_amplitude : -relay_amplitude; // Relay control

            if (fabs(current_pos) > fabs(max_error)) {
                max_error = current_pos;
                max_error_time = pros::millis();
            }

            double power = relay_state ? relay_amplitude : -relay_amplitude;
            if (isAngular) {
                chassis->arcade(0, power);
            } else {
                chassis->arcade(power, 0);
            }

            pros::delay(10);
        }

        peaks.push_back(max_error);
        peak_times.push_back(max_error_time);
        relay_state = !relay_state; // Switch relay state

        if (peaks.size() >= min_cycles) {
            // Check for stable oscillation
            double avg_peak = 0.0;
            for (double p : peaks) avg_peak += fabs(p);
            avg_peak /= peaks.size();

            if (peaks.size() >= 2) {
                double period = (peak_times.back() - peak_times[peaks.size() - 2]) / 1000.0; // seconds
                double amplitude = avg_peak;
                double ku = 4 * relay_amplitude / (M_PI * amplitude); // Ultimate gain
                double tu = period; // Ultimate period

                std::cout << "Relay Method: Ku=" << ku << ", Tu=" << tu << "s" << std::endl;
                return {ku, tu};
            }
        }
    }

    std::cout << "Relay Method failed to find stable oscillation." << std::endl;
    return {0, 0};
}

// Adaptive tuning function using hill-climbing algorithm
static void adaptiveTune(std::shared_ptr<Chassis> chassis, PIDGains& lateral_gains, PIDGains& angular_gains, pros::Controller* controller) {
    const int MAX_ITERATIONS = 20;
    const float STEP_SIZE = 0.1f; // Smaller step size for finer tuning
    const float DISTANCE = 6.0f; // Reduced distance for safety
    const float ANGLE = 45.0f; // Reduced angle for safety

    PIDGains best_lateral = lateral_gains;
    PIDGains best_angular = angular_gains;
    double best_score = std::numeric_limits<double>::max();

    std::cout << "Starting adaptive refinement with enforced Y pause safety..." << std::endl;
    std::cout << "Press Y at any time to pause/resume tuning." << std::endl;
    std::cout << "Press X to cancel tuning entirely." << std::endl;

    for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
        // ENFORCED Y PAUSE SAFETY - Check before every movement
        if (controller) {
            // Debounce and check for Y press
            static bool y_pressed_last = false;
            bool y_pressed = controller->get_digital(pros::E_CONTROLLER_DIGITAL_Y);

            if (y_pressed && !y_pressed_last) {
                std::cout << "Tuning PAUSED. Press Y again to resume." << std::endl;
                pros::lcd::print(0, "TUNING PAUSED");
                pros::lcd::print(1, "Press Y to resume");
                pros::delay(200); // Debounce

                // Wait for Y to be released
                while (controller->get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
                    pros::delay(10);
                }

                // Wait for Y to be pressed again
                while (!controller->get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
                    // Check for cancel with X button (require held for 500ms to prevent accidental presses)
                    static uint32_t x_press_start = 0;
                    static bool x_was_pressed = false;
                    bool x_pressed = controller->get_digital(pros::E_CONTROLLER_DIGITAL_X);

                    if (x_pressed && !x_was_pressed) {
                        x_press_start = pros::millis();
                        x_was_pressed = true;
                    } else if (!x_pressed) {
                        x_was_pressed = false;
                        x_press_start = 0;
                    }

                    if (x_pressed && x_was_pressed && (pros::millis() - x_press_start) > 500) {
                        std::cout << "Tuning CANCELLED by user (X held for 500ms)." << std::endl;
                        pros::lcd::print(0, "TUNING CANCELLED");
                        pros::delay(2000);
                        return;
                    }
                    pros::delay(10);
                }

                std::cout << "Tuning RESUMED." << std::endl;
                pros::lcd::clear_line(0);
                pros::lcd::clear_line(1);
                pros::delay(200); // Debounce
            }
            y_pressed_last = y_pressed;

            // Check for cancel with X button (require held for 500ms to prevent accidental presses)
            static uint32_t x_press_start_main = 0;
            static bool x_was_pressed_main = false;
            bool x_pressed_main = controller->get_digital(pros::E_CONTROLLER_DIGITAL_X);

            if (x_pressed_main && !x_was_pressed_main) {
                x_press_start_main = pros::millis();
                x_was_pressed_main = true;
            } else if (!x_pressed_main) {
                x_was_pressed_main = false;
                x_press_start_main = 0;
            }

            if (x_pressed_main && x_was_pressed_main && (pros::millis() - x_press_start_main) > 500) {
                std::cout << "Tuning CANCELLED by user (X held for 500ms)." << std::endl;
                pros::lcd::print(0, "TUNING CANCELLED");
                pros::delay(2000);
                return;
            }
        }

        std::cout << "Iteration " << (iter + 1) << "/" << MAX_ITERATIONS << std::endl;

        // Generate new candidate gains by perturbing current best
        PIDGains candidate_lateral = best_lateral;
        PIDGains candidate_angular = best_angular;

        // Random perturbation
        candidate_lateral.kP *= (1.0f + (rand() % 21 - 10) * STEP_SIZE);
        candidate_lateral.kI *= (1.0f + (rand() % 21 - 10) * STEP_SIZE);
        candidate_lateral.kD *= (1.0f + (rand() % 21 - 10) * STEP_SIZE);
        candidate_angular.kP *= (1.0f + (rand() % 21 - 10) * STEP_SIZE);
        candidate_angular.kI *= (1.0f + (rand() % 21 - 10) * STEP_SIZE);
        candidate_angular.kD *= (1.0f + (rand() % 21 - 10) * STEP_SIZE);

        // Ensure gains stay positive
        candidate_lateral.kP = std::max(0.1f, candidate_lateral.kP);
        candidate_lateral.kI = std::max(0.0f, candidate_lateral.kI);
        candidate_lateral.kD = std::max(0.0f, candidate_lateral.kD);
        candidate_angular.kP = std::max(0.1f, candidate_angular.kP);
        candidate_angular.kI = std::max(0.0f, candidate_angular.kI);
        candidate_angular.kD = std::max(0.0f, candidate_angular.kD);

        // Test candidate gains
        chassis->getLateralControl()->setGains(candidate_lateral);
        chassis->getAngularControl()->setGains(candidate_angular);

        // Perform test movements and calculate score
        double score = 0.0;
        int num_tests = 4;

        for (int test = 0; test < num_tests; ++test) {
            // ENFORCED Y PAUSE SAFETY - Check before each test movement
            if (controller) {
                bool y_pressed = controller->get_digital(pros::E_CONTROLLER_DIGITAL_Y);
                if (y_pressed) {
                    std::cout << "Tuning PAUSED before test movement. Press Y to resume." << std::endl;
                    pros::lcd::print(0, "PAUSED BEFORE TEST");
                    pros::delay(200);
                    while (controller->get_digital(pros::E_CONTROLLER_DIGITAL_Y)) pros::delay(10);
                    while (!controller->get_digital(pros::E_CONTROLLER_DIGITAL_Y)) pros::delay(10);
                    std::cout << "Tuning RESUMED." << std::endl;
                    pros::lcd::clear_line(0);
                    pros::delay(200);
                }

                if (controller->get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
                    // Check for cancel with X button (require held for 500ms to prevent accidental presses)
                    static uint32_t x_press_start_test = 0;
                    static bool x_was_pressed_test = false;
                    bool x_pressed_test = controller->get_digital(pros::E_CONTROLLER_DIGITAL_X);

                    if (x_pressed_test && !x_was_pressed_test) {
                        x_press_start_test = pros::millis();
                        x_was_pressed_test = true;
                    } else if (!x_pressed_test) {
                        x_was_pressed_test = false;
                        x_press_start_test = 0;
                    }

                    if (x_pressed_test && x_was_pressed_test && (pros::millis() - x_press_start_test) > 500) {
                        std::cout << "Tuning CANCELLED by user." << std::endl;
                        return;
                    }
                }
            }

            switch (test) {
                case 0: score += chassis->driveRelative(DISTANCE, 2000); break;
                case 1: score += chassis->driveRelative(-DISTANCE, 2000); break;
                case 2: score += chassis->turnToAngle(ANGLE, 2000); break;
                case 3: score += chassis->turnToAngle(-ANGLE, 2000); break;
            }
            pros::delay(500); // Brief pause between tests
        }

        score /= num_tests;

        std::cout << "Candidate score: " << score << " (best: " << best_score << ")" << std::endl;

        // Update best if improved
        if (score < best_score) {
            best_score = score;
            best_lateral = candidate_lateral;
            best_angular = candidate_angular;
            std::cout << "New best found!" << std::endl;
        } else {
            // Revert to best gains
            chassis->getLateralControl()->setGains(best_lateral);
            chassis->getAngularControl()->setGains(best_angular);
        }

        // Update LCD with current best
        pros::lcd::print(0, "Iter: %d/%d", iter + 1, MAX_ITERATIONS);
        pros::lcd::print(1, "Best Score: %.2f", best_score);
        pros::lcd::print(2, "L-P:%.1f I:%.1f D:%.3f", best_lateral.kP, best_lateral.kI, best_lateral.kD);
        pros::lcd::print(3, "A-P:%.2f I:%.2f D:%.3f", best_angular.kP, best_angular.kI, best_angular.kD);
    }

    // Set final best gains
    lateral_gains = best_lateral;
    angular_gains = best_angular;
}

void autoTune(std::shared_ptr<Chassis> chassis) {
    std::cout << "Starting auto-tuning process..." << std::endl;

    // Phase 1: Get baseline gains using Relay Method
    std::cout << "Phase 1: Finding baseline gains with Relay Method..." << std::endl;

    // Use pre-tuned angular values
    PIDGains angular_gains = {1.74f, 3.16364f, 0.23925f};
    chassis->getAngularControl()->setGains(angular_gains);

    // Find lateral baseline using Relay Method
    auto [lateral_ku, lateral_tu] = findUltimateGainsRelay(chassis, false);
    if (lateral_ku == 0) {
        std::cout << "Failed to find lateral baseline. Aborting." << std::endl;
        return;
    }

    // Calculate initial lateral gains using Tyreus-Luyben rules (more conservative than Ziegler-Nichols)
    PIDGains lateral_gains;
    lateral_gains.kP = lateral_ku / 3.2;
    lateral_gains.kI = lateral_ku / (2.2 * lateral_tu);
    lateral_gains.kD = lateral_ku * lateral_tu / 6.3;
    chassis->getLateralControl()->setGains(lateral_gains);

    std::cout << "Baseline Lateral Gains: P=" << lateral_gains.kP << ", I=" << lateral_gains.kI << ", D=" << lateral_gains.kD << std::endl;

    // Phase 2: Adaptive refinement
    std::cout << "Phase 2: Adaptive refinement..." << std::endl;
    pros::Controller* controller = chassis->getController();
    adaptiveTune(chassis, lateral_gains, angular_gains, controller);

    std::cout << "Auto-tuning complete!" << std::endl;
    std::cout << "Final Angular Gains: P=" << angular_gains.kP << ", I=" << angular_gains.kI << ", D=" << angular_gains.kD << std::endl;
    std::cout << "Final Lateral Gains: P=" << lateral_gains.kP << ", I=" << lateral_gains.kI << ", D=" << lateral_gains.kD << std::endl;
    std::cout << "To enable adaptive PID, set enableAdaptivePID=true in ChassisConfig." << std::endl;

    pros::lcd::print(0, "Tuning Complete!");
    pros::lcd::print(1, "Angular: P:%.2f I:%.2f D:%.2f", angular_gains.kP, angular_gains.kI, angular_gains.kD);
    pros::lcd::print(2, "Lateral: P:%.2f I:%.2f D:%.2f", lateral_gains.kP, lateral_gains.kI, lateral_gains.kD);
}

// Function to update adaptive controllers during operation
void updateAdaptivePID() {
    if (lateral_adaptive_controller) {
        lateral_adaptive_controller->update();
    }
    if (angular_adaptive_controller) {
        angular_adaptive_controller->update();
    }
}

// Function to record movement performance for adaptation learning
void recordMovementPerformance(double lateral_error, double lateral_settling_time,
                              double angular_error, double angular_settling_time) {
    if (lateral_adaptive_controller) {
        lateral_adaptive_controller->recordMovement(lateral_error, lateral_settling_time);
    }
    if (angular_adaptive_controller) {
        angular_adaptive_controller->recordMovement(angular_error, angular_settling_time);
    }
}

// Function to get current adapted gains (for debugging/monitoring)
PIDGains getCurrentLateralGains() {
    return lateral_adaptive_controller ?
           lateral_adaptive_controller->getCurrentGains() :
           PIDGains{0, 0, 0};
}

PIDGains getCurrentAngularGains() {
    return angular_adaptive_controller ?
           angular_adaptive_controller->getCurrentGains() :
           PIDGains{0, 0, 0};
}

} // namespace ls
