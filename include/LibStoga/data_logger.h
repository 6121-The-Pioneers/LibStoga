/** @file data_logger.h
 * @brief SD card data logging and persistence system for LibStoga robotics framework
 *
 * This file provides a comprehensive data logging system that saves tuning data,
 * match information, and system performance metrics to the robot's SD card.
 * All operations are completely optional and gracefully fail if no SD card
 * is available, making it safe to use in any environment.
 *
 * Key Features:
 * - PID tuning profile persistence and loading
 * - Adaptive PID history tracking
 * - Match data logging with events and results
 * - CSV export for data analysis
 * - SD card usage monitoring
 * - Graceful degradation when SD card unavailable
 * - Thread-safe file operations
 *
 * Data Types Logged:
 * - **Tuning Profiles**: PID gains with timestamps and notes
 * - **Adaptive Data**: Real-time PID adaptation with performance metrics
 * - **Match Data**: Competition information, events, and results
 * - **System Metrics**: Battery voltage, performance scores, surface conditions
 *
 * @author 6121D (The Pioneers)
 * @version 3.0.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025 6121D - All rights reserved
 *
 * @ingroup utilities
 */

#pragma once
#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <optional>
#include "pros/rtos.hpp"
#include "pros/misc.hpp"
#include "gains.h"

namespace ls {

/**
 * @brief Comprehensive SD card data logging and persistence system
 *
 * The DataLogger provides optional data persistence capabilities for the LibStoga
 * framework. It saves tuning profiles, match data, and system performance metrics
 * to the robot's SD card for later analysis and reuse. All operations are designed
 * to be completely optional and will gracefully fail if no SD card is present.
 *
 * The logger supports multiple data types:
 * - PID tuning profiles with metadata
 * - Adaptive PID performance history
 * - Competition match logs with events
 * - System diagnostics and performance metrics
 *
 * Usage Example:
 * @code
 * // Get global data logger
 * auto& logger = getDataLogger();
 *
 * if (logger.isSDAvailable()) {
 *     // Save a tuning profile
 *     DataLogger::TuningProfile profile{
 *         .name = "competition_tune",
 *         .driveGains = {1.2, 0.05, 0.8},
 *         .turnGains = {2.0, 0.1, 1.0},
 *         .timestamp = pros::millis(),
 *         .notes = "Final competition tuning"
 *     };
 *
 *     if (logger.saveTuningProfile(profile)) {
 *         pros::lcd::print(0, "Profile saved!");
 *     }
 *
 *     // Save match data
 *     DataLogger::MatchData match{
 *         .timestamp = pros::millis(),
 *         .matchType = "Qualification",
 *         .matchNumber = 25,
 *         .autonSelection = 1,
 *         .events = {"Auton started", "Scored 2 points", "Driver control"},
 *         .result = "Win"
 *     };
 *
 *     logger.saveMatchData(match);
 * }
 * @endcode
 */
class DataLogger {
public:
    /**
     * @brief PID tuning profile with metadata
     *
     * Stores a complete set of PID gains along with identification
     * and descriptive information for organization and reuse.
     */
    struct TuningProfile {
        std::string name;        /**< Profile name for identification */
        PIDGains driveGains;     /**< PID gains for drive/lateral motion */
        PIDGains turnGains;      /**< PID gains for turning/angular motion */
        uint32_t timestamp;      /**< When profile was created (PROS millis) */
        std::string notes;       /**< Additional notes about the tuning */
    };

    /**
     * @brief Adaptive PID performance data point
     *
     * Records the state of adaptive PID tuning at a specific point in time,
     * including environmental conditions and performance metrics.
     */
    struct AdaptiveData {
        uint32_t timestamp;      /**< When data was recorded */
        double batteryVoltage;   /**< Battery voltage at time of recording */
        PIDGains adaptedGains;   /**< Current adapted PID gains */
        std::string surfaceType; /**< Surface condition description */
        double performanceScore; /**< Performance metric (0-1, higher is better) */
    };

    /**
     * @brief Competition match data and events
     *
     * Records information about a competition match including events,
     * autonomous selection, and final result for post-match analysis.
     */
    struct MatchData {
        uint32_t timestamp;           /**< Match start time */
        std::string matchType;        /**< "Practice", "Qualification", "Elimination" */
        int matchNumber;              /**< Match number */
        int autonSelection;           /**< Autonomous routine selected (0-3) */
        std::vector<std::string> events; /**< Chronological list of match events */
        std::string result;           /**< Match result ("Win", "Loss", "Tie") */
    };

    /**
     * @brief Default constructor - checks SD card availability
     */
    DataLogger();

    /**
     * @brief Check if SD card is available for data operations
     *
     * @return true if SD card is present and accessible, false otherwise
     */
    bool isSDAvailable() const;

    /**
     * @brief Save PID tuning profile to SD card
     *
     * Saves a tuning profile as a JSON file on the SD card. The profile
     * can later be loaded and applied to the robot's control systems.
     *
     * @param profile The tuning profile to save
     * @return true if successful, false if SD unavailable or save failed
     */
    bool saveTuningProfile(const TuningProfile& profile);

    /**
     * @brief Load PID tuning profile from SD card
     *
     * Loads a previously saved tuning profile from the SD card.
     * Returns empty optional if profile not found or SD unavailable.
     *
     * @param name Name of the profile to load
     * @return Optional containing the profile if successful
     */
    std::optional<TuningProfile> loadTuningProfile(const std::string& name);

    /**
     * @brief Save adaptive PID history data
     *
     * Saves a series of adaptive PID data points to a CSV file for
     * analysis of how PID gains changed over time and conditions.
     *
     * @param history Vector of adaptive data points to save
     * @return true if successful, false if SD unavailable or save failed
     */
    bool saveAdaptiveHistory(const std::vector<AdaptiveData>& history);

    /**
     * @brief Save match data and events
     *
     * Saves detailed information about a competition match including
     * events, autonomous selection, and results for post-match analysis.
     *
     * @param match The match data to save
     * @return true if successful, false if SD unavailable or save failed
     */
    bool saveMatchData(const MatchData& match);

    /**
     * @brief List all saved tuning profiles
     *
     * Returns a list of all tuning profile names currently saved on the SD card.
     * Useful for creating profile selection menus in driver control.
     *
     * @return Vector of profile names (empty if SD unavailable)
     */
    std::vector<std::string> listTuningProfiles();

    /**
     * @brief Get SD card usage statistics
     *
     * Returns a human-readable string with SD card usage information
     * including used space, total space, and percentage used.
     *
     * @return Usage statistics string, or error message if unavailable
     */
    std::string getSDStats();

private:
    bool sdAvailable_;  /**< Whether SD card is available for operations */

    /**
     * @brief Check if SD card is available and accessible
     *
     * Performs a filesystem check to determine if the SD card is present
     * and can be used for file operations.
     *
     * @return true if SD card is available, false otherwise
     */
    bool checkSDAvailable();
};

/**
 * @brief Get global data logger instance
 *
 * Returns a reference to the global DataLogger instance, providing
 * a singleton pattern for easy access throughout the codebase.
 *
 * @return Reference to global data logger
 */
inline DataLogger& getDataLogger() {
    static DataLogger logger;
    return logger;
}

} // namespace ls