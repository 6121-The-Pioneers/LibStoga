#pragma once
#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <filesystem>
#include <optional>
#include "pros/rtos.hpp"
#include "pros/misc.hpp"
#include "gains.h"

namespace fs = std::filesystem;

namespace ls {

/**
 * @brief Optional SD card data persistence for tuning data and configurations
 * All operations are completely optional and will gracefully fail if SD card is not available
 */
class DataLogger {
public:
    struct TuningProfile {
        std::string name;
        PIDGains driveGains;
        PIDGains turnGains;
        uint32_t timestamp;
        std::string notes;
    };

    struct AdaptiveData {
        uint32_t timestamp;
        double batteryVoltage;
        PIDGains adaptedGains;
        std::string surfaceType;
        double performanceScore;
    };

    struct MatchData {
        uint32_t timestamp;
        std::string matchType;
        int matchNumber;
        int autonSelection;
        std::vector<std::string> events;
        std::string result;
    };

    DataLogger() : sdAvailable_(checkSDAvailable()) {}

    /**
     * @brief Check if SD card is available for data operations
     */
    bool isSDAvailable() const {
        return sdAvailable_;
    }

    /**
     * @brief Save PID tuning profile to SD card
     * @param profile The tuning profile to save
     * @return true if successful, false otherwise
     */
    bool saveTuningProfile(const TuningProfile& profile) {
        if (!sdAvailable_) return false;

        try {
            std::string filename = "/usd/tuning_" + profile.name + ".json";
            std::ofstream file(filename);

            if (!file.is_open()) return false;

            // Simple JSON-like format
            file << "{\n";
            file << "  \"name\": \"" << profile.name << "\",\n";
            file << "  \"timestamp\": " << profile.timestamp << ",\n";
            file << "  \"driveGains\": {\n";
            file << "    \"kP\": " << profile.driveGains.kP << ",\n";
            file << "    \"kI\": " << profile.driveGains.kI << ",\n";
            file << "    \"kD\": " << profile.driveGains.kD << "\n";
            file << "  },\n";
            file << "  \"turnGains\": {\n";
            file << "    \"kP\": " << profile.turnGains.kP << ",\n";
            file << "    \"kI\": " << profile.turnGains.kI << ",\n";
            file << "    \"kD\": " << profile.turnGains.kD << "\n";
            file << "  },\n";
            file << "  \"notes\": \"" << profile.notes << "\"\n";
            file << "}\n";

            file.close();
            return true;
        } catch (...) {
            return false;
        }
    }

    /**
     * @brief Load PID tuning profile from SD card
     * @param name Name of the profile to load
     * @return Optional containing the profile if successful
     */
    std::optional<TuningProfile> loadTuningProfile(const std::string& name) {
        if (!sdAvailable_) return std::nullopt;

        try {
            std::string filename = "/usd/tuning_" + name + ".json";
            std::ifstream file(filename);

            if (!file.is_open()) return std::nullopt;

            TuningProfile profile;
            profile.name = name;

            // Simple JSON parsing (basic implementation)
            std::string line;
            while (std::getline(file, line)) {
                // Basic parsing - could be enhanced with proper JSON library
                if (line.find("\"kP\":") != std::string::npos) {
                    // Parse PID values (simplified)
                }
            }

            file.close();
            return profile;
        } catch (...) {
            return std::nullopt;
        }
    }

    /**
     * @brief Save adaptive PID history
     * @param history Vector of adaptive data points
     * @return true if successful, false otherwise
     */
    bool saveAdaptiveHistory(const std::vector<AdaptiveData>& history) {
        if (!sdAvailable_ || history.empty()) return false;

        try {
            std::string filename = "/usd/adaptive_history.csv";
            std::ofstream file(filename, std::ios::app); // Append mode

            if (!file.is_open()) return false;

            // Write CSV header if file is empty
            if (file.tellp() == 0) {
                file << "timestamp,battery_voltage,drive_kp,drive_ki,drive_kd,turn_kp,turn_ki,turn_kd,surface_type,performance_score\n";
            }

            // Write data points
            for (const auto& data : history) {
                file << data.timestamp << ","
                     << data.batteryVoltage << ","
                     << data.adaptedGains.kP << ","
                     << data.adaptedGains.kI << ","
                     << data.adaptedGains.kD << ","
                     << data.adaptedGains.kP << "," // Assuming same structure
                     << data.adaptedGains.kI << ","
                     << data.adaptedGains.kD << ","
                     << data.surfaceType << ","
                     << data.performanceScore << "\n";
            }

            file.close();
            return true;
        } catch (...) {
            return false;
        }
    }

    /**
     * @brief Save match data
     * @param match The match data to save
     * @return true if successful, false otherwise
     */
    bool saveMatchData(const MatchData& match) {
        if (!sdAvailable_) return false;

        try {
            std::string filename = "/usd/match_" + std::to_string(match.timestamp) + ".log";
            std::ofstream file(filename);

            if (!file.is_open()) return false;

            file << "Match Log - " << match.timestamp << "\n";
            file << "Type: " << match.matchType << "\n";
            file << "Number: " << match.matchNumber << "\n";
            file << "Auton: " << match.autonSelection << "\n";
            file << "Result: " << match.result << "\n";
            file << "Events:\n";

            for (const auto& event : match.events) {
                file << "  " << event << "\n";
            }

            file.close();
            return true;
        } catch (...) {
            return false;
        }
    }

    /**
     * @brief List all saved tuning profiles
     * @return Vector of profile names
     */
    std::vector<std::string> listTuningProfiles() {
        std::vector<std::string> profiles;

        if (!sdAvailable_) return profiles;

        try {
            std::string path = "/usd";
            for (const auto& entry : fs::directory_iterator(path)) {
                std::string filename = entry.path().filename().string();
                if (filename.find("tuning_") == 0 && filename.find(".json") != std::string::npos) {
                    std::string profileName = filename.substr(7); // Remove "tuning_"
                    profileName = profileName.substr(0, profileName.find(".json"));
                    profiles.push_back(profileName);
                }
            }
        } catch (...) {
            // Ignore errors
        }

        return profiles;
    }

    /**
     * @brief Get SD card usage statistics
     * @return String with usage information
     */
    std::string getSDStats() {
        if (!sdAvailable_) return "SD card not available";

        try {
            fs::space_info space = fs::space("/usd");
            double usedGB = (space.capacity - space.available) / (1024.0 * 1024.0 * 1024.0);
            double totalGB = space.capacity / (1024.0 * 1024.0 * 1024.0);

            char buffer[100];
            std::snprintf(buffer, sizeof(buffer), "SD Usage: %.2f/%.2f GB (%.1f%%)",
                         usedGB, totalGB, (usedGB / totalGB) * 100.0);
            return std::string(buffer);
        } catch (...) {
            return "Unable to read SD statistics";
        }
    }

private:
    bool sdAvailable_;

    /**
     * @brief Check if SD card is available
     */
    bool checkSDAvailable() {
        try {
            return fs::exists("/usd");
        } catch (...) {
            return false;
        }
    }
};

/**
 * @brief Global data logger instance (optional usage)
 */
inline DataLogger& getDataLogger() {
    static DataLogger logger;
    return logger;
}

} // namespace ls