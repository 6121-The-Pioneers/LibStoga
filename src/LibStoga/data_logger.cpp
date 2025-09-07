#include "LibStoga/data_logger.h"
#include <iostream>
#include <fstream>

namespace ls {

DataLogger::DataLogger() {
    sdAvailable_ = checkSDAvailable();
}

bool DataLogger::isSDAvailable() const {
    return sdAvailable_;
}

bool DataLogger::saveTuningProfile(const TuningProfile& profile) {
    if (!sdAvailable_) return false;
    // JSON functionality removed due to incompatibility
    return false;
}

std::optional<DataLogger::TuningProfile> DataLogger::loadTuningProfile(const std::string& name) {
    if (!sdAvailable_) return std::nullopt;
    // JSON functionality removed due to incompatibility
    return std::nullopt;
}

bool DataLogger::saveAdaptiveHistory(const std::vector<AdaptiveData>& history) {
    if (!sdAvailable_) return false;
    std::ofstream file("/usd/adaptive_history.csv");
    if (!file.is_open()) return false;
    file << "timestamp,batteryVoltage,kP,kI,kD,surfaceType,performanceScore\\n";
    for (const auto& data : history) {
        file << data.timestamp << "," << data.batteryVoltage << ","
             << data.adaptedGains.kP << "," << data.adaptedGains.kI << "," << data.adaptedGains.kD << ","
             << data.surfaceType << "," << data.performanceScore << "\\n";
    }
    file.close();
    return true;
}

bool DataLogger::saveMatchData(const MatchData& match) {
    if (!sdAvailable_) return false;
    // Implementation for saving match data
    return false;
}

std::vector<std::string> DataLogger::listTuningProfiles() {
    if (!sdAvailable_) return {};
    // Implementation for listing profiles
    return {};
}

std::string DataLogger::getSDStats() {
    if (!sdAvailable_) return "SD card not available.";
    // Implementation for getting SD stats
    return "Not implemented.";
}

bool DataLogger::checkSDAvailable() {
    std::ifstream file("/usd/test.txt");
    return file.good();
}

} // namespace ls
