//
// Created by Yinghao Qin on 08/06/2025.
//

#include "stats_interface.hpp"

Indicators StatsInterface::calculate_statistical_indicators(const std::vector<double>& data) {
    Indicators indicators;

    if (data.empty()) return indicators;

    indicators.size = data.size();
    indicators.min = *std::min_element(data.begin(), data.end());
    indicators.max = *std::max_element(data.begin(), data.end());
    const double sum = std::accumulate(data.begin(), data.end(), 0.0);
    indicators.avg = sum / static_cast<double>(indicators.size);

    if (indicators.size == 1) {
        indicators.std = 0.0;
    } else {
        const double variance = std::accumulate(data.begin(), data.end(), 0.0,
                                                [indicators](const double accum, const double value) {
                                                    return accum + (value - indicators.avg) * (value - indicators.avg);
                                                }) / static_cast<double>(indicators.size - 1);
        indicators.std = std::sqrt(variance);
    }

    return indicators;
}

void StatsInterface::calculate_statistical_indicators(const std::vector<double>& data, Indicators& indicators) {
    if (data.empty()) return;

    indicators.size = data.size();
    indicators.min = *std::min_element(data.begin(), data.end());
    indicators.max = *std::max_element(data.begin(), data.end());
    const double sum = std::accumulate(data.begin(), data.end(), 0.0);
    indicators.avg = sum / static_cast<double>(indicators.size);

    if (indicators.size == 1) {
        indicators.std = 0.0;
    } else {
        const double variance = std::accumulate(data.begin(), data.end(), 0.0,
                                          [indicators](const double accum, const double value) {
                                              return accum + (value - indicators.avg) * (value - indicators.avg);
                                          }) / static_cast<double>(indicators.size - 1);
        indicators.std = std::sqrt(variance);
    }
}

bool StatsInterface::create_directories_if_not_exists(const fs::path& directory_path) {
    if (!fs::exists(directory_path)) {
        try {
            fs::create_directories(directory_path);
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error creating directory: " << e.what() << std::endl;
            return false;
        }
    }
    return true;
}

void StatsInterface::stats_for_multiple_trials(const fs::path& file_path, const std::vector<double>& data) {
    std::ofstream log_stats(file_path);

    std::ostringstream oss;
    for (const auto& perf : data) {
        oss << std::fixed << std::setprecision(2) << perf << '\n';
    }

    Indicators indicators = calculate_statistical_indicators(data);
    oss << "Mean: " << indicators.avg << "\t\tStd Dev: " << indicators.std << '\n';
    oss << "Min : " << indicators.min << '\n';
    oss << "Max : " << indicators.max << '\n';

    log_stats << oss.str() << std::flush;
    log_stats.close();
}
