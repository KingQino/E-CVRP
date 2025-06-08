//
// Created by Yinghao Qin on 08/06/2025.
//

#ifndef STATS_INTERFACE_HPP
#define STATS_INTERFACE_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <string>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <numeric>

namespace fs = std::filesystem;
using namespace std;

struct Indicators {
    double min{};
    double max{};
    double avg{};
    double std{};
    std::size_t size{};
};


class StatsInterface {
public:
    std::chrono::high_resolution_clock::time_point start;
    std::chrono::duration<double> duration;

    std::ofstream log_evolution;
    std::ostringstream oss_row_evol;
    std::ofstream log_solution;

    static Indicators calculate_statistical_indicators(const std::vector<double>& datas);
    static void calculate_statistical_indicators(const std::vector<double>& data, Indicators& indicators);
    static bool create_directories_if_not_exists(const std::string& directory_path);
    static void stats_for_multiple_trials(const std::string& file_path, const std::vector<double>& data); // open a file, save the statistical info, and then close it
    virtual void open_log_for_evolution() = 0;  // open a file
    virtual void flush_row_into_evol_log() = 0; // flush the evolution info into the file
    virtual void close_log_for_evolution() = 0; // close the file
    virtual void save_log_for_solution() = 0;   // open a file, save the solution, and close it

    virtual ~StatsInterface() = default;
};

#endif //STATS_INTERFACE_HPP
