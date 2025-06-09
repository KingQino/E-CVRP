//
// Created by Yinghao Qin on 08/06/2025.
//

#include "command_line.hpp"
#include <stdexcept>
#include <unistd.h>   // realpath
#include <filesystem>
#include <iostream>
#include <cstdlib>

namespace fs = std::filesystem;

CommandLine::CommandLine(const int argc, char* argv[]) {
    try {
        const fs::path exec_path = fs::canonical(argv[0]);  // get the absolute path of the executable
        const fs::path project_root = exec_path.parent_path().parent_path();  // assume the project root is two levels up

        arguments["root_path"] = project_root.string();
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error resolving path: " << e.what() << '\n';
        std::exit(1);
    }

    for (int i = 1; i < argc; i += 2) {
        if (i + 1 < argc) { // Ensure there's a value after the key
            std::string key = argv[i];
            const std::string value = argv[i + 1];

            // Remove leading '-' or '--' if present
            if (key[0] == '-') key.erase(0, 1);
            if (key[0] == '-') key.erase(0, 1);

            arguments[key] = value;
        } else {
            std::cerr << "Warning: Missing value for argument '" << argv[i] << "'" << std::endl;
        }
    }

    if (arguments.count("help") || arguments.count("h")) {
        display_help();
        std::exit(0);
    }
}

void CommandLine::parse_parameters(Parameters& params) const {
    try {
        params.k_data_path = (fs::path(get_string("root_path", "..")) / "data").string();
        params.k_stats_path = (fs::path(get_string("root_path", "..")) / "stats").string();
        params.algorithm = string_to_algorithm(get_string("alg", "Lahc"));
        params.instance = get_string("ins", params.instance);
        params.enable_logging = get_bool("log", params.enable_logging);
        params.stop_criteria = get_int("stp", params.stop_criteria);
        params.enable_multithreading = get_bool("mth", params.enable_multithreading);
        params.seed = get_int("seed", params.seed);
        params.history_length = get_int("his_len", params.history_length);
        params.max_search_depth = get_int("max_depth", params.max_search_depth);
        params.low_opt_trigger_threshold = get_double("low_thresh", params.low_opt_trigger_threshold);
        params.low_opt_trigger_margin = get_double("low_margin", params.low_opt_trigger_margin);
        params.T0 = get_double("t0", params.T0);
        params.alpha = get_double("alpha", params.alpha);
        params.max_neigh_attempts = get_int("neigh_attempts", params.max_neigh_attempts);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        display_help();
        std::exit(1);
    }

    std::cout << "[INFO] Default data path: " << params.k_data_path << "\n";
    std::cout << "[INFO] Default stats path: " << params.k_stats_path << "\n";
}

// Display help message
void CommandLine::display_help() {
    std::cout << "--------------------------------------------------- Parameters Instruction  ---------------------------------------------------" << std::endl;
    std::cout << "Usage: ./Run [options]\n"
              << "Options:\n"
              << "  -alg [enum]                  : Algorithm name (e.g., Cbma, Lahc, Sga)\n"
              << "  -ins [filename]              : Problem instance filename\n"
              << "  -log [0|1]                   : Enable logging (default: 0)\n"
              << "  -stp [0|1|2]                 : Stopping criteria, 0: max-evals, 1: max-time, 2: obj-converge (default: 0)\n"
              << "  -mth [0|1]                   : Enable multi-threading (default: 1)\n"
              << "  -seed [int]                  : Random seed (default: 0)\n"
              << "  -his_len [int]               : LAHC history length (default: 5000)\n"
              << "  -max_depth [int]             : LAHC max search depth (default: 25)\n"
              << "  -low_thresh [double]         : LAHC threshold for triggering lower optimisation, [0, 1] (default: 0.3)\n"
              << "  -low_margin [double]         : LAHC margin related to the best upper cost for lower optimisation, >= 1.0 (default: 1.10)\n"
              << "  -rt_mul [int]                : Runtime multiplier (default: 1)\n"
              << "  -nb_granular [int]           : Granular search parameter (default: 20)\n"
              << "  -is_hard_constraint [0|1]    : Whether to use hard constraint (default: 1)\n"
              << "  -is_duration_constraint [0|1]: Whether to consider duration constraint (default: 0)\n"
              << "  -neigh_attempts [int]        : Maximum attempts for neighbourhood exploration (default: 10000)\n"
              << "  -t0 [double]                 : Initial temperature for simulated annealing (default: 30.0)\n"
              << "  -alpha [double]              : Cooling rate for simulated annealing (default: 0.98)\n"
              << "  -min_win [int]               : Minimum window size for recent deltas (default: 20)\n"
              << "  -max_win [int]               : Maximum window size for recent deltas (default: 500)\n"
              << "  -win_k [double]              : k value for dynamic window size calculation (default: 0.5)\n";
    std::cout << "-------------------------------------------------------------------------------------------------------------------------------" << std::endl;
}

int CommandLine::get_int(const std::string& key, const int default_value) const {
    if (const auto it = arguments.find(key); it != arguments.end()) {
        try {
            return std::stoi(it->second);
        } catch (const std::exception&) {
            std::cerr << "Error: Invalid integer value for argument '" << key << "'. Using default: " << default_value << std::endl;
        }
    }
    return default_value;
}

double CommandLine::get_double(const std::string& key, const double default_value) const {
    if (const auto it = arguments.find(key); it != arguments.end()) {
        try {
            return std::stod(it->second);
        } catch (const std::exception&) {
            std::cerr << "Error: Invalid double value for argument '" << key << "'. Using default: " << default_value << std::endl;
        }
    }
    return default_value;
}

std::string CommandLine::get_string(const std::string& key, const std::string& default_value) const {
    const auto it = arguments.find(key);
    return (it != arguments.end()) ? it->second : default_value;
}

bool CommandLine::get_bool(const std::string& key, const bool default_value) const {
    if (const auto it = arguments.find(key); it != arguments.end()) {
        const std::string val = it->second;
        return (val == "1" || val == "true" || val == "yes");
    }
    return default_value;
}

void CommandLine::print_arguments() const {
    std::cout << "Parsed Command-Line Arguments:\n";
    for (const auto& [key, value] : arguments) {
        std::cout << "  " << key << " = " << value << std::endl;
    }
}

std::string CommandLine::to_lowercase(const std::string &str) {
    std::string lower_str = str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);
    return lower_str;
}

Algorithm CommandLine::string_to_algorithm(const std::string& algo_str) {
    const std::string lower_algo = to_lowercase(algo_str);

    if (lower_algo == "cbma") return Algorithm::CBMA;
    if (lower_algo == "lahc") return Algorithm::LAHC;

    std::cerr << "Warning: Unknown algorithm '" << algo_str << "', defaulting to Lahc.\n";
    return Algorithm::LAHC; // Default to Lahc if input is invalid
}
