//
// Created by Yinghao Qin on 08/06/2025.
//

#include "command_line.hpp"
#include <stdexcept>
#include <unistd.h>   // realpath
#include <filesystem>
#include <iostream>
#include <cstdlib>
#include <algorithm>

namespace fs = std::filesystem;

namespace {

std::string normalize_key(std::string key) {
    while (!key.empty() && key.front() == '-') {
        key.erase(0, 1);
    }
    return key;
}

bool is_help_flag(const std::string& raw_key) {
    const std::string key = normalize_key(raw_key);
    return key == "h" || key == "help";
}

} // namespace

CommandLine::CommandLine(const int argc, char* argv[]) {
    try {
        const fs::path exec_path = fs::canonical(argv[0]);  // get the absolute path of the executable
        const fs::path project_root = exec_path.parent_path().parent_path();  // assume the project root is two levels up

        arguments["root_path"] = project_root.string();
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error resolving path: " << e.what() << '\n';
        std::exit(1);
    }

    for (int i = 1; i < argc;) {
        const std::string raw_key = argv[i];
        if (is_help_flag(raw_key)) {
            display_help();
            std::exit(0);
        }

        if (i + 1 >= argc) {
            std::cerr << "Warning: Missing value for argument '" << argv[i] << "'" << std::endl;
            ++i;
            continue;
        }

        arguments[normalize_key(raw_key)] = argv[i + 1];
        i += 2;
    }
}

void CommandLine::parse_parameters(Parameters& params) const {
    try {
        auto get_double_alias = [&](const std::string& key, const std::string& legacy_key, const double default_value) {
            return arguments.find(key) != arguments.end() ? get_double(key, default_value)
                                                          : get_double(legacy_key, default_value);
        };
        auto get_int_alias = [&](const std::string& key, const std::string& legacy_key, const int default_value) {
            return arguments.find(key) != arguments.end() ? get_int(key, default_value)
                                                          : get_int(legacy_key, default_value);
        };

        params.k_data_path = (fs::path(get_string("root_path", "..")) / "data").string();
        params.k_stats_path = (fs::path(get_string("root_path", "..")) / "stats").string();
        params.algorithm = string_to_algorithm(get_string("alg", "Lahc"));
        params.instance = get_string("ins", params.instance);
        params.enable_logging = get_bool("log", params.enable_logging);
        params.stop_criteria = get_int("stp", params.stop_criteria);
        params.enable_multithreading = get_bool("mth", params.enable_multithreading);
        params.activate_exp_mode = get_bool("exp", params.activate_exp_mode);
        params.enable_final_refinement = get_bool("refine", params.enable_final_refinement);
        params.seed = get_int("seed", params.seed);
        params.history_length = get_int("his_len", params.history_length);
        params.max_attempts = get_int("max_attempts", params.max_attempts);
        params.low_opt_trigger_margin = get_double("low_margin", params.low_opt_trigger_margin);
        params.noise_lb = get_double("noise_lb", params.noise_lb);
        params.noise_ub = get_double("noise_ub", params.noise_ub);
        params.alns_start_deviation = get_double_alias("alns_start_dev", "basin_start_dev", params.alns_start_deviation);
        params.alns_decay = get_double_alias("alns_decay", "basin_decay", params.alns_decay);
        params.alns_score_accepted = get_double_alias("alns_score_acc", "basin_score_acc", params.alns_score_accepted);
        params.alns_score_improved = get_double_alias("alns_score_imp", "basin_score_imp", params.alns_score_improved);
        params.alns_score_global_best = get_double_alias("alns_score_best", "basin_score_best", params.alns_score_global_best);
        params.alns_min_destroy_ratio = get_double_alias("alns_min_destroy_ratio", "basin_min_destroy_ratio", params.alns_min_destroy_ratio);
        params.alns_max_destroy_ratio = get_double_alias("alns_max_destroy_ratio", "basin_max_destroy_ratio", params.alns_max_destroy_ratio);
        params.alns_min_destroy_abs = get_int_alias("alns_min_destroy_abs", "basin_min_destroy_abs", params.alns_min_destroy_abs);
        params.alns_max_destroy_abs = get_int_alias("alns_max_destroy_abs", "basin_max_destroy_abs", params.alns_max_destroy_abs);
        params.alns_geo_destroy_rand_factor = get_double_alias("alns_geo_bias", "basin_geo_bias", params.alns_geo_destroy_rand_factor);
        params.alns_repair_rand_factor = get_double_alias("alns_repair_bias", "basin_repair_bias", params.alns_repair_rand_factor);
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
              << "  -alg [enum]                  : Algorithm name (Lahc or TwoStageAlns)\n"
              << "  -ins [filename]              : Problem instance filename\n"
              << "  -log [0|1]                   : Enable logging (default: 0)\n"
              << "  -stp [0|1]                   : Stopping criteria, 0: max-evals, 1: max-time (default: 0)\n"
              << "  -mth [0|1]                   : Enable multi-threading (default: 1)\n"
              << "  -exp [0|1]                   : Activate experimental mode (default: 0)\n"
              << "  -refine [0|1]                : Enable final follower refinement (default: 1)\n"
              << "  -seed [int]                  : Random seed (default: 0)\n"
              << "  -his_len [int]               : LAHC history length (default: 5723)\n"
              << "  -max_attempts [int]          : LAHC max attempts for LeaderSingle (default: 60)\n"
              << "  -low_margin [double]         : LAHC upper-cost margin to trigger follower optimisation, >= 1.0 (default: 1.05)\n"
              << "  -noise_lb [double]           : LAHC history-noise lower bound (default: 0.95)\n"
              << "  -noise_ub [double]           : LAHC history-noise upper bound (default: 1.05)\n"
              << "  -alns_start_dev [double]     : TwoStageAlns initial record-deviation gap (default: 0.0167)\n"
              << "  -alns_decay [double]         : TwoStageAlns adaptive-weight decay (default: 0.99)\n"
              << "  -alns_score_acc [double]     : TwoStageAlns score for accepted solutions (default: 2.0)\n"
              << "  -alns_score_imp [double]     : TwoStageAlns score for improving solutions (default: 4.0)\n"
              << "  -alns_score_best [double]    : TwoStageAlns score for new best upper solutions (default: 10.0)\n"
              << "  -alns_min_destroy_ratio [double] : TwoStageAlns minimum relative destroy size (default: 0.10)\n"
              << "  -alns_max_destroy_ratio [double] : TwoStageAlns maximum relative destroy size (default: 0.40)\n"
              << "  -alns_min_destroy_abs [int]  : TwoStageAlns minimum absolute destroy size (default: 5)\n"
              << "  -alns_max_destroy_abs [int]  : TwoStageAlns maximum absolute destroy size (default: 150)\n"
              << "  -alns_geo_bias [double]      : TwoStageAlns geo destroy bias (default: 5.0)\n"
              << "  -alns_repair_bias [double]   : TwoStageAlns randomized regret factor (default: 1.5)\n";
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

    if (lower_algo == "lahc") return Algorithm::LAHC;
    if (lower_algo == "twostagealns" || lower_algo == "two_stage_alns") return Algorithm::TWOSTAGEALNS;

    throw std::invalid_argument(
        "Unknown algorithm '" + algo_str + "'. Supported algorithms: Lahc, TwoStageAlns.");
}
