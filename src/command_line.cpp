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
#include <cmath>

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

LocalSearchAction parse_ma_static_action(const std::string& value) {
    const std::string v = CommandLine::to_lowercase(value);
    if (v == "weak") return LocalSearchAction::Weak;
    if (v == "medium") return LocalSearchAction::Medium;
    if (v == "strong") return LocalSearchAction::Strong;
    throw std::invalid_argument("ma_static_action must be one of: weak, medium, strong");
}

LocalSearchPolicy parse_ma_ls_policy(const std::string& value) {
    const std::string v = CommandLine::to_lowercase(value);
    if (v == "static") return LocalSearchPolicy::Static;
    if (v == "random") return LocalSearchPolicy::RandomMixed;
    if (v == "matched_random") return LocalSearchPolicy::MatchedRandom;
    if (v == "online") return LocalSearchPolicy::OnlineIndividual;
    throw std::invalid_argument("ma_ls_policy must be one of: static, random, matched_random, online");
}

LocalSearchDepthProfile parse_ma_ls_depth(const std::string& value) {
    const std::string v = CommandLine::to_lowercase(value);
    if (v == "d1") return LocalSearchDepthProfile::D1;
    if (v == "d2") return LocalSearchDepthProfile::D2;
    if (v == "d3") return LocalSearchDepthProfile::D3;
    if (v == "d4") return LocalSearchDepthProfile::D4;
    if (v == "d5") return LocalSearchDepthProfile::D5;
    if (v == "d6") return LocalSearchDepthProfile::D6;
    throw std::invalid_argument("ma_ls_depth must be one of: d1, d2, d3, d4, d5, d6");
}

void require_probability(const double value, const std::string& name) {
    if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
        throw std::invalid_argument(name + " must be in [0, 1]");
    }
}

// Same constraints as the original MA implementation (Parameters::validate).
void validate_ma_parameters(const Parameters& params) {
    if (params.ma_pop_size < 2) {
        throw std::invalid_argument("ma_pop_size must be at least 2");
    }
    require_probability(params.ma_mutation_prob, "ma_mutation_prob");
    require_probability(params.ma_mutation_ind_prob, "ma_mutation_ind_prob");
    require_probability(params.ma_parent_pool_ratio, "ma_parent_pool_ratio");
    require_probability(params.ma_verified_upper_ratio, "ma_verified_upper_ratio");
    require_probability(params.ma_pure_immigrant_ratio, "ma_pure_immigrant_ratio");
    require_probability(params.ma_unrestricted_credit_rho, "ma_unrestricted_credit_rho");
    if (params.ma_parent_pool_ratio * static_cast<double>(params.ma_pop_size) < 5.0 - 1e-12) {
        throw std::invalid_argument("ma_parent_pool_ratio * ma_pop_size must be at least 5");
    }
    if (params.ma_verified_upper_ratio + params.ma_pure_immigrant_ratio > 0.25 + 1e-12) {
        throw std::invalid_argument("ma_verified_upper_ratio + ma_pure_immigrant_ratio must not exceed 0.25");
    }
    const int parent_pool_size = static_cast<int>(std::ceil(params.ma_parent_pool_ratio * static_cast<double>(params.ma_pop_size)));
    if (params.ma_tournament_size < 1 || params.ma_tournament_size > parent_pool_size) {
        throw std::invalid_argument("ma_tournament_size must be in [1, parent-pool size]");
    }
    if (!std::isfinite(params.low_opt_trigger_margin) || params.low_opt_trigger_margin < 1.0) {
        throw std::invalid_argument("low_margin (MA gamma) must be finite and at least 1.0");
    }
    if (!std::isfinite(params.ma_action_cost_weight) || params.ma_action_cost_weight < 0.0) {
        throw std::invalid_argument("ma_action_cost_weight must be finite and non-negative");
    }
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
        // LAHC (B-LAHC); the unprefixed names are accepted as legacy aliases.
        params.lahc_history_length = get_int_alias("lahc_his_len", "his_len", params.lahc_history_length);
        params.lahc_max_attempts = get_int_alias("lahc_max_attempts", "max_attempts", params.lahc_max_attempts);
        params.low_opt_trigger_margin = get_double("low_margin", params.low_opt_trigger_margin);
        params.lahc_noise_lb = get_double_alias("lahc_noise_lb", "noise_lb", params.lahc_noise_lb);
        params.lahc_noise_ub = get_double_alias("lahc_noise_ub", "noise_ub", params.lahc_noise_ub);
        // MA (BA-BMA)
        params.ma_pop_size = get_int("ma_pop_size", params.ma_pop_size);
        params.ma_mutation_prob = get_double("ma_mutation_prob", params.ma_mutation_prob);
        params.ma_mutation_ind_prob = get_double("ma_mutation_ind_prob", params.ma_mutation_ind_prob);
        params.ma_tournament_size = get_int("ma_tournament_size", params.ma_tournament_size);
        params.ma_ls_policy = parse_ma_ls_policy(get_string("ma_ls_policy", local_search_policy_name(params.ma_ls_policy)));
        params.ma_static_action = parse_ma_static_action(get_string("ma_static_action", local_search_action_name(params.ma_static_action)));
        params.ma_static_unrestricted = get_bool("ma_static_unrestricted", params.ma_static_unrestricted);
        params.ma_parent_pool_ratio = get_double("ma_parent_pool_ratio", params.ma_parent_pool_ratio);
        params.ma_verified_upper_ratio = get_double("ma_verified_upper_ratio", params.ma_verified_upper_ratio);
        params.ma_pure_immigrant_ratio = get_double("ma_pure_immigrant_ratio", params.ma_pure_immigrant_ratio);
        params.ma_unrestricted_credit_rho = get_double("ma_unrestricted_credit_rho", params.ma_unrestricted_credit_rho);
        params.ma_ls_depth = parse_ma_ls_depth(get_string("ma_ls_depth", local_search_depth_profile_name(params.ma_ls_depth)));
        params.ma_action_cost_weight = get_double("ma_action_cost_weight", params.ma_action_cost_weight);
        // TwoStageAlns; the basin_* names are accepted as legacy aliases.
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
        if (params.algorithm == Algorithm::MA) {
            validate_ma_parameters(params);
        }
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
              << "  -alg [enum]                          : Algorithm name: lahc (B-LAHC), ma (BA-BMA) or twostagealns (case-insensitive)\n"
              << "  -ins [filename]                      : Problem instance filename\n"
              << "  -log [0|1]                           : Enable logging (default: 0)\n"
              << "  -stp [0|1]                           : Stopping criteria, 0: max-evals, 1: max-time (default: 0)\n"
              << "  -mth [0|1]                           : Enable multi-threading (default: 1)\n"
              << "  -exp [0|1]                           : Activate experimental mode (default: 0)\n"
              << "  -refine [0|1]                        : Enable final follower refinement (default: 1)\n"
              << "  -seed [int]                          : Random seed (default: 0)\n"
              << "  -low_margin [double]                 : LAHC/MA upper-cost margin to trigger follower optimisation (MA: gamma), >= 1.0 (default: 1.05)\n"
              << "  -lahc_his_len [int]                  : LAHC history length (default: 5723)\n"
              << "  -lahc_max_attempts [int]             : LAHC max attempts for LeaderSingle (default: 60)\n"
              << "  -lahc_noise_lb [double]              : LAHC history-noise lower bound (default: 0.95)\n"
              << "  -lahc_noise_ub [double]              : LAHC history-noise upper bound (default: 1.05)\n"
              << "  -ma_pop_size [int]                   : MA population size (default: 100)\n"
              << "  -ma_mutation_prob [double]           : MA probability that an offspring is mutated (default: 0.5)\n"
              << "  -ma_mutation_ind_prob [double]       : MA per-gene mutation probability (default: 0.2)\n"
              << "  -ma_tournament_size [int]            : MA upper-parent tournament size (default: 2)\n"
              << "  -ma_ls_policy [enum]                 : MA local-search action allocation: static|random|matched_random|online (default: online)\n"
              << "  -ma_static_action [enum]             : MA action of the static policy: weak|medium|strong (default: strong)\n"
              << "  -ma_static_unrestricted [0|1]        : MA static policy runs unrestricted RVND to a local optimum (default: 0)\n"
              << "  -ma_parent_pool_ratio [double]       : MA parent-pool size / population size (default: 0.10)\n"
              << "  -ma_verified_upper_ratio [double]    : MA share of x* x parent-pool offspring, x* = best complete solution so far (default: 0.05)\n"
              << "  -ma_pure_immigrant_ratio [double]    : MA share of random-immigrant offspring (default: 0.10)\n"
              << "  -ma_unrestricted_credit_rho [double] : MA Elite Unrestricted credit per ordinary LS distance call, online policy only (default: 0.20)\n"
              << "  -ma_ls_depth [d1..d6]                : MA local-search depth profile (default: d5)\n"
              << "  -ma_action_cost_weight [double]      : MA weight of the cost term in the online action score (default: 0.02)\n"
              << "  -alns_start_dev [double]             : TwoStageAlns initial record-deviation gap (default: 0.0167)\n"
              << "  -alns_decay [double]                 : TwoStageAlns adaptive-weight decay (default: 0.99)\n"
              << "  -alns_score_acc [double]             : TwoStageAlns score for accepted solutions (default: 2.0)\n"
              << "  -alns_score_imp [double]             : TwoStageAlns score for improving solutions (default: 4.0)\n"
              << "  -alns_score_best [double]            : TwoStageAlns score for new best upper solutions (default: 10.0)\n"
              << "  -alns_min_destroy_ratio [double]     : TwoStageAlns minimum relative destroy size (default: 0.10)\n"
              << "  -alns_max_destroy_ratio [double]     : TwoStageAlns maximum relative destroy size (default: 0.40)\n"
              << "  -alns_min_destroy_abs [int]          : TwoStageAlns minimum absolute destroy size (default: 5)\n"
              << "  -alns_max_destroy_abs [int]          : TwoStageAlns maximum absolute destroy size (default: 150)\n"
              << "  -alns_geo_bias [double]              : TwoStageAlns geo destroy bias (default: 5.0)\n"
              << "  -alns_repair_bias [double]           : TwoStageAlns randomized regret factor (default: 1.5)\n";
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
    if (lower_algo == "ma") return Algorithm::MA;
    if (lower_algo == "twostagealns" || lower_algo == "two_stage_alns") return Algorithm::TWOSTAGEALNS;

    throw std::invalid_argument(
        "Unknown algorithm '" + algo_str + "'. Supported algorithms: lahc, ma, twostagealns.");
}
