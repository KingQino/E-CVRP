//
// Created by Yinghao Qin on 08/06/2025.
//

#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include<iostream>
#include<string>

enum class Algorithm { LAHC, TWOSTAGEALNS };

struct Parameters {
    // Running parameters
    std::string k_data_path;         // Path to the data directory
    std::string k_stats_path;        // Path to the statistics directory
    Algorithm algorithm;        // Algorithm name
    std::string instance;            // Problem instance name
    bool enable_logging;        // Enable logging
    int stop_criteria;          // Stopping criteria (e.g., max evaluations used)
    bool enable_multithreading; // Enable multi-threading
    bool activate_exp_mode;     // Activate experimental mode (default: false)
    bool enable_final_refinement; // Enable final follower refinement (default: true)
    int seed;                   // Random seed

    // LAHC parameters (LAHC uses LeaderSingle only)
    int history_length;                 // LAHC: history length
    int max_attempts;                   // LAHC: max_attempts determines how many times the neighbourhood (random) will be explored
    double low_opt_trigger_margin;      // LAHC: margin to trigger lower-level optimisation, >= 1.0
    double noise_lb;                    // LAHC: The lower bound of noise range for history list value
    double noise_ub;                    // LAHC: The upper bound of noise range for history list value

    // TwoStageAlns parameters
    double alns_start_deviation;        // TwoStageAlns: initial record-deviation acceptance gap
    double alns_decay;                  // TwoStageAlns: adaptive weight decay
    double alns_score_accepted;         // TwoStageAlns: score for accepted solution
    double alns_score_improved;         // TwoStageAlns: score for improving solution
    double alns_score_global_best;      // TwoStageAlns: score for new best upper solution
    double alns_min_destroy_ratio;      // TwoStageAlns: minimum relative destroy size
    double alns_max_destroy_ratio;      // TwoStageAlns: maximum relative destroy size
    int alns_min_destroy_abs;           // TwoStageAlns: minimum absolute destroy size
    int alns_max_destroy_abs;           // TwoStageAlns: maximum absolute destroy size
    double alns_geo_destroy_rand_factor;// TwoStageAlns: geo destroy bias
    double alns_repair_rand_factor;     // TwoStageAlns: randomized regret repair factor

    // Constructor: Initializes default values
    Parameters() :
            algorithm(Algorithm::LAHC),
            instance("E-n22-k4.evrp"),
            enable_logging(false),
            stop_criteria(0),
            enable_multithreading(false),
            activate_exp_mode(false),
            enable_final_refinement(true),
            seed(0) {

        history_length = 5'723;
        max_attempts = 60;
        low_opt_trigger_margin = 1.05;   // Relative cost margin (≥ 1.0) within which the candidate solution must fall to trigger lower-level optimisation
        noise_lb = 0.95;
        noise_ub = 1.05;

        alns_start_deviation = 0.0167;
        alns_decay = 0.99;
        alns_score_accepted = 2.0;
        alns_score_improved = 4.0;
        alns_score_global_best = 10.0;
        alns_min_destroy_ratio = 0.10;
        alns_max_destroy_ratio = 0.40;
        alns_min_destroy_abs = 5;
        alns_max_destroy_abs = 150;
        alns_geo_destroy_rand_factor = 5.0;
        alns_repair_rand_factor = 1.5;
    }
};

#endif //PARAMETERS_HPP
