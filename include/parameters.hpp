//
// Created by Yinghao Qin on 08/06/2025.
//

#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include<iostream>
#include<string>

#include "ma_types.hpp"

enum class Algorithm { LAHC, MA, TWOSTAGEALNS };

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

    // STN logging (stn-logging branch): passive trajectory logging for STNs-v2, off by default
    bool enable_stn_logging;    // write stn_raw.<instance>.csv
    double stn_grid;            // observation grid as a fraction of the max-evals budget
    bool stn_solutions;         // also write stn_solutions.<instance>.csv (signature -> canonical routes)

    // LAHC parameters: bilevel late acceptance hill climbing, B-LAHC (LAHC uses LeaderSingle only)
    int lahc_history_length;            // LAHC: history length
    int lahc_max_attempts;              // LAHC: how many times the (random) neighbourhood is explored per move
    double low_opt_trigger_margin;      // LAHC/MA: margin to trigger lower-level optimisation, >= 1.0 (MA: gamma)
    double lahc_noise_lb;               // LAHC: lower bound of the noise range for history list values
    double lahc_noise_ub;               // LAHC: upper bound of the noise range for history list values

    // MA parameters: budget-aware bilevel memetic algorithm, BA-BMA
    int ma_pop_size;                    // MA: population size
    double ma_mutation_prob;            // MA: probability that an offspring enters mutation
    double ma_mutation_ind_prob;        // MA: per-gene probability inside the mutation operator
    int ma_tournament_size;             // MA: upper-parent tournament size
    LocalSearchPolicy ma_ls_policy;     // MA: local-search allocation policy
    LocalSearchAction ma_static_action; // MA: action applied to every individual by the static policy
    bool ma_static_unrestricted;        // MA: static policy runs unrestricted RVND instead (original BMA baseline)
    double ma_parent_pool_ratio;        // MA: parent-pool size relative to population size
    double ma_verified_upper_ratio;     // MA: share of x* x parent-pool offspring (x*: best complete solution so far)
    double ma_pure_immigrant_ratio;     // MA: share of pure random-immigrant offspring
    double ma_unrestricted_credit_rho;  // MA: Elite Unrestricted credit earned per ordinary local-search distance call (rho)
    LocalSearchDepthProfile ma_ls_depth;// MA: coupled weak/medium/strong depth profile
    double ma_action_cost_weight;       // MA: weight of the predicted-cost term in the online learner's action score

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
            seed(0),
            enable_stn_logging(false),
            stn_grid(0.0001),
            stn_solutions(true) {

        lahc_history_length = 5'723;
        lahc_max_attempts = 60;
        low_opt_trigger_margin = 1.05;   // Relative cost margin (>= 1.0) within which the candidate solution must fall to trigger lower-level optimisation
        lahc_noise_lb = 0.95;
        lahc_noise_ub = 1.05;

        ma_pop_size = 100;
        ma_mutation_prob = 0.5;
        ma_mutation_ind_prob = 0.2;
        ma_tournament_size = 2;
        ma_ls_policy = LocalSearchPolicy::OnlineIndividual;
        ma_static_action = LocalSearchAction::Strong;
        ma_static_unrestricted = false;
        ma_parent_pool_ratio = 0.10;
        ma_verified_upper_ratio = 0.05;
        ma_pure_immigrant_ratio = 0.10;
        ma_unrestricted_credit_rho = 0.20;
        ma_ls_depth = LocalSearchDepthProfile::D5;
        ma_action_cost_weight = 0.02;

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
