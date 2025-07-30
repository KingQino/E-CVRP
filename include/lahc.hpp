//
// Created by Yinghao Qin on 09/06/2025.
//

#ifndef LAHC_HPP
#define LAHC_HPP

#include "case.hpp"
#include "initializer.hpp"
#include "leader_lahc.hpp"
#include "follower.hpp"
#include "individual.hpp"
#include "heuristic_interface.hpp"
#include "stats_interface.hpp"

using namespace std;

class Lahc final : public HeuristicInterface, public StatsInterface {
public:
    bool enable_logging;                        // Whether to enable logging
    int stop_criteria;                          // Stop criteria for the algorithm

    long iter;                                  // Iteration counter I
    long idle_iter;                             // Idle iteration counter
    long history_length;                        // Parameter：LAHC history length Lh
    double num_successful_moves_per_history;    // the number of algorithm moves per history length iteration
    double ratio_successful_moves;              // the ratio of successful moves per history length iteration
    double low_opt_trigger_threshold;           // Parameter：The threshold to trigger the lower-level optimisation，in [0.0, 1.0]
    double low_opt_trigger_margin;              // Parameter：The margin to trigger the lower-level optimisation, >= 1.0
    vector<double> history_list;                // Lahc history list L, it holds the objetive values
    Indicators history_list_metrics;            // The statistical info of the history list
    Individual* current;                        // Current solution s
    double best_upper_cost;                     // The best upper-level cost found so far
    std::unique_ptr<Individual> global_best;    // Global best solution found so far
    std::uniform_real_distribution<> history_noise; // history list initialisation noise
    int restart_idx;                            // Restart count for the heuristic

    Initializer* initializer;
    LeaderLahc* leader;
    Follower* follower;

    Lahc(int seed, Case *instance, Preprocessor* preprocessor);
    ~Lahc() override;
    void run() override;
    void initialize_heuristic() override;
    void run_heuristic() override;
    void open_log_for_evolution() override;
    void close_log_for_evolution() override;
    void flush_row_into_evol_log() override;
    void save_log_for_solution() override;

};

#endif //LAHC_HPP
