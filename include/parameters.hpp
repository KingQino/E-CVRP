//
// Created by Yinghao Qin on 08/06/2025.
//

#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include<iostream>
#include<string>

using namespace std;

enum class Algorithm { CBMA, LAHC};

struct Parameters {
    // Running parameters
    string k_data_path;         // Path to the data directory
    string k_stats_path;        // Path to the statistics directory
    Algorithm algorithm;        // Algorithm name
    string instance;            // Problem instance name
    bool enable_logging;        // Enable logging
    int stop_criteria;          // Stopping criteria (e.g., max evaluations used)
    bool enable_multithreading; // Enable multi-threading
    int seed;                   // Random seed

    // Algorithm parameters
    int history_length;                 // LAHC: history length
    int max_search_depth;               // LAHC: max_search_depth determines how many times the neighbourhood (random) will be explored
    double low_opt_trigger_threshold;   // LAHC: The threshold to trigger the lower-level optimisation
    double T0;                          // CBMA: Initial temperature for simulated annealing (if applicable)
    double alpha;                       // CBMA: Cooling rate for simulated annealing (if applicable)
    int max_neigh_attempts;             // CBMA: Maximum attempts for neighbourhood exploration

    // Constructor: Initializes default values
    Parameters() :
            algorithm(Algorithm::LAHC),
            instance("E-n22-k4.evrp"),
            enable_logging(false),
            stop_criteria(0),
            enable_multithreading(false),
            seed(0) {

        history_length = 5'000;
        max_search_depth = 200;
        low_opt_trigger_threshold = 0.3;
        T0 = 30.0;
        alpha = 0.98;
        max_neigh_attempts = 10'000;
    }
};

#endif //PARAMETERS_HPP
