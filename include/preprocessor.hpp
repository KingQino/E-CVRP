//
// Created by Yinghao Qin on 08/06/2025.
//

#ifndef PREPROCESSOR_HPP
#define PREPROCESSOR_HPP

#include "case.hpp"
#include "parameters.hpp"
#include <random>


#define EPSILON 0.000'01       // Precision parameter, used to avoid numerical instabilities

// Hash function for pairs of route indices
struct PairHash {
    std::size_t operator()(const std::pair<int,int>& p) const noexcept {
        const uint64_t a = static_cast<uint32_t>(p.first);
        const uint64_t b = static_cast<uint32_t>(p.second);
        return static_cast<std::size_t>((a << 32) | b);
    }
};

class Preprocessor {
    static const int MAX_EVALUATION_FACTOR;

public:
    const Case& c;
    const Parameters& params;

    // Stop criteria
    double max_evals_{};            // the max number of evaluations can be used
    int max_exec_time_{};           // the max execution time of the algorithm
    int max_no_improvement_count_{};// the max number of iterations without improvement

    int max_demand_{};              // the max customer demand among all customers
    int total_demand_{};            // the total customer demand
    int route_cap_{};               // the capacity of routes
    int node_cap_{};                // the capacity of node per route, the number of customers + 2
    double max_cruise_distance_{};  // the max cruise distance the vehicle can travel without recharging the battery
    double max_distance_{};         // the longest arc distance between any two nodes

    std::vector<int> customer_ids_;      // the id of customers
    std::vector<int> station_ids_;       // the id of charging stations

    std::vector<std::vector<int>> sorted_nearby_customers_;   // For Hien's clustering usage only. For each customer, a list of customer nodes from near to far, e.g., {index 1: [5,3,2,6], index 2: [], ...}
    std::vector<std::vector<int>> best_station_;              // For each pair of customers, the best station to visit, i.e., the station that minimizes the extra cost

    Preprocessor(const Case& c, const Parameters& params);

    [[nodiscard]] int get_best_station(int from, int to) const;
    [[nodiscard]] int get_best_and_feasible_station(int from, int to, double max_dis) const; // the station within allowed max distance from "from", and min dis[from][s]+dis[to][s]

};

#endif //PREPROCESSOR_HPP
