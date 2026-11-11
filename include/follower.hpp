//
// Created by Yinghao Qin on 09/06/2025.
//

#ifndef FOLLOWER_HPP
#define FOLLOWER_HPP

#include "case.hpp"
#include "preprocessor.hpp"
#include "individual.hpp"
#include <array>
#include <cstdint>
#include <filesystem>
#include <random>
#include <string>
#include <vector>
#include <unordered_map>


#define INFEASIBLE 1'000'000'000L

struct Station {
    int pos;
    int station_id;
};

// This struct is used to store the charging station information for the given route
struct ChargingMeta {
    double cost{};          // cost after applying recharging decision
    int num_stations{};     // num of stations
    std::vector<int> chosen_pos; // chosen position
    std::vector<int> chosen_sta; // chosen station
};

class Follower {
    /* All these auxiliary data structures are used to reduce the memory allocation repeatedly */
    mutable int* temp_route = nullptr;
    mutable int* temp_chosen_pos = nullptr;
    mutable int* temp_best_chosen_pos = nullptr;
    mutable double* temp_accumulated_distance = nullptr;
    mutable int buffer_size = 0;

    mutable std::vector<Station> temp_station_inserted;

    void prepare_temp_buffers(int required_size) const;
    void release_temp_buffers() const;
public:

    Case* instance;
    Preprocessor* preprocessor;

    /* Auxiliary data structures to run the follower (i.e., lower optimisation) algorithm */
    int route_cap;                         // the maximum number of routes
    int node_cap;                          // the maximum number of nodes in a route
    int lower_node_cap;                    // capacity of each lower_routes row (charged route: nodes + stations)
    int num_routes;                        // Number of routes
    int** lower_routes;                    // the routing + charging solution
    int*  lower_num_nodes_per_route;       // the node number of each route in the lower-level solution
    double* lower_cost_per_route;          // the cost of each route in the lower-level solution
    double lower_cost;                     // the lower cost of the solution, i.e., the total distance of all complete routes

    // Simple enumeration (SE) station counts. false (default): try l_min, then l_min + 1 only if no placement
    // with l_min stations is feasible (original B-LAHC behaviour, used by LAHC and TwoStageAlns).
    // true: evaluate both l_min and l_min + 1 and keep the cheaper placement (used by MA).
    bool se_compare_station_counts = false;

    // Charging during search (Follower::run): false = simple enumeration with insert-then-remove fallback,
    // true = exact DP (charging_placement_dp), every distance read charged as for SE.
    bool use_optimal_charging = false;

    // Verification (-lower_solver verify): SE as usual; if the candidate's optimistic heuristic cost (SE cost, or the
    // uncharged route length where SE and its fallback fail) is at most verify_threshold, every route is re-evaluated
    // with the exact DP (reads charged) and the exact cost replaces the heuristic one. The host sets the threshold.
    bool verify_candidates = false;
    double verify_threshold = std::numeric_limits<double>::max();
    std::uint64_t verify_calls = 0, verify_improved = 0, verify_rescued = 0, verify_evaluations = 0;
    void write_verify_log(const std::filesystem::path& directory, const std::string& instance_name) const;

    // Shadow diagnostics (SE mode only): every route evaluated by Follower::run is also solved exactly with
    // the DP; the reads of the shadow solve are not charged, so the search is unchanged. Statistics are
    // kept per tenth of the evaluation budget, and a reservoir sample of routes is kept for offline tests.
    bool shadow_optimal = false;
    struct ShadowStage {
        std::uint64_t calls = 0;               // Follower::run calls
        std::uint64_t charging_routes = 0;     // routes longer than the driving range
        std::uint64_t se_optimal = 0;          // heuristic cost equals the exact cost
        std::uint64_t se_failed = 0;           // simple enumeration found no placement (fallback used)
        std::uint64_t false_infeasible = 0;    // heuristic infeasible, exact feasible
        std::uint64_t both_infeasible = 0;     // no feasible placement exists
        double gap_sum = 0.0;                  // sum of relative gaps (%) over routes solved by the heuristic
        double gap_max = 0.0;
        std::array<std::uint64_t, 3> decisions{};    // decision points per kind (see record_decision)
        std::array<std::uint64_t, 3> disagreements{};
        std::array<std::uint64_t, 3> se_only{};      // update under heuristic costs only
        std::array<std::uint64_t, 3> dp_only{};      // update under exact costs only
    };
    struct ShadowRoute {
        int stage;
        double heuristic_cost;  // -1 if infeasible for the heuristic
        double exact_cost;      // -1 if infeasible
        std::vector<int> route;
    };
    std::array<ShadowStage, 10> shadow_stages{};
    std::vector<ShadowRoute> shadow_routes;  // reservoir sample
    std::size_t shadow_route_capacity = 2000;
    std::uint64_t shadow_routes_seen = 0;
    std::mt19937 shadow_rng{12345};         // private engine: never touches the algorithm's random streams
    double shadow_cost_per_route_total = 0.0;  // running sum within one Follower::run call
    // Exact costs of routes already solved in shadow mode (key: hash of the node sequence). Unchanged routes are
    // not solved again, and route-level statistics count each distinct route once.
    std::unordered_map<std::uint64_t, double> shadow_cache;
    std::size_t shadow_cache_limit = 4'000'000;
    double shadow_best_seen = std::numeric_limits<double>::max();  // lowest exact cost of any evaluated solution
    int shadow_stage() const;
    // kind 0: LAHC global-best update; 1: MA choice of the best evaluated individual (update = "same choice");
    // 2: MA update of the best complete solution so far.
    void record_decision(int kind, bool heuristic_update, bool exact_update);
    void write_shadow_log(const std::filesystem::path& directory, const std::string& instance_name,
                          double heuristic_final_cost, double exact_cost_of_heuristic_final) const;

    Follower(Case* instance, Preprocessor* preprocessor);
    ~Follower();

    void run_for_single_route(int route_idx) const;  // run the follower decision for the given single route
    void load_individual(const Individual* ind);     // load the individual to the follower
    void export_individual(Individual* ind) const;   // export the individual from the follower, i.e., update lower_cost
    void run(Individual* ind);                       // run the follower decision for the given complete solution
    void run(const PartialSolution* partial_sol);    // run the follower decision for the given partial solution
    void refine(Individual* ind);                    // refine the final solution with the optimal charging visits per route
    void run_with_variables(Individual* ind);        // run the follower decision with intermediate variables (e.g., lower_routes)

    // Evaluate-only (do NOT write stations into the given route)
    double eval_simple_enum_cost(const int* route, int length) const;
    double eval_remove_enum_cost(const int* route, int length) const;

    double insert_station_by_simple_enum(int* repaired_route, int& repaired_length) const;
    int best_feasible_station_charged(int from, int to, double max_dis) const;  // RE station choice, reads charged
    double insert_station_by_remove_enum(int* repaired_route, int& repaired_length) const;
    void recursive_charging_placement(int m_len, int n_len, int* chosen_pos, int* best_chosen_pos, double& final_cost,
        int cur_upper_bound, const int* route, int length, const double* accumulated_distance) const;
    // Optimal charging placement for the final refinement (shortest path over any number of stations).
    double insert_station_by_all_enumeration(int* repaired_route, int& repaired_length) const;
    // Exact charging cost of a route (evaluate only), -1 if infeasible; distance reads are charged.
    double eval_optimal_cost(const int* route, int length) const;

    friend std::ostream& operator<<(std::ostream& os, const Follower& follower);
};

#endif //FOLLOWER_HPP
