//
// Created by Yinghao Qin on 09/06/2025.
//

#ifndef FOLLOWER_HPP
#define FOLLOWER_HPP

#include "case.hpp"
#include "preprocessor.hpp"
#include "individual.hpp"


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
    int num_routes;                        // Number of routes
    int** lower_routes;                    // the routing + charging solution
    int*  lower_num_nodes_per_route;       // the node number of each route in the lower-level solution
    double* lower_cost_per_route;          // the cost of each route in the lower-level solution
    double lower_cost;                     // the lower cost of the solution, i.e., the total distance of all complete routes

    Follower(Case* instance, Preprocessor* preprocessor);
    ~Follower();

    void run_for_single_route(int route_idx) const;  // run the follower decision for the given single route
    void load_individual(const Individual* ind);     // load the individual to the follower
    void export_individual(Individual* ind) const;   // export the individual from the follower, i.e., update lower_cost
    void run(Individual* ind);                       // run the follower decision for the given complete solution
    void run(const PartialSolution* partial_sol);    // run the follower decision for the given partial solution
    void refine(Individual* ind);                    // refine the final solution by enumerating all possible charging visits
    void run_with_variables(Individual* ind);        // run the follower decision with intermediate variables (e.g., lower_routes)

    // Evaluate-only (do NOT write stations into the given route)
    double eval_simple_enum_cost(const int* route, int length) const;
    double eval_remove_enum_cost(const int* route, int length) const;

    double insert_station_by_simple_enum(int* repaired_route, int& repaired_length) const;
    double insert_station_by_remove_enum(int* repaired_route, int& repaired_length) const;
    void recursive_charging_placement(int m_len, int n_len, int* chosen_pos, int* best_chosen_pos, double& final_cost,
        int cur_upper_bound, const int* route, int length, const double* accumulated_distance) const;
    double insert_station_by_all_enumeration(int* repaired_route, int& repaired_length) const;
    ChargingMeta try_enumerate_n_stations_to_route(int m_len, int n_len, int* chosen_sta, int* chosen_pos, double& cost,
        int cur_upper_bound, const int* route, int length, const std::vector<double>& accumulated_distance) const;

    friend std::ostream& operator<<(std::ostream& os, const Follower& follower);
};

#endif //FOLLOWER_HPP
