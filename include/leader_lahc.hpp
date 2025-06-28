//
// Created by Yinghao Qin on 09/06/2025.
//

#ifndef LEADER_LAHC_HPP
#define LEADER_LAHC_HPP

#include "case.hpp"
#include "preprocessor.hpp"
#include "individual.hpp"
#include <functional>  // For std::function


class LeaderLahc {
    int* temp_r1 = nullptr;
    int* temp_r2 = nullptr;
    int temp_buffer_size = 0;

    void prepare_temp_buffers(int required_size);
public:
    Case* instance;
    Preprocessor* preprocessor;
    std::mt19937& random_engine;
    uniform_int_distribution<> move_type_selector;

    int route_cap;
    int node_cap;
    int** routes;
    int num_routes;
    int* num_nodes_per_route;
    int* demand_sum_per_route;
    int max_attempts;
    double upper_cost;
    double history_cost;

    LeaderLahc(std::mt19937& engine, Case* instance, Preprocessor* preprocessor);
    ~LeaderLahc();

    void clean();
    bool random_walk(const double& history_val);    // Random walk exploration with late acceptance criteria
    void load_individual(const Individual* ind);
    void export_individual(Individual* ind) const;

    static void moveItoJ(int* route, int a, int b);
    [[nodiscard]] bool late_accept(const double& change) const;
    bool two_opt_for_single_route(int* route, int length);
    bool two_opt_intra_for_individual();
    bool node_relocation_for_single_route(int* route, int length);
    bool node_relocation_intra_for_individual(); // three-arcs exchange, intra-route
    bool node_relocation_between_two_routes(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool node_relocation_inter_for_individual(); // three-arcs exchange, inter-route
    bool node_exchange_for_single_route(int* route, int length);
    bool node_exchange_intra_for_individual();   // four-arcs exchange, intra-route
    bool node_exchange_between_two_routes(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2);
    bool node_exchange_inter_for_individual();   // four-arcs exchange, inter-route

    void clean_empty_routes(int r1, int r2); // clean possible empty routes after move
    bool perform_intra_move(const std::function<bool(int*, int)>& move_func) const;
    bool perform_inter_move(const std::function<bool(int*, int*, int&, int&, int&, int&)>& move_func);
    bool perform_inter_move_with_empty_route(const std::function<bool(int*, int*, int&, int&, int&, int&)>& move_func);
    bool move8_inter(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2); // 2-opt inter-route move 1
    bool move9_inter(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2); // 2-opt inter-route move 2
    bool move1_inter_with_empty_route(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    // move8: head-to-head two-way 2-opt
    // move9: tail-to-tail two-way 2-opt
    // move1: insert one customer from route1 to new empty route


    friend ostream& operator<<(ostream& os, const LeaderLahc& leader);
};

#endif //LEADER_LAHC_HPP
