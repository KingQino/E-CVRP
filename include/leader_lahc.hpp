//
// Created by Yinghao Qin on 09/06/2025.
//

#ifndef LEADER_LAHC_HPP
#define LEADER_LAHC_HPP

#include "case.hpp"
#include "preprocessor.hpp"
#include "individual.hpp"
#include <functional>  // For std::function
#include <unordered_set>


using AcceptanceFunc = std::function<bool(const double& change, const double& temperature)>;


class LeaderLahc {
    [[nodiscard]] static bool greedy_accept(const double& change, const double& temperature);

    using IntraImproFunc = std::function<bool(int*, int, double)>;
    using InterImproFunc = std::function<bool(int*, int*, int&, int&, int&, int&, double)>;
    std::vector<IntraImproFunc> intra_greedy_moves;
    std::vector<InterImproFunc> inter_greedy_moves;

    using IntraFunc = std::function<bool(int*, int)>;
    using InterFunc = std::function<bool(int*, int*, int&, int&, int&, int&)>;
    std::vector<IntraFunc> intra_perturb_moves;
    std::vector<InterFunc> inter_perturb_moves;
    std::vector<InterFunc> inter_perturb_with_empty_moves;

    unordered_set<pair<int, int>, PairHash> route_pairs;

    int* temp_r1 = nullptr;
    int* temp_r2 = nullptr;
    int temp_buffer_size = 0;

    void prepare_temp_buffers(int required_size);
    void prepare_moves();
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

    /* =========================================================== */
    /* Greedy search until local optimum */
    int greedy_moves_count;
    vector<int> greedy_move_indices;
    void fully_greedy_local_optimum(Individual* ind);

    [[nodiscard]] bool perform_intra_move_greedy(const IntraImproFunc& move_func) const;
    [[nodiscard]] bool perform_inter_move_greedy(const InterImproFunc& move_func);
    // basic operators - neighbourhood size O(n^2)
    bool move1_intra_impro(int* route, int length, const AcceptanceFunc& accept_func, double temperature); // if U is ahead of V, then move U to the behind of V; otherwise, move U to the ahead of V
    bool move1_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, const AcceptanceFunc& accept_func, double temperature);
    bool move2_intra_impro(int* route, int length, const AcceptanceFunc& accept_func, double temperature);
    bool move2_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, const AcceptanceFunc& accept_func, double temperature);
    bool move3_intra_impro(int* route, int length, const AcceptanceFunc& accept_func, double temperature);
    bool move3_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, const AcceptanceFunc& accept_func, double temperature);
    bool move4_intra_impro(int* route, int length, const AcceptanceFunc& accept_func, double temperature);
    bool move4_inter_impro(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2, const AcceptanceFunc& accept_func, double temperature);
    bool move5_intra_impro(int* route, int length, const AcceptanceFunc& accept_func, double temperature);
    bool move5_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, const AcceptanceFunc& accept_func, double temperature);
    bool move6_intra_impro(int* route, int length, const AcceptanceFunc& accept_func, double temperature);
    bool move6_inter_impro(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2, const AcceptanceFunc& accept_func, double temperature);
    bool move7_intra_impro(int* route, int length, const AcceptanceFunc& accept_func, double temperature);
    bool move8_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, const AcceptanceFunc& accept_func, double temperature);
    bool move9_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, const AcceptanceFunc& accept_func, double temperature);

    /* =========================================================== */
    /* Perturbation */
    uniform_int_distribution<> perturb_move_dist;
    bool perturbation(int strength);
    [[nodiscard]] std::pair<int, int> randomly_select_diff_route_pair() const noexcept;
    [[nodiscard]] bool perform_intra_move_pert(const IntraFunc& move_func) const;
    [[nodiscard]] bool perform_inter_move_pert(const InterFunc& move_func);
    [[nodiscard]] bool perform_inter_move_with_empty_pert(const InterFunc& move_func);
    // basic perturbation moves
    bool move1_intra_pert(int* route, int length);
    bool move1_inter_pert(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move1_inter_with_empty_route_pert(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move2_intra_pert(int* route, int length);
    bool move2_inter_pert(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move3_intra_pert(int* route, int length);
    bool move3_inter_pert(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move4_intra_pert(int* route, int length);
    bool move4_inter_pert(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2);
    bool move5_intra_pert(int* route, int length);
    bool move5_inter_pert(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move6_intra_pert(int* route, int length);
    bool move6_inter_pert(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2);
    bool move7_intra_pert(int* route, int length);
    bool move8_inter_pert(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move8_inter_with_empty_route_pert(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move9_inter_pert(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move9_inter_with_empty_route_pert(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);


    friend ostream& operator<<(ostream& os, const LeaderLahc& leader);
};

#endif //LEADER_LAHC_HPP
