//
// Created by Yinghao Qin on 10/06/2025.
//

#ifndef LEADER_CBMA_HPP
#define LEADER_CBMA_HPP

#include "case.hpp"
#include "preprocessor.hpp"
#include "individual.hpp"
#include <functional>  // For std::function
#include <unordered_set>

using AcceptanceFunc = std::function<bool(const double& change, const double& temperature)>;


class LeaderCbma {
private:
    [[nodiscard]] static bool greedy_accept(const double& change, const double& temperature);
    [[nodiscard]] static bool neighbor_accept(const double& change, const double& temperature);

    using IntraFunc = std::function<bool(int*, int)>;
    using InterFunc = std::function<bool(int*, int*, int&, int&, int&, int&)>;
    const std::vector<IntraFunc> intra_perturb_moves;
    const std::vector<InterFunc> inter_perturb_moves;
    const std::vector<InterFunc> inter_perturb_with_empty_moves;

    using IntraImproFunc = std::function<bool(int*, int, double)>;
    using InterImproFunc = std::function<bool(int*, int*, int&, int&, int&, int&, double)>;
    const std::vector<IntraImproFunc> intra_greedy_moves;
    const std::vector<InterImproFunc> inter_greedy_moves;

    const std::vector<IntraImproFunc> intra_neigh_moves;
    const std::vector<InterImproFunc> inter_neigh_moves;


    unordered_set<pair<int, int>, PairHash> route_pairs;

    int* temp_r1 = nullptr;
    int* temp_r2 = nullptr;
    int temp_buffer_size = 0;

    void prepare_temp_buffers(int required_size);
    void prepare_moves();
public:

    Case* instance;
    Preprocessor* preprocessor;

    PartialSolution* partial_sol;

    int route_cap;
    int node_cap;
    int** routes;
    int num_routes;
    int* num_nodes_per_route;
    int* demand_sum_per_route;
    double upper_cost;

    uniform_int_distribution<> perturb_move_dist;
    int greedy_moves_count;
    vector<int> greedy_move_indices;
    uniform_int_distribution<> neigh_search_dist;

    LeaderCbma(Case* instance, Preprocessor* preprocessor);
    ~LeaderCbma();

    static void moveItoJ(int* route, int a, int b);
    void clean();
    void clean_empty_routes(int r1, int r2); // clean possible empty routes after move
    [[nodiscard]] std::pair<int, int> randomly_select_diff_route_pair() const noexcept;

    bool perturbation(int strength);
    void fully_greedy_local_optimum(Individual* ind);
    bool neighbourhood_search(PartialSolution* partial_solution, const double& temperature);
    void load_individual(const Individual* ind);
    void export_individual(Individual* ind) const;

    /* Strong perturbation */
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

    /* Greedy search & neighbourhood search */
    [[nodiscard]] bool perform_intra_move_neigh(const IntraImproFunc& move_func, double temperature) const;
    [[nodiscard]] bool perform_inter_move_neigh(const InterImproFunc& move_func, double temperature);
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


    friend ostream& operator<<(ostream& os, const LeaderCbma& leader);
};

#endif //LEADER_CBMA_HPP
