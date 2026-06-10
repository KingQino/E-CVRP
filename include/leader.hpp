//
// Created by Yinghao Qin on 22/01/2026.
//

#ifndef LEADER_HPP
#define LEADER_HPP

#include "case.hpp"
#include "preprocessor.hpp"
#include "individual.hpp"

#include <array>
#include <cstdint>
#include <random>
#include <unordered_set>
#include <utility>
#include <ostream>

class Leader {
public:
    Leader(std::mt19937& engine, Case* instance, Preprocessor* preprocessor);
    virtual ~Leader() noexcept;

    // Core API
    virtual void load_individual(const Individual* ind);
    virtual void export_individual(Individual* ind) const;
    virtual void on_after_load(const Individual* /*ind*/) {}

    // The DIFFERENCE point between derived leaders
    virtual bool neighbourhood_explore(Individual* ind, const double& history_val) = 0;

    // Common greedy phase shared by all leaders
    void greedy_descent(Individual* ind);

    void sweep_descent(Individual* ind);

    void light_descent(Individual* ind);

    // Shared state (points to Individual buffers after load_individual)
    Case* instance = nullptr;
    Preprocessor* preprocessor = nullptr;
    std::mt19937& random_engine;

    int route_cap = 0;
    int node_cap  = 0;

    int** routes = nullptr;
    int num_routes = 0;
    int* num_nodes_per_route = nullptr;
    int* demand_sum_per_route = nullptr;

    double upper_cost = 0.0;
    double history_cost = 0.0;

protected:
    /* ============================ */
    /* Acceptance rules             */
    enum class AcceptRule : uint8_t { Greedy, Late };

    [[nodiscard]] inline bool accept(AcceptRule rule, double change) const noexcept {
        return (rule == AcceptRule::Greedy) ? greedy_accept(change) : late_accept(change);
    }

    [[nodiscard]] static inline bool greedy_accept(double change) noexcept {
        return change < -EPSILON;
    }

    [[nodiscard]] inline bool late_accept(double change) const noexcept {
        return (upper_cost + change < history_cost) || (change < -EPSILON);
    }

    /* ============================ */
    /* Greedy-improvement move table */
    using IntraGreedyMemFn = bool (Leader::*)(int*, int, AcceptRule);
    using InterGreedyMemFn = bool (Leader::*)(int*, int*, int&, int&, int&, int&, AcceptRule);

    static constexpr int kNumIntraGreedy = 7;
    static constexpr int kNumInterGreedy = 8;
    static constexpr int kGreedyChoices  = kNumIntraGreedy + kNumInterGreedy;

    std::array<IntraGreedyMemFn, kNumIntraGreedy> intra_greedy_moves{};
    std::array<InterGreedyMemFn, kNumInterGreedy> inter_greedy_moves{};
    std::array<int, kGreedyChoices> greedy_move_indices{};

    void prepare_greedy_moves();

    /* ============================ */
    /* Inter-route bookkeeping      */
    std::unordered_set<std::pair<int, int>, PairHash> route_pairs;

    /* ============================ */
    /* Temporary buffers            */
    int* temp_r1 = nullptr;
    int* temp_r2 = nullptr;
    int temp_buffer_size = 0;
    void prepare_temp_buffers(int required_size);

    /* ============================ */
    /* Greedy move executors        */
    bool perform_intra_move_greedy(IntraGreedyMemFn fn);
    bool perform_inter_move_greedy(InterGreedyMemFn fn);

    /* ============================ */
    /* Primitive operators (O(n^2)) */
    static void relocate_pos(int* route, int from, int to);

    bool move1_intra_impro(int* route, int length, AcceptRule rule);
    bool move2_intra_impro(int* route, int length, AcceptRule rule);
    bool move3_intra_impro(int* route, int length, AcceptRule rule);
    bool move4_intra_impro(int* route, int length, AcceptRule rule);
    bool move5_intra_impro(int* route, int length, AcceptRule rule);
    bool move6_intra_impro(int* route, int length, AcceptRule rule);
    bool move7_intra_impro(int* route, int length, AcceptRule rule);

    bool move1_inter_impro(int* r1, int* r2, int& len1, int& len2, int& load1, int& load2, AcceptRule rule);
    bool move2_inter_impro(int* r1, int* r2, int& len1, int& len2, int& load1, int& load2, AcceptRule rule);
    bool move3_inter_impro(int* r1, int* r2, int& len1, int& len2, int& load1, int& load2, AcceptRule rule);
    bool move4_inter_impro(int* r1, int* r2, int& len1, int& len2, int& load1, int& load2, AcceptRule rule);
    bool move5_inter_impro(int* r1, int* r2, int& len1, int& len2, int& load1, int& load2, AcceptRule rule);
    bool move6_inter_impro(int* r1, int* r2, int& len1, int& len2, int& load1, int& load2, AcceptRule rule);
    bool move8_inter_impro(int* r1, int* r2, int& len1, int& len2, int& load1, int& load2, AcceptRule rule);
    bool move9_inter_impro(int* r1, int* r2, int& len1, int& len2, int& load1, int& load2, AcceptRule rule);

    /* ============================ */
    /* Sweep move executors         */
    bool perform_intra_move_sweep(IntraGreedyMemFn fn);
    bool perform_inter_move_sweep(InterGreedyMemFn fn);

    std::vector<std::pair<int, int>> vec_route_pairs;

    bool move7_intra_impro_sweep(int* route, int length, AcceptRule rule);
    bool move8_inter_impro_sweep(int* r1, int* r2, int& len1, int& len2, int& load1, int& load2, AcceptRule rule);

    /* ============================ */
    /* Light move executors         */
    static constexpr double kLightInterProb = 0.6;

    std::bernoulli_distribution light_inter_choice_dist{kLightInterProb};
    std::uniform_int_distribution<int> light_intra_move_dist{0, kNumIntraGreedy - 1};
    std::uniform_int_distribution<int> light_inter_move_dist{0, kNumInterGreedy - 1};

    std::vector<int> vec_route_order;

    bool perform_intra_move_once(const IntraGreedyMemFn fn);
    bool perform_inter_move_once(const InterGreedyMemFn fn);

    friend std::ostream& operator<<(std::ostream& os, const Leader& leader);
};

#endif // LEADER_HPP
