//
// Created by Yinghao Qin on 09/06/2025.
//
// Make an official naming for the upper-level neighbourhood exploration,
// i.e., "Single-Operator Neighbourhood Exploration",
// which selects one operator at a time to the incumbent solution (maybe attempts up to multiple times).


#ifndef LEADER_SINGLE_HPP
#define LEADER_SINGLE_HPP

#include "leader.hpp"

using Param = std::uniform_int_distribution<int>::param_type;

class LeaderSingle final : public Leader {
public:
    explicit LeaderSingle(std::mt19937& engine, Case* instance, Preprocessor* preprocessor);
    bool neighbourhood_explore(Individual* ind, const double& history_val) override;
    void write_operator_profile(std::ostream& os) const;

    int max_attempts = 0;

protected:
    /* ============================ */
    /* Config / constants / types   */
    static constexpr int kNumIntraNeigh = 3;
    static constexpr int kNumInterNeigh = 4;
    static constexpr int kNumInterEmptyNeigh = 1;
    static constexpr int kNumOps = kNumIntraNeigh + kNumInterNeigh + kNumInterEmptyNeigh;
    static inline constexpr std::array<const char*, kNumOps> kOpLabels = {
        "M1_relocate_intra",
        "M2_relocate_inter",
        "M3_swap_intra",
        "M4_swap_inter",
        "M5_2opt_intra",
        "M6_2opt_inter_h2h",
        "M7_2opt_inter_h2t",
        "M8_relocate_inter_empty"
    };

    using OpFn       = bool (LeaderSingle::*)();
    using IntraMemFn = bool (LeaderSingle::*)(int* route, int length, AcceptRule);
    using InterMemFn = bool (LeaderSingle::*)(int* r1, int* r2, int& len1, int& len2, int& load1, int& load2, AcceptRule);

    /* ============================ */
    /* Operator selection (table)   */
    std::array<OpFn, kNumOps> ops{};
    std::uniform_int_distribution<int> op_selector; // [0, kNumOps-1]

    bool op_relocate_intra();
    bool op_relocate_inter();
    bool op_swap_intra();
    bool op_swap_inter();
    bool op_2opt_intra();
    bool op_2opt_inter_h2h();
    bool op_2opt_inter_h2t();
    bool op_relocate_inter_empty();

    /* ============================ */
    /* Neighbourhood operator wrappers       */

    bool perform_intra_move(IntraMemFn fn);
    bool perform_inter_move(InterMemFn fn);
    bool perform_inter_move_with_empty_route(InterMemFn fn);

    /* ============================ */
    /* RNG helpers (hot path)       */
    std::uniform_int_distribution<int> dist_route;
    std::uniform_int_distribution<int> dist_i;
    std::uniform_int_distribution<int> dist_j;

    bool enable_op_profile = false;
    std::array<double, kNumOps> op_time_sec{};
    std::array<long, kNumOps> op_call_counts{};
    std::array<long, kNumOps> op_accept_counts{};

    /* ============================ */
    /* Operators (primitive moves)  */
    void clean_empty_routes(int r1, int r2);

    bool node_relocate_intra(int* route, int length, AcceptRule rule);
    bool node_swap_intra(int* route, int length, AcceptRule rule);
    bool two_opt_intra(int* route, int length, AcceptRule rule);

    bool node_relocate_inter(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, AcceptRule rule);
    bool node_swap_inter(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, AcceptRule rule);
    bool two_opt_inter_head_to_head(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, AcceptRule rule);
    bool two_opt_inter_head_to_tail(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, AcceptRule rule);

    bool node_relocate_inter_with_empty_route(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, AcceptRule rule);
};

#endif // LEADER_SINGLE_HPP
