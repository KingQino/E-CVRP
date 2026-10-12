//
// Created by Yinghao Qin on 11/10/2026.
//
// Upper-level local search of the MA: eight neighbourhood operators RVND that accepts at most one
// improving move per selected neighbourhood (LS-8-RVND-OneMove, including SWAP*).
//

#ifndef MA_LEADER_HPP
#define MA_LEADER_HPP

#include "ma_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <utility>
#include <vector>

class Case;
class Individual;

enum class LocalSearchOperator {
    NodeShift,
    InterRouteRelocate,
    IntraRouteSwap,
    InterRouteSwap,
    SwapStar,
    TwoOpt,
    TwoOptStarHeadToHead,
    TwoOptStarHeadToTail,
    Count,
};

constexpr std::size_t LOCAL_SEARCH_OPERATOR_COUNT = static_cast<std::size_t>(LocalSearchOperator::Count);

struct LocalSearchOperatorStats {
    int calls{};
    int accepts{};
    std::uint64_t distance_calls{};
    double upper_gain{};
    int gamma_crosses{};
};

struct LocalSearchResult {
    int move_limit{};
    int accepted_moves{};
    int neighborhood_calls{};
    std::uint64_t distance_calls_used{};
    double relative_upper_improvement{};
    bool reached_local_optimum{};
    bool hit_move_limit{};
    bool hit_distance_call_limit{};
    std::array<LocalSearchOperatorStats, LOCAL_SEARCH_OPERATOR_COUNT> operator_stats{};
};

// State of one RVND session; allows a weak probe to be continued to a deeper action.
struct LocalSearchSession {
    bool initialized{};
    int total_accepted_moves{};
    int total_neighborhood_calls{};
    std::uint64_t total_distance_calls{};
    std::vector<LocalSearchOperator> active_operators;
};

// Reusable buffers and failure caches of the RVND session.
struct LocalSearchWorkspace {
    struct InsertionCandidate {
        double cost{};
        int insertion_index{-1};
    };

    using TopThreeInsertions = std::array<InsertionCandidate, 3>;

    struct ActiveRoutePairPool {
        std::vector<std::pair<int, int>> pairs;
        std::vector<unsigned char> membership;
        std::vector<std::uint64_t> observed_route_versions;
        std::uint64_t topology_version{};
        int route_count{};
        bool directed{};
    };

    std::vector<int> route_order;
    std::vector<int> first_route_buffer;
    std::vector<int> second_route_buffer;
    std::vector<TopThreeInsertions> first_customers_into_second;
    std::vector<TopThreeInsertions> second_customers_into_first;
    std::vector<double> first_removal_costs;
    std::vector<double> second_removal_costs;
    // Failed route scans use versions; failed pair scans leave the active pool.
    std::vector<std::uint64_t> route_versions;
    std::array<std::vector<std::uint64_t>, LOCAL_SEARCH_OPERATOR_COUNT> failed_route_versions;
    std::array<ActiveRoutePairPool, LOCAL_SEARCH_OPERATOR_COUNT> active_route_pair_pools;
    std::size_t route_pair_cache_stride{};
    std::uint64_t next_route_version{1};
    std::uint64_t route_topology_version{1};
    std::array<int, 2> changed_routes{-1, -1};
    int changed_route_count{};
    bool all_routes_changed{};
};

class MaLeader {
public:
    static const char* operator_name(LocalSearchOperator local_search_operator);

    // Run one complete RVND session for the given action (Strong = weak probe, then continuation
    // up to the strong move and distance-call caps). `gamma_upper_bound` is only used to count
    // moves that bring the upper cost inside the follower trigger bound (statistics).
    static LocalSearchResult improve_with_eight_neighborhood_rvnd_one_move(
        Individual& individual,
        Case& instance,
        std::mt19937& random_engine,
        LocalSearchAction action,
        LocalSearchWorkspace& workspace,
        double gamma_upper_bound = std::numeric_limits<double>::infinity(),
        const LocalSearchDepthConfig& depth_config = {});

    // Unrestricted RVND: run until all eight neighbourhoods fail (local optimum). Not an action;
    // used by Elite Unrestricted and the static baseline.
    static LocalSearchResult improve_unrestricted(
        Individual& individual,
        Case& instance,
        std::mt19937& random_engine,
        LocalSearchWorkspace& workspace,
        double gamma_upper_bound = std::numeric_limits<double>::infinity());

    static void begin_eight_neighborhood_rvnd_one_move_session(
        Individual& individual,
        LocalSearchSession& session,
        LocalSearchWorkspace& workspace);

    static LocalSearchResult continue_eight_neighborhood_rvnd_one_move_session(
        Individual& individual,
        Case& instance,
        std::mt19937& random_engine,
        LocalSearchSession& session,
        int cumulative_move_limit,
        LocalSearchWorkspace& workspace,
        double gamma_upper_bound,
        std::uint64_t cumulative_distance_call_limit = std::numeric_limits<std::uint64_t>::max());

    static int move_limit_for_action(
        const Individual& individual,
        const Case& instance,
        LocalSearchAction action,
        const LocalSearchDepthConfig& depth_config = {});

    static std::uint64_t strong_distance_call_limit(
        std::uint64_t weak_distance_calls,
        const LocalSearchDepthConfig& depth_config = {});
};

#endif //MA_LEADER_HPP
