//
// Created by Yinghao Qin on 11/10/2026.
//

#include "ma_leader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include "case.hpp"
#include "individual.hpp"

namespace {

// Changing the upper-level routes invalidates the follower result and the local-optimum flag.
inline void set_upper_cost(Individual& individual, const double cost) {
    individual.upper_cost = cost;
    individual.upper_locally_optimal = false;
    individual.lower_cost = std::numeric_limits<double>::max();
}

enum class Neighborhood {
    TwoOpt,
    TwoOptStar,
    NodeShift,
    InterRouteRelocate,
    InterRouteSwap,
    SwapStar,
    IntraRouteSwap,
    TwoOptStarHeadToHead,
    TwoOptStarHeadToTail,
};

constexpr double kImprovementTolerance = 0.00000001;

constexpr std::array<LocalSearchOperator, 8> kEightOperators = {
    LocalSearchOperator::NodeShift,
    LocalSearchOperator::InterRouteRelocate,
    LocalSearchOperator::IntraRouteSwap,
    LocalSearchOperator::InterRouteSwap,
    LocalSearchOperator::SwapStar,
    LocalSearchOperator::TwoOpt,
    LocalSearchOperator::TwoOptStarHeadToHead,
    LocalSearchOperator::TwoOptStarHeadToTail,
};

Neighborhood neighborhood_for_operator(
    LocalSearchOperator local_search_operator) {
    switch (local_search_operator) {
        case LocalSearchOperator::NodeShift:
            return Neighborhood::NodeShift;
        case LocalSearchOperator::InterRouteRelocate:
            return Neighborhood::InterRouteRelocate;
        case LocalSearchOperator::IntraRouteSwap:
            return Neighborhood::IntraRouteSwap;
        case LocalSearchOperator::InterRouteSwap:
            return Neighborhood::InterRouteSwap;
        case LocalSearchOperator::SwapStar:
            return Neighborhood::SwapStar;
        case LocalSearchOperator::TwoOpt:
            return Neighborhood::TwoOpt;
        case LocalSearchOperator::TwoOptStarHeadToHead:
            return Neighborhood::TwoOptStarHeadToHead;
        case LocalSearchOperator::TwoOptStarHeadToTail:
            return Neighborhood::TwoOptStarHeadToTail;
        case LocalSearchOperator::Count:
            break;
    }
    throw std::logic_error("unknown local-search operator");
}

std::size_t operator_index(LocalSearchOperator local_search_operator) {
    return static_cast<std::size_t>(local_search_operator);
}

void clear_failure_stamps(LocalSearchWorkspace& workspace) {
    for (auto& failed_versions : workspace.failed_route_versions) {
        std::fill(failed_versions.begin(), failed_versions.end(), 0);
    }
    for (auto& route_pair_pool : workspace.active_route_pair_pools) {
        route_pair_pool.topology_version = 0;
    }
    workspace.next_route_version = 1;
}

void advance_route_topology_version(LocalSearchWorkspace& workspace) {
    if (workspace.route_topology_version
        == std::numeric_limits<std::uint64_t>::max()) {
        for (auto& route_pair_pool : workspace.active_route_pair_pools) {
            route_pair_pool.topology_version = 0;
        }
        workspace.route_topology_version = 1;
        return;
    }
    ++workspace.route_topology_version;
}

std::uint64_t next_route_version(LocalSearchWorkspace& workspace) {
    if (workspace.next_route_version
        == std::numeric_limits<std::uint64_t>::max()) {
        clear_failure_stamps(workspace);
        advance_route_topology_version(workspace);
    }
    return workspace.next_route_version++;
}

void begin_failure_cache_session(
    int route_count,
    LocalSearchWorkspace& workspace) {
    const std::size_t required_stride =
        static_cast<std::size_t>(route_count);
    if (workspace.route_pair_cache_stride < required_stride) {
        workspace.route_pair_cache_stride = required_stride;
    }

    advance_route_topology_version(workspace);
    workspace.route_versions.resize(workspace.route_pair_cache_stride);
    for (auto& failed_versions : workspace.failed_route_versions) {
        failed_versions.resize(workspace.route_pair_cache_stride, 0);
    }
    for (int route_index = 0; route_index < route_count; ++route_index) {
        workspace.route_versions[static_cast<std::size_t>(route_index)] =
            next_route_version(workspace);
    }
    workspace.changed_route_count = 0;
    workspace.all_routes_changed = false;
}

bool route_failure_is_cached(
    LocalSearchOperator local_search_operator,
    int route_index,
    const LocalSearchWorkspace& workspace) {
    const auto& failed_versions =
        workspace.failed_route_versions[operator_index(local_search_operator)];
    return failed_versions[static_cast<std::size_t>(route_index)]
        == workspace.route_versions[static_cast<std::size_t>(route_index)];
}

void cache_route_failure(
    LocalSearchOperator local_search_operator,
    int route_index,
    LocalSearchWorkspace& workspace) {
    workspace.failed_route_versions[operator_index(local_search_operator)]
        [static_cast<std::size_t>(route_index)] =
            workspace.route_versions[static_cast<std::size_t>(route_index)];
}

std::size_t route_pair_cache_index(
    int first_route,
    int second_route,
    const LocalSearchWorkspace& workspace) {
    return static_cast<std::size_t>(first_route)
        * workspace.route_pair_cache_stride
        + static_cast<std::size_t>(second_route);
}

void add_active_route_pair(
    LocalSearchWorkspace::ActiveRoutePairPool& route_pair_pool,
    int first_route,
    int second_route,
    const LocalSearchWorkspace& workspace) {
    const std::size_t membership_index =
        route_pair_cache_index(first_route, second_route, workspace);
    if (route_pair_pool.membership[membership_index] != 0) {
        return;
    }
    route_pair_pool.membership[membership_index] = 1;
    route_pair_pool.pairs.emplace_back(first_route, second_route);
}

void initialize_active_route_pair_pool(
    LocalSearchOperator local_search_operator,
    int route_count,
    bool directed,
    LocalSearchWorkspace& workspace) {
    auto& route_pair_pool =
        workspace.active_route_pair_pools[operator_index(local_search_operator)];
    route_pair_pool.pairs.clear();
    route_pair_pool.membership.assign(
        workspace.route_pair_cache_stride * workspace.route_pair_cache_stride,
        0);
    const int pair_count = directed
        ? route_count * (route_count - 1)
        : route_count * (route_count - 1) / 2;
    route_pair_pool.pairs.reserve(static_cast<std::size_t>(pair_count));

    for (int first_route = 0; first_route < route_count; ++first_route) {
        const int second_route_begin = directed ? 0 : first_route + 1;
        for (int second_route = second_route_begin;
             second_route < route_count;
             ++second_route) {
            if (first_route != second_route) {
                add_active_route_pair(
                    route_pair_pool,
                    first_route,
                    second_route,
                    workspace);
            }
        }
    }

    route_pair_pool.observed_route_versions.resize(
        workspace.route_pair_cache_stride);
    for (int route_index = 0; route_index < route_count; ++route_index) {
        route_pair_pool.observed_route_versions[
            static_cast<std::size_t>(route_index)] =
            workspace.route_versions[
                static_cast<std::size_t>(route_index)];
    }
    route_pair_pool.topology_version = workspace.route_topology_version;
    route_pair_pool.route_count = route_count;
    route_pair_pool.directed = directed;
}

void reactivate_incident_route_pairs(
    LocalSearchWorkspace::ActiveRoutePairPool& route_pair_pool,
    int changed_route,
    const LocalSearchWorkspace& workspace) {
    for (int other_route = 0;
         other_route < route_pair_pool.route_count;
         ++other_route) {
        if (other_route == changed_route) {
            continue;
        }
        if (route_pair_pool.directed) {
            add_active_route_pair(
                route_pair_pool,
                changed_route,
                other_route,
                workspace);
            add_active_route_pair(
                route_pair_pool,
                other_route,
                changed_route,
                workspace);
        } else {
            add_active_route_pair(
                route_pair_pool,
                std::min(changed_route, other_route),
                std::max(changed_route, other_route),
                workspace);
        }
    }
}

LocalSearchWorkspace::ActiveRoutePairPool& synchronize_active_route_pairs(
    LocalSearchOperator local_search_operator,
    int route_count,
    bool directed,
    LocalSearchWorkspace& workspace) {
    auto& route_pair_pool =
        workspace.active_route_pair_pools[operator_index(local_search_operator)];
    if (route_pair_pool.topology_version
            != workspace.route_topology_version
        || route_pair_pool.route_count != route_count
        || route_pair_pool.directed != directed) {
        initialize_active_route_pair_pool(
            local_search_operator,
            route_count,
            directed,
            workspace);
    } else {
        for (int route_index = 0; route_index < route_count; ++route_index) {
            const std::size_t version_index =
                static_cast<std::size_t>(route_index);
            if (route_pair_pool.observed_route_versions[version_index]
                == workspace.route_versions[version_index]) {
                continue;
            }
            reactivate_incident_route_pairs(
                route_pair_pool,
                route_index,
                workspace);
            route_pair_pool.observed_route_versions[version_index] =
                workspace.route_versions[version_index];
        }
    }
    return route_pair_pool;
}

std::size_t select_random_active_route_pair(
    const LocalSearchWorkspace::ActiveRoutePairPool& route_pair_pool,
    std::mt19937& random_engine) {
    // Removing failed samples makes this a lazy random permutation prefix.
    std::uniform_int_distribution<std::size_t> select_pair(
        0,
        route_pair_pool.pairs.size() - 1);
    return select_pair(random_engine);
}

void discard_active_route_pair(
    LocalSearchWorkspace::ActiveRoutePairPool& route_pair_pool,
    std::size_t pair_index,
    const LocalSearchWorkspace& workspace) {
    const auto [first_route, second_route] =
        route_pair_pool.pairs[pair_index];
    route_pair_pool.membership[
        route_pair_cache_index(
            first_route,
            second_route,
            workspace)] = 0;
    if (pair_index + 1 != route_pair_pool.pairs.size()) {
        route_pair_pool.pairs[pair_index] =
            route_pair_pool.pairs.back();
    }
    route_pair_pool.pairs.pop_back();
}

void reset_move_impact(LocalSearchWorkspace& workspace) {
    workspace.changed_route_count = 0;
    workspace.changed_routes = {-1, -1};
    workspace.all_routes_changed = false;
}

void record_changed_route(
    int route_index,
    LocalSearchWorkspace& workspace) {
    for (int changed_index = 0;
         changed_index < workspace.changed_route_count;
         ++changed_index) {
        if (workspace.changed_routes[
                static_cast<std::size_t>(changed_index)] == route_index) {
            return;
        }
    }
    if (workspace.changed_route_count
        >= static_cast<int>(workspace.changed_routes.size())) {
        throw std::logic_error(
            "local-search move changed more than two routes");
    }
    workspace.changed_routes[
        static_cast<std::size_t>(workspace.changed_route_count++)] =
            route_index;
}

void record_changed_route_pair(
    int first_route,
    int second_route,
    LocalSearchWorkspace& workspace) {
    record_changed_route(first_route, workspace);
    record_changed_route(second_route, workspace);
}

void record_all_routes_changed(LocalSearchWorkspace& workspace) {
    workspace.all_routes_changed = true;
}

void invalidate_failure_cache_after_move(
    int route_count,
    LocalSearchWorkspace& workspace) {
    if (workspace.all_routes_changed) {
        advance_route_topology_version(workspace);
        for (int route_index = 0; route_index < route_count; ++route_index) {
            workspace.route_versions[static_cast<std::size_t>(route_index)] =
                next_route_version(workspace);
        }
        return;
    }
    if (workspace.changed_route_count == 0) {
        throw std::logic_error(
            "improving local-search move did not report changed routes");
    }
    for (int changed_index = 0;
         changed_index < workspace.changed_route_count;
         ++changed_index) {
        const int route_index = workspace.changed_routes[
            static_cast<std::size_t>(changed_index)];
        workspace.route_versions[static_cast<std::size_t>(route_index)] =
            next_route_version(workspace);
    }
}

struct RoutePairHash {
    std::size_t operator()(const std::pair<int, int>& route_pair) const {
        return route_pair.first * 256 + route_pair.second;
    }
};

void move_node(int* route, int from_index, int to_index) {
    const int node = route[from_index];
    if (from_index < to_index) {
        for (int i = from_index; i < to_index; ++i) {
            route[i] = route[i + 1];
        }
        route[to_index] = node;
    } else if (from_index > to_index) {
        for (int i = from_index; i > to_index; --i) {
            route[i] = route[i - 1];
        }
        route[to_index] = node;
    }
}

void remove_empty_route(Individual& individual, int route_index) {
    const int last_route = individual.num_routes - 1;
    if (route_index != last_route) {
        std::swap(individual.routes[route_index], individual.routes[last_route]);
        std::swap(individual.num_nodes_per_route[route_index], individual.num_nodes_per_route[last_route]);
        std::swap(individual.demand_sum_per_route[route_index], individual.demand_sum_per_route[last_route]);
    }

    individual.num_nodes_per_route[last_route] = 0;
    individual.demand_sum_per_route[last_route] = 0;
    --individual.num_routes;
}

void replace_route(
    Individual& individual,
    int route_index,
    const std::vector<int>& route,
    int demand) {
    std::copy(route.begin(), route.end(), individual.routes[route_index]);
    individual.num_nodes_per_route[route_index] = static_cast<int>(route.size());
    individual.demand_sum_per_route[route_index] = demand;
}

void relocate_customer(
    Individual& individual,
    int source_route,
    int source_node,
    int target_route,
    int target_edge,
    int customer,
    int customer_demand) {
    int* source = individual.routes[source_route];
    for (int i = source_node; i < individual.num_nodes_per_route[source_route] - 1; ++i) {
        source[i] = source[i + 1];
    }
    --individual.num_nodes_per_route[source_route];
    individual.demand_sum_per_route[source_route] -= customer_demand;

    int* target = individual.routes[target_route];
    for (int i = individual.num_nodes_per_route[target_route]; i > target_edge + 1; --i) {
        target[i] = target[i - 1];
    }
    target[target_edge + 1] = customer;
    ++individual.num_nodes_per_route[target_route];
    individual.demand_sum_per_route[target_route] += customer_demand;
}

const std::vector<int>& shuffled_route_order(
    int route_count,
    std::mt19937& random_engine,
    LocalSearchWorkspace& workspace) {
    auto& route_order = workspace.route_order;
    route_order.resize(route_count);
    std::iota(route_order.begin(), route_order.end(), 0);
    std::shuffle(route_order.begin(), route_order.end(), random_engine);
    return route_order;
}

bool improve_with_node_shift_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& random_engine,
    LocalSearchWorkspace& workspace) {
    constexpr LocalSearchOperator local_search_operator =
        LocalSearchOperator::NodeShift;
    const auto& route_order = shuffled_route_order(
        individual.num_routes,
        random_engine,
        workspace);
    for (const int route_index : route_order) {
        if (route_failure_is_cached(
                local_search_operator,
                route_index,
                workspace)) {
            continue;
        }
        int* route = individual.routes[route_index];
        const int length = individual.num_nodes_per_route[route_index];
        if (length <= 4) {
            cache_route_failure(
                local_search_operator,
                route_index,
                workspace);
            continue;
        }

        for (int from_index = 1; from_index < length - 1; ++from_index) {
            for (int to_index = 1; to_index < length - 1; ++to_index) {
                double improvement = 0.0;
                if (from_index < to_index) {
                    const double old_cost =
                        instance.get_distance(route[from_index - 1], route[from_index])
                        + instance.get_distance(route[from_index], route[from_index + 1])
                        + instance.get_distance(route[to_index], route[to_index + 1]);
                    const double new_cost =
                        instance.get_distance(route[from_index - 1], route[from_index + 1])
                        + instance.get_distance(route[to_index], route[from_index])
                        + instance.get_distance(route[from_index], route[to_index + 1]);
                    improvement = old_cost - new_cost;
                } else if (from_index > to_index) {
                    const double old_cost =
                        instance.get_distance(route[from_index - 1], route[from_index])
                        + instance.get_distance(route[from_index], route[from_index + 1])
                        + instance.get_distance(route[to_index - 1], route[to_index]);
                    const double new_cost =
                        instance.get_distance(route[to_index - 1], route[from_index])
                        + instance.get_distance(route[from_index], route[to_index])
                        + instance.get_distance(route[from_index - 1], route[from_index + 1]);
                    improvement = old_cost - new_cost;
                }

                if (improvement <= kImprovementTolerance) {
                    continue;
                }
                move_node(route, from_index, to_index);
                set_upper_cost(individual, individual.upper_cost - improvement);
                record_changed_route(route_index, workspace);
                return true;
            }
        }
        cache_route_failure(
            local_search_operator,
            route_index,
            workspace);
    }
    return false;
}

bool improve_with_inter_route_relocate_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& random_engine,
    LocalSearchWorkspace& workspace) {
    if (individual.num_routes <= 1) {
        return false;
    }

    constexpr LocalSearchOperator local_search_operator =
        LocalSearchOperator::InterRouteRelocate;
    auto& route_pair_pool = synchronize_active_route_pairs(
        local_search_operator,
        individual.num_routes,
        true,
        workspace);
    while (!route_pair_pool.pairs.empty()) {
        const std::size_t pair_index =
            select_random_active_route_pair(
                route_pair_pool,
                random_engine);
        const auto [source_route, target_route] =
            route_pair_pool.pairs[pair_index];
        const int source_length = individual.num_nodes_per_route[source_route];
        const int target_length = individual.num_nodes_per_route[target_route];
        for (int source_node = 1; source_node < source_length - 1; ++source_node) {
            const int customer = individual.routes[source_route][source_node];
            const int customer_demand = instance.get_customer_demand_(customer);
            if (individual.demand_sum_per_route[target_route] + customer_demand > instance.max_vehicle_capa_) {
                continue;
            }

            const double source_improvement =
                instance.get_distance(
                    individual.routes[source_route][source_node - 1],
                    customer)
                + instance.get_distance(
                    customer,
                    individual.routes[source_route][source_node + 1])
                - instance.get_distance(
                    individual.routes[source_route][source_node - 1],
                    individual.routes[source_route][source_node + 1]);
            for (int target_edge = 0; target_edge < target_length - 1; ++target_edge) {
                const int target_from = individual.routes[target_route][target_edge];
                const int target_to = individual.routes[target_route][target_edge + 1];
                const double improvement = source_improvement
                    + instance.get_distance(target_from, target_to)
                    - instance.get_distance(target_from, customer)
                    - instance.get_distance(customer, target_to);
                if (improvement <= kImprovementTolerance) {
                    continue;
                }

                relocate_customer(
                    individual,
                    source_route,
                    source_node,
                    target_route,
                    target_edge,
                    customer,
                    customer_demand);
                set_upper_cost(individual, individual.upper_cost - improvement);
                if (individual.num_nodes_per_route[source_route] == 2) {
                    remove_empty_route(individual, source_route);
                    record_all_routes_changed(workspace);
                } else {
                    record_changed_route_pair(
                        source_route,
                        target_route,
                        workspace);
                }
                return true;
            }
        }
        discard_active_route_pair(
            route_pair_pool,
            pair_index,
            workspace);
    }
    return false;
}

bool improve_with_intra_route_swap_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& random_engine,
    LocalSearchWorkspace& workspace) {
    constexpr LocalSearchOperator local_search_operator =
        LocalSearchOperator::IntraRouteSwap;
    const auto& route_order = shuffled_route_order(
        individual.num_routes,
        random_engine,
        workspace);
    for (const int route_index : route_order) {
        if (route_failure_is_cached(
                local_search_operator,
                route_index,
                workspace)) {
            continue;
        }
        int* route = individual.routes[route_index];
        const int length = individual.num_nodes_per_route[route_index];
        if (length < 5) {
            cache_route_failure(
                local_search_operator,
                route_index,
                workspace);
            continue;
        }

        for (int first_node = 1; first_node < length - 2; ++first_node) {
            for (int second_node = first_node + 2;
                 second_node < length - 1;
                 ++second_node) {
                const double old_cost =
                    instance.get_distance(route[first_node - 1], route[first_node])
                    + instance.get_distance(route[first_node], route[first_node + 1])
                    + instance.get_distance(route[second_node - 1], route[second_node])
                    + instance.get_distance(route[second_node], route[second_node + 1]);
                const double new_cost =
                    instance.get_distance(route[first_node - 1], route[second_node])
                    + instance.get_distance(route[second_node], route[first_node + 1])
                    + instance.get_distance(route[second_node - 1], route[first_node])
                    + instance.get_distance(route[first_node], route[second_node + 1]);
                const double improvement = old_cost - new_cost;
                if (improvement <= kImprovementTolerance) {
                    continue;
                }

                std::swap(route[first_node], route[second_node]);
                set_upper_cost(individual, individual.upper_cost - improvement);
                record_changed_route(route_index, workspace);
                return true;
            }
        }
        cache_route_failure(
            local_search_operator,
            route_index,
            workspace);
    }
    return false;
}

bool improve_with_inter_route_swap_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& random_engine,
    LocalSearchWorkspace& workspace) {
    if (individual.num_routes <= 1) {
        return false;
    }

    constexpr LocalSearchOperator local_search_operator =
        LocalSearchOperator::InterRouteSwap;
    auto& route_pair_pool = synchronize_active_route_pairs(
        local_search_operator,
        individual.num_routes,
        false,
        workspace);
    while (!route_pair_pool.pairs.empty()) {
        const std::size_t pair_index =
            select_random_active_route_pair(
                route_pair_pool,
                random_engine);
        const auto [first_route, second_route] =
            route_pair_pool.pairs[pair_index];
        for (int first_node = 1;
             first_node < individual.num_nodes_per_route[first_route] - 1;
             ++first_node) {
            const int first_customer = individual.routes[first_route][first_node];
            const int first_demand = instance.get_customer_demand_(first_customer);
            for (int second_node = 1;
                 second_node < individual.num_nodes_per_route[second_route] - 1;
                 ++second_node) {
                const int second_customer = individual.routes[second_route][second_node];
                const int second_demand = instance.get_customer_demand_(second_customer);
                if (individual.demand_sum_per_route[first_route] - first_demand + second_demand > instance.max_vehicle_capa_
                    || individual.demand_sum_per_route[second_route] - second_demand + first_demand > instance.max_vehicle_capa_) {
                    continue;
                }

                const double old_cost =
                    instance.get_distance(
                        individual.routes[first_route][first_node - 1],
                        first_customer)
                    + instance.get_distance(
                        first_customer,
                        individual.routes[first_route][first_node + 1])
                    + instance.get_distance(
                        individual.routes[second_route][second_node - 1],
                        second_customer)
                    + instance.get_distance(
                        second_customer,
                        individual.routes[second_route][second_node + 1]);
                const double new_cost =
                    instance.get_distance(
                        individual.routes[first_route][first_node - 1],
                        second_customer)
                    + instance.get_distance(
                        second_customer,
                        individual.routes[first_route][first_node + 1])
                    + instance.get_distance(
                        individual.routes[second_route][second_node - 1],
                        first_customer)
                    + instance.get_distance(
                        first_customer,
                        individual.routes[second_route][second_node + 1]);
                const double improvement = old_cost - new_cost;
                if (improvement <= kImprovementTolerance) {
                    continue;
                }

                std::swap(
                    individual.routes[first_route][first_node],
                    individual.routes[second_route][second_node]);
                individual.demand_sum_per_route[first_route] =
                    individual.demand_sum_per_route[first_route] - first_demand + second_demand;
                individual.demand_sum_per_route[second_route] =
                    individual.demand_sum_per_route[second_route] - second_demand + first_demand;
                set_upper_cost(individual, individual.upper_cost - improvement);
                record_changed_route_pair(
                    first_route,
                    second_route,
                    workspace);
                return true;
            }
        }
        discard_active_route_pair(
            route_pair_pool,
            pair_index,
            workspace);
    }
    return false;
}

struct BestInsertionAfterRemoval {
    double cost = std::numeric_limits<double>::infinity();
    int insertion_index = -1;
};

bool insertion_precedes(
    double cost,
    int insertion_index,
    const LocalSearchWorkspace::InsertionCandidate& candidate) {
    return cost < candidate.cost
        || (cost == candidate.cost
            && insertion_index < candidate.insertion_index);
}

void insert_into_top_three(
    double cost,
    int insertion_index,
    LocalSearchWorkspace::TopThreeInsertions& top_three) {
    for (std::size_t rank = 0; rank < top_three.size(); ++rank) {
        if (!insertion_precedes(cost, insertion_index, top_three[rank])) {
            continue;
        }

        for (std::size_t shifted_rank = top_three.size() - 1;
             shifted_rank > rank;
             --shifted_rank) {
            top_three[shifted_rank] = top_three[shifted_rank - 1];
        }
        top_three[rank] = {cost, insertion_index};
        return;
    }
}

void cache_top_three_insertions(
    const int* customer_route,
    int customer_route_length,
    const int* target_route,
    int target_route_length,
    Case& instance,
    std::vector<LocalSearchWorkspace::TopThreeInsertions>& cache) {
    const int customer_count = customer_route_length - 2;
    cache.resize(static_cast<std::size_t>(customer_count));
    for (int customer_index = 1;
         customer_index < customer_route_length - 1;
         ++customer_index) {
        auto& top_three = cache[static_cast<std::size_t>(customer_index - 1)];
        for (auto& candidate : top_three) {
            candidate = {
                std::numeric_limits<double>::infinity(),
                -1};
        }

        const int customer = customer_route[customer_index];
        for (int insertion_index = 1;
             insertion_index < target_route_length;
             ++insertion_index) {
            const int from = target_route[insertion_index - 1];
            const int to = target_route[insertion_index];
            const double insertion_cost =
                instance.get_distance(from, customer)
                + instance.get_distance(customer, to)
                - instance.get_distance(from, to);
            insert_into_top_three(
                insertion_cost,
                insertion_index,
                top_three);
        }
    }
}

void cache_removal_costs(
    const int* route,
    int length,
    Case& instance,
    std::vector<double>& removal_costs) {
    removal_costs.resize(static_cast<std::size_t>(length - 2));
    for (int removed_index = 1;
         removed_index < length - 1;
         ++removed_index) {
        const int customer = route[removed_index];
        removal_costs[static_cast<std::size_t>(removed_index - 1)] =
            instance.get_distance(
                route[removed_index - 1],
                route[removed_index + 1])
            - instance.get_distance(
                route[removed_index - 1],
                customer)
            - instance.get_distance(
                customer,
                route[removed_index + 1]);
    }
}

void consider_insertion(
    double cost,
    int insertion_index,
    BestInsertionAfterRemoval& best) {
    if (cost < best.cost
        || (cost == best.cost
            && insertion_index < best.insertion_index)) {
        best.cost = cost;
        best.insertion_index = insertion_index;
    }
}

BestInsertionAfterRemoval best_insertion_after_removal(
    const LocalSearchWorkspace::TopThreeInsertions& top_three,
    const int* route,
    int removed_index,
    int customer,
    Case& instance) {
    BestInsertionAfterRemoval best;

    // Removing one customer invalidates only its two incident insertion edges,
    // so top-3 retains the best unaffected edge whenever one exists.
    for (const auto& candidate : top_three) {
        if (candidate.insertion_index < 0
            || candidate.insertion_index == removed_index
            || candidate.insertion_index == removed_index + 1) {
            continue;
        }
        const int reduced_insertion_index =
            candidate.insertion_index < removed_index
            ? candidate.insertion_index
            : candidate.insertion_index - 1;
        consider_insertion(
            candidate.cost,
            reduced_insertion_index,
            best);
    }

    const int from = route[removed_index - 1];
    const int to = route[removed_index + 1];
    const double insertion_cost =
        instance.get_distance(from, customer)
        + instance.get_distance(customer, to)
        - instance.get_distance(from, to);
    consider_insertion(
        insertion_cost,
        removed_index,
        best);
    return best;
}

void build_route_after_swap_star(
    const int* route,
    int length,
    int removed_index,
    int inserted_customer,
    int insertion_index,
    std::vector<int>& result) {
    result.clear();
    result.reserve(static_cast<std::size_t>(length));
    for (int node_index = 0; node_index < length; ++node_index) {
        if (node_index != removed_index) {
            result.push_back(route[node_index]);
        }
    }
    result.insert(result.begin() + insertion_index, inserted_customer);
}

bool improve_with_swap_star_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& random_engine,
    LocalSearchWorkspace& workspace) {
    if (individual.num_routes <= 1) {
        return false;
    }

    constexpr LocalSearchOperator local_search_operator =
        LocalSearchOperator::SwapStar;
    auto& route_pair_pool = synchronize_active_route_pairs(
        local_search_operator,
        individual.num_routes,
        false,
        workspace);
    while (!route_pair_pool.pairs.empty()) {
        const std::size_t pair_index =
            select_random_active_route_pair(
                route_pair_pool,
                random_engine);
        const auto [first_route, second_route] =
            route_pair_pool.pairs[pair_index];
        const int first_length = individual.num_nodes_per_route[first_route];
        const int second_length = individual.num_nodes_per_route[second_route];
        cache_top_three_insertions(
            individual.routes[first_route],
            first_length,
            individual.routes[second_route],
            second_length,
            instance,
            workspace.first_customers_into_second);
        cache_top_three_insertions(
            individual.routes[second_route],
            second_length,
            individual.routes[first_route],
            first_length,
            instance,
            workspace.second_customers_into_first);
        cache_removal_costs(
            individual.routes[first_route],
            first_length,
            instance,
            workspace.first_removal_costs);
        cache_removal_costs(
            individual.routes[second_route],
            second_length,
            instance,
            workspace.second_removal_costs);

        double best_improvement = 0.0;
        int best_first_node = -1;
        int best_second_node = -1;
        int best_first_insertion = -1;
        int best_second_insertion = -1;

        for (int first_node = 1; first_node < first_length - 1; ++first_node) {
            const int first_customer = individual.routes[first_route][first_node];
            const int first_demand = instance.get_customer_demand_(first_customer);
            const double first_removal_cost =
                workspace.first_removal_costs[
                    static_cast<std::size_t>(first_node - 1)];

            for (int second_node = 1;
                 second_node < second_length - 1;
                 ++second_node) {
                const int second_customer =
                    individual.routes[second_route][second_node];
                const int second_demand =
                    instance.get_customer_demand_(second_customer);
                const int first_new_demand =
                    individual.demand_sum_per_route[first_route] - first_demand + second_demand;
                const int second_new_demand =
                    individual.demand_sum_per_route[second_route] - second_demand + first_demand;
                if (first_new_demand > instance.max_vehicle_capa_
                    || second_new_demand > instance.max_vehicle_capa_) {
                    continue;
                }

                const double second_removal_cost =
                    workspace.second_removal_costs[
                        static_cast<std::size_t>(second_node - 1)];
                const BestInsertionAfterRemoval second_into_first =
                    best_insertion_after_removal(
                        workspace.second_customers_into_first[
                            static_cast<std::size_t>(second_node - 1)],
                        individual.routes[first_route],
                        first_node,
                        second_customer,
                        instance);
                const BestInsertionAfterRemoval first_into_second =
                    best_insertion_after_removal(
                        workspace.first_customers_into_second[
                            static_cast<std::size_t>(first_node - 1)],
                        individual.routes[second_route],
                        second_node,
                        first_customer,
                        instance);
                const double improvement = -(
                    first_removal_cost
                    + second_into_first.cost
                    + second_removal_cost
                    + first_into_second.cost);
                if (improvement > best_improvement + kImprovementTolerance) {
                    best_improvement = improvement;
                    best_first_node = first_node;
                    best_second_node = second_node;
                    best_first_insertion = second_into_first.insertion_index;
                    best_second_insertion = first_into_second.insertion_index;
                }
            }
        }

        if (best_improvement <= kImprovementTolerance) {
            discard_active_route_pair(
                route_pair_pool,
                pair_index,
                workspace);
            continue;
        }

        const int first_customer =
            individual.routes[first_route][best_first_node];
        const int second_customer =
            individual.routes[second_route][best_second_node];
        const int first_new_demand =
            individual.demand_sum_per_route[first_route]
            - instance.get_customer_demand_(first_customer)
            + instance.get_customer_demand_(second_customer);
        const int second_new_demand =
            individual.demand_sum_per_route[second_route]
            - instance.get_customer_demand_(second_customer)
            + instance.get_customer_demand_(first_customer);
        build_route_after_swap_star(
            individual.routes[first_route],
            first_length,
            best_first_node,
            second_customer,
            best_first_insertion,
            workspace.first_route_buffer);
        build_route_after_swap_star(
            individual.routes[second_route],
            second_length,
            best_second_node,
            first_customer,
            best_second_insertion,
            workspace.second_route_buffer);
        replace_route(
            individual,
            first_route,
            workspace.first_route_buffer,
            first_new_demand);
        replace_route(
            individual,
            second_route,
            workspace.second_route_buffer,
            second_new_demand);
        set_upper_cost(individual, 
            individual.upper_cost - best_improvement);
        record_changed_route_pair(
            first_route,
            second_route,
            workspace);
        return true;
    }
    return false;
}

bool improve_with_two_opt_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& random_engine,
    LocalSearchWorkspace& workspace) {
    constexpr LocalSearchOperator local_search_operator =
        LocalSearchOperator::TwoOpt;
    const auto& route_order = shuffled_route_order(
        individual.num_routes,
        random_engine,
        workspace);
    for (const int route_index : route_order) {
        if (route_failure_is_cached(
                local_search_operator,
                route_index,
                workspace)) {
            continue;
        }
        int* route = individual.routes[route_index];
        const int length = individual.num_nodes_per_route[route_index];
        for (int first_node = 1; first_node < length - 2; ++first_node) {
            for (int second_node = first_node + 1;
                 second_node < length - 1;
                 ++second_node) {
                const double old_cost =
                    instance.get_distance(route[first_node - 1], route[first_node])
                    + instance.get_distance(route[second_node], route[second_node + 1]);
                const double new_cost =
                    instance.get_distance(route[first_node - 1], route[second_node])
                    + instance.get_distance(route[first_node], route[second_node + 1]);
                const double improvement = old_cost - new_cost;
                if (improvement <= kImprovementTolerance) {
                    continue;
                }

                std::reverse(route + first_node, route + second_node + 1);
                set_upper_cost(individual, individual.upper_cost - improvement);
                record_changed_route(route_index, workspace);
                return true;
            }
        }
        cache_route_failure(
            local_search_operator,
            route_index,
            workspace);
    }
    return false;
}

void build_two_opt_star_head_to_head_routes(
    const int* first_route,
    int first_length,
    int first_node,
    const int* second_route,
    int second_length,
    int second_node,
    std::vector<int>& first_result,
    std::vector<int>& second_result) {
    first_result.clear();
    first_result.reserve(
        static_cast<std::size_t>(first_node + second_node + 2));
    first_result.insert(
        first_result.end(),
        first_route,
        first_route + first_node + 1);
    for (int node = second_node; node >= 0; --node) {
        first_result.push_back(second_route[node]);
    }

    second_result.clear();
    second_result.reserve(static_cast<std::size_t>(
        first_length - first_node - 1
        + second_length - second_node - 1));
    for (int node = first_length - 1; node >= first_node + 1; --node) {
        second_result.push_back(first_route[node]);
    }
    second_result.insert(
        second_result.end(),
        second_route + second_node + 1,
        second_route + second_length);
}

void build_two_opt_star_head_to_tail_routes(
    const int* first_route,
    int first_length,
    int first_node,
    const int* second_route,
    int second_length,
    int second_node,
    std::vector<int>& first_result,
    std::vector<int>& second_result) {
    first_result.clear();
    first_result.reserve(static_cast<std::size_t>(
        first_node + 1
        + second_length - second_node - 1));
    first_result.insert(
        first_result.end(),
        first_route,
        first_route + first_node + 1);
    first_result.insert(
        first_result.end(),
        second_route + second_node + 1,
        second_route + second_length);

    second_result.clear();
    second_result.reserve(static_cast<std::size_t>(
        second_node + 1
        + first_length - first_node - 1));
    second_result.insert(
        second_result.end(),
        second_route,
        second_route + second_node + 1);
    second_result.insert(
        second_result.end(),
        first_route + first_node + 1,
        first_route + first_length);
}

bool improve_with_two_opt_star_head_to_head_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& random_engine,
    LocalSearchWorkspace& workspace) {
    if (individual.num_routes <= 1) {
        return false;
    }

    constexpr LocalSearchOperator local_search_operator =
        LocalSearchOperator::TwoOptStarHeadToHead;
    auto& route_pair_pool = synchronize_active_route_pairs(
        local_search_operator,
        individual.num_routes,
        false,
        workspace);
    while (!route_pair_pool.pairs.empty()) {
        const std::size_t pair_index =
            select_random_active_route_pair(
                route_pair_pool,
                random_engine);
        const auto [first_route, second_route] =
            route_pair_pool.pairs[pair_index];
        const int first_length = individual.num_nodes_per_route[first_route];
        const int second_length = individual.num_nodes_per_route[second_route];
        int first_prefix_demand = 0;
        for (int first_node = 0; first_node < first_length - 1; ++first_node) {
            first_prefix_demand += instance.get_customer_demand_(
                individual.routes[first_route][first_node]);
            int second_prefix_demand = 0;
            for (int second_node = 0; second_node < second_length - 1; ++second_node) {
                second_prefix_demand += instance.get_customer_demand_(
                    individual.routes[second_route][second_node]);
                const bool leaves_routes_unchanged =
                    (first_node == first_length - 2 && second_node == 0)
                    || (first_node == 0 && second_node == second_length - 2);
                if (leaves_routes_unchanged) {
                    continue;
                }

                const int first_new_demand = first_prefix_demand + second_prefix_demand;
                const int second_new_demand =
                    individual.demand_sum_per_route[first_route] - first_prefix_demand
                    + individual.demand_sum_per_route[second_route] - second_prefix_demand;
                if (first_new_demand > instance.max_vehicle_capa_
                    || second_new_demand > instance.max_vehicle_capa_) {
                    continue;
                }

                const double old_cost =
                    instance.get_distance(
                        individual.routes[first_route][first_node],
                        individual.routes[first_route][first_node + 1])
                    + instance.get_distance(
                        individual.routes[second_route][second_node],
                        individual.routes[second_route][second_node + 1]);
                const double new_cost =
                    instance.get_distance(
                        individual.routes[first_route][first_node],
                        individual.routes[second_route][second_node])
                    + instance.get_distance(
                        individual.routes[first_route][first_node + 1],
                        individual.routes[second_route][second_node + 1]);
                const double improvement = old_cost - new_cost;
                if (improvement <= kImprovementTolerance) {
                    continue;
                }

                build_two_opt_star_head_to_head_routes(
                    individual.routes[first_route],
                    first_length,
                    first_node,
                    individual.routes[second_route],
                    second_length,
                    second_node,
                    workspace.first_route_buffer,
                    workspace.second_route_buffer);
                replace_route(
                    individual,
                    first_route,
                    workspace.first_route_buffer,
                    first_new_demand);
                replace_route(
                    individual,
                    second_route,
                    workspace.second_route_buffer,
                    second_new_demand);
                set_upper_cost(individual, individual.upper_cost - improvement);
                if (individual.num_nodes_per_route[first_route] == 2) {
                    remove_empty_route(individual, first_route);
                    record_all_routes_changed(workspace);
                } else if (individual.num_nodes_per_route[second_route] == 2) {
                    remove_empty_route(individual, second_route);
                    record_all_routes_changed(workspace);
                } else {
                    record_changed_route_pair(
                        first_route,
                        second_route,
                        workspace);
                }
                return true;
            }
        }
        discard_active_route_pair(
            route_pair_pool,
            pair_index,
            workspace);
    }
    return false;
}

bool improve_with_two_opt_star_head_to_tail_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& random_engine,
    LocalSearchWorkspace& workspace) {
    if (individual.num_routes <= 1) {
        return false;
    }

    constexpr LocalSearchOperator local_search_operator =
        LocalSearchOperator::TwoOptStarHeadToTail;
    auto& route_pair_pool = synchronize_active_route_pairs(
        local_search_operator,
        individual.num_routes,
        false,
        workspace);
    while (!route_pair_pool.pairs.empty()) {
        const std::size_t pair_index =
            select_random_active_route_pair(
                route_pair_pool,
                random_engine);
        const auto [first_route, second_route] =
            route_pair_pool.pairs[pair_index];
        const int first_length = individual.num_nodes_per_route[first_route];
        const int second_length = individual.num_nodes_per_route[second_route];
        int first_prefix_demand = 0;
        for (int first_node = 0; first_node < first_length - 1; ++first_node) {
            first_prefix_demand += instance.get_customer_demand_(
                individual.routes[first_route][first_node]);
            int second_prefix_demand = 0;
            for (int second_node = 0; second_node < second_length - 1; ++second_node) {
                second_prefix_demand += instance.get_customer_demand_(
                    individual.routes[second_route][second_node]);
                const bool leaves_routes_unchanged =
                    (first_node == 0 && second_node == 0)
                    || (first_node == first_length - 2
                        && second_node == second_length - 2);
                if (leaves_routes_unchanged) {
                    continue;
                }

                const int first_new_demand = first_prefix_demand
                    + individual.demand_sum_per_route[second_route] - second_prefix_demand;
                const int second_new_demand = second_prefix_demand
                    + individual.demand_sum_per_route[first_route] - first_prefix_demand;
                if (first_new_demand > instance.max_vehicle_capa_
                    || second_new_demand > instance.max_vehicle_capa_) {
                    continue;
                }

                const double old_cost =
                    instance.get_distance(
                        individual.routes[first_route][first_node],
                        individual.routes[first_route][first_node + 1])
                    + instance.get_distance(
                        individual.routes[second_route][second_node],
                        individual.routes[second_route][second_node + 1]);
                const double new_cost =
                    instance.get_distance(
                        individual.routes[first_route][first_node],
                        individual.routes[second_route][second_node + 1])
                    + instance.get_distance(
                        individual.routes[second_route][second_node],
                        individual.routes[first_route][first_node + 1]);
                const double improvement = old_cost - new_cost;
                if (improvement <= kImprovementTolerance) {
                    continue;
                }

                build_two_opt_star_head_to_tail_routes(
                    individual.routes[first_route],
                    first_length,
                    first_node,
                    individual.routes[second_route],
                    second_length,
                    second_node,
                    workspace.first_route_buffer,
                    workspace.second_route_buffer);
                replace_route(
                    individual,
                    first_route,
                    workspace.first_route_buffer,
                    first_new_demand);
                replace_route(
                    individual,
                    second_route,
                    workspace.second_route_buffer,
                    second_new_demand);
                set_upper_cost(individual, individual.upper_cost - improvement);
                if (individual.num_nodes_per_route[first_route] == 2) {
                    remove_empty_route(individual, first_route);
                    record_all_routes_changed(workspace);
                } else if (individual.num_nodes_per_route[second_route] == 2) {
                    remove_empty_route(individual, second_route);
                    record_all_routes_changed(workspace);
                } else {
                    record_changed_route_pair(
                        first_route,
                        second_route,
                        workspace);
                }
                return true;
            }
        }
        discard_active_route_pair(
            route_pair_pool,
            pair_index,
            workspace);
    }
    return false;
}

bool improve_with_neighborhood_one_move(
    Neighborhood neighborhood,
    Individual& individual,
    Case& instance,
    std::mt19937& random_engine,
    LocalSearchWorkspace& workspace) {
    switch (neighborhood) {
        case Neighborhood::TwoOpt:
            return improve_with_two_opt_one_move(
                individual,
                instance,
                random_engine,
                workspace);
        case Neighborhood::NodeShift:
            return improve_with_node_shift_one_move(
                individual,
                instance,
                random_engine,
                workspace);
        case Neighborhood::InterRouteRelocate:
            return improve_with_inter_route_relocate_one_move(
                individual,
                instance,
                random_engine,
                workspace);
        case Neighborhood::InterRouteSwap:
            return improve_with_inter_route_swap_one_move(
                individual,
                instance,
                random_engine,
                workspace);
        case Neighborhood::SwapStar:
            return improve_with_swap_star_one_move(
                individual,
                instance,
                random_engine,
                workspace);
        case Neighborhood::IntraRouteSwap:
            return improve_with_intra_route_swap_one_move(
                individual,
                instance,
                random_engine,
                workspace);
        case Neighborhood::TwoOptStarHeadToHead:
            return improve_with_two_opt_star_head_to_head_one_move(
                individual,
                instance,
                random_engine,
                workspace);
        case Neighborhood::TwoOptStarHeadToTail:
            return improve_with_two_opt_star_head_to_tail_one_move(
                individual,
                instance,
                random_engine,
                workspace);
        case Neighborhood::TwoOptStar:
            return false;
    }
    return false;
}

int scaled_move_limit_for_action(
    const Individual& individual,
    const Case& instance,
    LocalSearchAction action,
    const LocalSearchDepthConfig& depth_config) {
    const int solution_scale = instance.num_customer_ + individual.num_routes;
    double fraction = depth_config.weak_move_fraction;
    switch (action) {
        case LocalSearchAction::Weak:
            fraction = depth_config.weak_move_fraction;
            break;
        case LocalSearchAction::Medium:
            fraction = depth_config.medium_move_fraction;
            break;
        case LocalSearchAction::Strong:
            fraction = depth_config.strong_move_fraction;
            break;
    }
    return std::max(1, static_cast<int>(std::ceil(fraction * solution_scale)));
}

}  // namespace

LocalSearchDepthConfig local_search_depth_config(
    LocalSearchDepthProfile profile) {
    switch (profile) {
        case LocalSearchDepthProfile::D1:
            return {0.02, 0.05, 0.15, 64};
        case LocalSearchDepthProfile::D2:
            return {0.02, 0.075, 0.225, 96};
        case LocalSearchDepthProfile::D3:
            return {0.02, 0.10, 0.30, 128};
        case LocalSearchDepthProfile::D4:
            return {0.02, 0.15, 0.45, 192};
        case LocalSearchDepthProfile::D5:
            return {0.02, 0.20, 0.60, 256};
        case LocalSearchDepthProfile::D6:
            return {0.02, 0.25, 0.75, 320};
    }
    throw std::logic_error("unknown local-search depth profile");
}

const char* local_search_depth_profile_name(
    LocalSearchDepthProfile profile) {
    switch (profile) {
        case LocalSearchDepthProfile::D1:
            return "d1";
        case LocalSearchDepthProfile::D2:
            return "d2";
        case LocalSearchDepthProfile::D3:
            return "d3";
        case LocalSearchDepthProfile::D4:
            return "d4";
        case LocalSearchDepthProfile::D5:
            return "d5";
        case LocalSearchDepthProfile::D6:
            return "d6";
    }
    throw std::logic_error("unknown local-search depth profile");
}

const char* MaLeader::operator_name(
    LocalSearchOperator local_search_operator) {
    switch (local_search_operator) {
        case LocalSearchOperator::NodeShift:
            return "node_shift";
        case LocalSearchOperator::InterRouteRelocate:
            return "inter_route_relocate";
        case LocalSearchOperator::IntraRouteSwap:
            return "intra_route_swap";
        case LocalSearchOperator::InterRouteSwap:
            return "inter_route_swap";
        case LocalSearchOperator::SwapStar:
            return "swap_star";
        case LocalSearchOperator::TwoOpt:
            return "two_opt";
        case LocalSearchOperator::TwoOptStarHeadToHead:
            return "two_opt_star_head_to_head";
        case LocalSearchOperator::TwoOptStarHeadToTail:
            return "two_opt_star_head_to_tail";
        case LocalSearchOperator::Count:
            return "unknown";
    }
    return "unknown";
}

LocalSearchResult MaLeader::improve_with_eight_neighborhood_rvnd_one_move(
    Individual& individual,
    Case& instance,
    std::mt19937& random_engine,
    LocalSearchAction action,
    LocalSearchWorkspace& workspace,
    double gamma_upper_bound,
    const LocalSearchDepthConfig& depth_config) {
    const int move_limit = scaled_move_limit_for_action(
        individual,
        instance,
        action,
        depth_config);

    LocalSearchSession session;
    begin_eight_neighborhood_rvnd_one_move_session(
        individual,
        session,
        workspace);
    if (action != LocalSearchAction::Strong) {
        return continue_eight_neighborhood_rvnd_one_move_session(
            individual,
            instance,
            random_engine,
            session,
            move_limit,
            workspace,
            gamma_upper_bound);
    }

    // Strong: weak probe first, then continue up to the strong move cap and the strong
    // distance-call cap (a multiple of the probe's distance calls).
    const double upper_cost_before = individual.upper_cost;
    const int weak_move_limit = scaled_move_limit_for_action(
        individual,
        instance,
        LocalSearchAction::Weak,
        depth_config);
    LocalSearchResult result =
        continue_eight_neighborhood_rvnd_one_move_session(
            individual,
            instance,
            random_engine,
            session,
            weak_move_limit,
            workspace,
            gamma_upper_bound);
    if (!result.reached_local_optimum) {
        const LocalSearchResult continuation =
            continue_eight_neighborhood_rvnd_one_move_session(
                individual,
                instance,
                random_engine,
                session,
                move_limit,
                workspace,
                gamma_upper_bound,
                strong_distance_call_limit(
                    result.distance_calls_used,
                    depth_config));
        result.accepted_moves += continuation.accepted_moves;
        result.neighborhood_calls += continuation.neighborhood_calls;
        result.distance_calls_used += continuation.distance_calls_used;
        result.reached_local_optimum =
            continuation.reached_local_optimum;
        result.hit_move_limit = continuation.hit_move_limit;
        result.hit_distance_call_limit =
            continuation.hit_distance_call_limit;
        for (std::size_t operator_index = 0;
             operator_index < LOCAL_SEARCH_OPERATOR_COUNT;
             ++operator_index) {
            auto& total_stats = result.operator_stats[operator_index];
            const auto& continuation_stats =
                continuation.operator_stats[operator_index];
            total_stats.calls += continuation_stats.calls;
            total_stats.accepts += continuation_stats.accepts;
            total_stats.distance_calls +=
                continuation_stats.distance_calls;
            total_stats.upper_gain += continuation_stats.upper_gain;
            total_stats.gamma_crosses +=
                continuation_stats.gamma_crosses;
        }
    }
    result.move_limit = move_limit;
    result.relative_upper_improvement = upper_cost_before > 0.0
        ? (upper_cost_before - individual.upper_cost)
            / upper_cost_before
        : 0.0;
    return result;
}

LocalSearchResult MaLeader::improve_unrestricted(
    Individual& individual,
    Case& instance,
    std::mt19937& random_engine,
    LocalSearchWorkspace& workspace,
    double gamma_upper_bound) {
    LocalSearchSession session;
    begin_eight_neighborhood_rvnd_one_move_session(
        individual,
        session,
        workspace);
    return continue_eight_neighborhood_rvnd_one_move_session(
        individual,
        instance,
        random_engine,
        session,
        -1,
        workspace,
        gamma_upper_bound);
}

void MaLeader::begin_eight_neighborhood_rvnd_one_move_session(
    Individual& individual,
    LocalSearchSession& session,
    LocalSearchWorkspace& workspace) {
    begin_failure_cache_session(individual.num_routes, workspace);
    session.initialized = true;
    session.total_accepted_moves = 0;
    session.total_neighborhood_calls = 0;
    session.total_distance_calls = 0;
    session.active_operators.assign(
        kEightOperators.begin(),
        kEightOperators.end());
}

LocalSearchResult MaLeader::continue_eight_neighborhood_rvnd_one_move_session(
    Individual& individual,
    Case& instance,
    std::mt19937& random_engine,
    LocalSearchSession& session,
    int cumulative_move_limit,
    LocalSearchWorkspace& workspace,
    double gamma_upper_bound,
    std::uint64_t cumulative_distance_call_limit) {
    if (!session.initialized) {
        throw std::logic_error(
            "local-search session must be initialized before continuation");
    }

    const double upper_cost_before = individual.upper_cost;
    const std::uint64_t distance_calls_before =
        instance.get_distance_calls();
    LocalSearchResult result;
    result.move_limit = cumulative_move_limit;

    while (!session.active_operators.empty()) {
        if (cumulative_move_limit >= 0
            && session.total_accepted_moves >= cumulative_move_limit) {
            result.hit_move_limit = true;
            break;
        }
        if (session.total_distance_calls
            >= cumulative_distance_call_limit) {
            result.hit_distance_call_limit = true;
            break;
        }

        std::uniform_int_distribution<std::size_t> select_operator(
            0,
            session.active_operators.size() - 1);
        const std::size_t selected_index = select_operator(random_engine);
        const LocalSearchOperator selected_operator =
            session.active_operators[selected_index];
        const Neighborhood selected_neighborhood =
            neighborhood_for_operator(selected_operator);
        LocalSearchOperatorStats& operator_stats =
            result.operator_stats[operator_index(selected_operator)];
        const std::uint64_t operator_distance_calls_before =
            instance.get_distance_calls();
        const double operator_upper_cost_before =
            individual.upper_cost;
        const bool outside_gamma_before =
            operator_upper_cost_before > gamma_upper_bound;

        ++result.neighborhood_calls;
        ++session.total_neighborhood_calls;
        ++operator_stats.calls;
        reset_move_impact(workspace);
        const bool improved = improve_with_neighborhood_one_move(
            selected_neighborhood,
            individual,
            instance,
            random_engine,
            workspace);
        const std::uint64_t operator_distance_calls =
            instance.get_distance_calls()
            - operator_distance_calls_before;
        operator_stats.distance_calls += operator_distance_calls;
        session.total_distance_calls += operator_distance_calls;

        if (improved) {
            invalidate_failure_cache_after_move(
                individual.num_routes,
                workspace);
            ++result.accepted_moves;
            ++session.total_accepted_moves;
            ++operator_stats.accepts;
            operator_stats.upper_gain +=
                operator_upper_cost_before - individual.upper_cost;
            if (outside_gamma_before
                && individual.upper_cost <= gamma_upper_bound) {
                ++operator_stats.gamma_crosses;
            }
            session.active_operators.assign(
                kEightOperators.begin(),
                kEightOperators.end());
        } else {
            session.active_operators.erase(
                session.active_operators.begin() + selected_index);
        }
    }

    result.distance_calls_used =
        instance.get_distance_calls() - distance_calls_before;
    result.relative_upper_improvement = upper_cost_before > 0.0
        ? (upper_cost_before - individual.upper_cost)
            / upper_cost_before
        : 0.0;
    result.reached_local_optimum = session.active_operators.empty();
    individual.upper_locally_optimal = result.reached_local_optimum;
    return result;
}

int MaLeader::move_limit_for_action(
    const Individual& individual,
    const Case& instance,
    LocalSearchAction action,
    const LocalSearchDepthConfig& depth_config) {
    return scaled_move_limit_for_action(
        individual,
        instance,
        action,
        depth_config);
}

std::uint64_t MaLeader::strong_distance_call_limit(
    std::uint64_t weak_distance_calls,
    const LocalSearchDepthConfig& depth_config) {
    const std::uint64_t base_calls =
        std::max<std::uint64_t>(weak_distance_calls, 1);
    if (base_calls
        > std::numeric_limits<std::uint64_t>::max()
            / depth_config.strong_weak_call_multiplier) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return base_calls * depth_config.strong_weak_call_multiplier;
}

const char* local_search_action_name(const LocalSearchAction action) {
    switch (action) {
        case LocalSearchAction::Weak:
            return "weak";
        case LocalSearchAction::Medium:
            return "medium";
        case LocalSearchAction::Strong:
            return "strong";
    }
    return "unknown";
}
