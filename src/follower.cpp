//
// Created by Yinghao Qin on 09/06/2025.
//

#include "follower.hpp"
#include <algorithm>
#include <cmath>
#include <list>
#include <stack>
#include <cassert>
#include <limits>
#include <vector>
#include <numeric>
#include <iomanip>
#include <fstream>

using namespace std;

Follower::Follower(Case* instance, Preprocessor* preprocessor) {
    this->instance = instance;
    this->preprocessor = preprocessor;

    route_cap = preprocessor->route_cap_;
    node_cap = preprocessor->node_cap_;

    // Charged routes are longer than upper-level routes (stations are inserted), so the lower-level
    // route rows get their own capacity. An upper route has at most node_cap nodes and the standard
    // follower inserts at most one station per edge, so 2 * node_cap always suffices; the final
    // refinement checks the length and falls back to one station per edge if its placement would not fit.
    lower_node_cap = 2 * node_cap;
    prepare_temp_buffers(node_cap);

    this->num_routes = 0;
    this->lower_routes = new int *[route_cap];
    for (int i = 0; i < route_cap; ++i) {
        this->lower_routes[i] = new int [lower_node_cap];
        memset(this->lower_routes[i], 0, sizeof(int) * lower_node_cap);
    }
    this->lower_num_nodes_per_route = new int [route_cap];
    memset(this->lower_num_nodes_per_route, 0, sizeof(int) * route_cap);
    this->lower_cost_per_route = new double [route_cap];
    std::fill_n(this->lower_cost_per_route, route_cap, 0.0);
    this->lower_cost = 0;
}

Follower::~Follower() {
    for (int i = 0; i < this->route_cap; ++i) {
        delete[] this->lower_routes[i];
    }
    delete[] this->lower_routes;
    delete[] this->lower_num_nodes_per_route;
    delete[] this->lower_cost_per_route;

    release_temp_buffers();
}

void Follower::prepare_temp_buffers(const int required_size) const {
    if (buffer_size >= required_size) return;

    // delete old memory
    delete[] temp_route;
    delete[] temp_chosen_pos;
    delete[] temp_best_chosen_pos;
    delete[] temp_accumulated_distance;

    // allocate new memory
    buffer_size = required_size;
    temp_route = new int[buffer_size];
    temp_chosen_pos = new int[buffer_size];
    temp_best_chosen_pos = new int[buffer_size];
    temp_accumulated_distance = new double[buffer_size];

    temp_station_inserted.reserve(buffer_size);
}

void Follower::release_temp_buffers() const {
    delete[] temp_route;
    delete[] temp_chosen_pos;
    delete[] temp_best_chosen_pos;
    delete[] temp_accumulated_distance;
}

void Follower::run_for_single_route(const int route_idx) const {
    if (lower_num_nodes_per_route[route_idx] <= 2) {
        lower_cost_per_route[route_idx] = 0.0;
        return;
    }
    const double cost_SE = insert_station_by_simple_enum( lower_routes[route_idx], lower_num_nodes_per_route[route_idx]);

    if (cost_SE == -1) {
        const double cost_RE = insert_station_by_remove_enum( lower_routes[route_idx], lower_num_nodes_per_route[route_idx]);
        if (cost_RE == -1) {
            lower_cost_per_route[route_idx] = INFEASIBLE;
        } else {
            lower_cost_per_route[route_idx] = cost_RE;
        }
    } else {
        lower_cost_per_route[route_idx] = cost_SE;
    }
}

void Follower::run(const PartialSolution* partial_sol) {
    // move_type: 0 for intra-route, 1 for inter-route, 2 for inter-route with empty route
    if (partial_sol->move_type == 0) {
        assert(partial_sol->idx1 != -1 && "The partial solution passed is wrong!");

        // intra-route move
        const int idx = partial_sol->idx1;

        memcpy(lower_routes[idx], partial_sol->route1, sizeof(int) * node_cap);
        lower_num_nodes_per_route[idx] = partial_sol->length1;
        run_for_single_route(idx);
    } else if (partial_sol->move_type == 1) {
        assert(partial_sol->idx1 != -1 && partial_sol->idx2 != -1 && "The partial solution passed is wrong!");

        // inter-route move
        const int idx1 = partial_sol->idx1;
        const int idx2 = partial_sol->idx2;

        if (!partial_sol->is_empty1 && !partial_sol->is_empty2) {
            memcpy(lower_routes[idx1], partial_sol->route1, sizeof(int) * node_cap);
            lower_num_nodes_per_route[idx1] = partial_sol->length1;

            memcpy(lower_routes[idx2], partial_sol->route2, sizeof(int) * node_cap);
            lower_num_nodes_per_route[idx2] = partial_sol->length2;

            run_for_single_route(idx1);
            run_for_single_route(idx2);
        } else if (!partial_sol->is_empty1) {
            // r1 is not empty, r2 is empty

            memcpy(lower_routes[idx1], partial_sol->route1, sizeof(int) * node_cap);
            lower_num_nodes_per_route[idx1] = partial_sol->length1;
            run_for_single_route(idx1);

            const int last = num_routes - 1;
            if (idx2 != last) {
                memcpy(lower_routes[idx2], lower_routes[last], sizeof(int) * lower_node_cap);
                lower_num_nodes_per_route[idx2] = lower_num_nodes_per_route[last];
                lower_cost_per_route[idx2] = lower_cost_per_route[last];
            }
            memset(lower_routes[last], 0, sizeof(int) * lower_node_cap);
            lower_num_nodes_per_route[last] = 0;
            lower_cost_per_route[last] = 0.0;
            num_routes--;

        } else if (!partial_sol->is_empty2) {
            // r1 is empty, r2 is not empty

            memcpy(lower_routes[idx2], partial_sol->route2, sizeof(int) * node_cap);
            lower_num_nodes_per_route[idx2] = partial_sol->length2;
            run_for_single_route(idx2);

            const int last = num_routes - 1;
            if (idx1 != last) {
                memcpy(lower_routes[idx1], lower_routes[last], sizeof(int) * lower_node_cap);
                lower_num_nodes_per_route[idx1] = lower_num_nodes_per_route[last];
                lower_cost_per_route[idx1] = lower_cost_per_route[last];
            }
            memset(lower_routes[last], 0, sizeof(int) * lower_node_cap);
            lower_num_nodes_per_route[last] = 0;
            lower_cost_per_route[last] = 0.0;
            num_routes--;

        } else {
            // both routes are empty, it's impossible to happen
            assert(false && "The partial solution passed is wrong!");
        }

    }

    lower_cost = std::accumulate(lower_cost_per_route, lower_cost_per_route + num_routes, 0.0);
}

void Follower::refine(Individual* ind) {
    load_individual(ind);

    lower_cost = 0.0;
    for (int i = 0; i < num_routes; ++i) {
        const double cost = insert_station_by_all_enumeration(lower_routes[i], lower_num_nodes_per_route[i]);
        lower_cost_per_route[i] = (cost < 0.0) ? INFEASIBLE : cost;
        lower_cost += lower_cost_per_route[i];
    }

    export_individual(ind);
}

// The `run_with_variables` method is theoretically equivalent to `run`, but the latter
// doesn't require the follower to maintain intermediate variables (e.g., lower_routes) and thus
// is more memory efficient. However, the `run_with_variables` method can directly keep all the variables
// for logging output.
void Follower::run_with_variables(Individual *ind) {
    load_individual(ind);

    for (int i = 0; i < num_routes; ++i) {
        run_for_single_route(i);
    }
    lower_cost = std::accumulate(lower_cost_per_route, lower_cost_per_route + num_routes, 0.0);

    export_individual(ind);
}

void Follower::run(Individual* ind) {
    this->num_routes = ind->num_routes;

    for (int i = 0; i < num_routes; ++i) {
        const int length = ind->num_nodes_per_route[i];
        if (length <= 2) {
            lower_cost_per_route[i] = 0.0;
            continue;
        }

        const int* route = ind->routes[i];

        if (use_optimal_charging) {
            const double cost_DP = eval_optimal_cost(route, length);
            lower_cost_per_route[i] = (cost_DP < 0) ? INFEASIBLE : cost_DP;
            continue;
        }

        const double cost_SE = eval_simple_enum_cost(route, length);
        double heuristic_cost = cost_SE;
        if (cost_SE < 0) {
            const double cost_RE = eval_remove_enum_cost(route, length);
            heuristic_cost = cost_RE;
            lower_cost_per_route[i] = (cost_RE < 0) ? INFEASIBLE : cost_RE;
        } else {
            lower_cost_per_route[i] = cost_SE;
        }

        if (shadow_optimal) {
            // Exact solve with the reads restored afterwards: the evaluation budget is not affected.
            std::uint64_t key = 14695981039346656037ULL ^ static_cast<std::uint64_t>(length);
            for (int j = 0; j < length; ++j) {
                key ^= static_cast<std::uint64_t>(route[j]) + 0x9e3779b97f4a7c15ULL + (key << 6) + (key >> 2);
                key *= 1099511628211ULL;
            }
            double exact;
            bool first_time = false;
            if (const auto hit = shadow_cache.find(key); hit != shadow_cache.end()) {
                exact = hit->second;
            } else {
                const std::uint64_t saved_calls = instance->distance_calls_;
                exact = eval_optimal_cost(route, length);
                instance->distance_calls_ = saved_calls;
                if (shadow_cache.size() >= shadow_cache_limit) shadow_cache.clear();
                shadow_cache.emplace(key, exact);
                first_time = true;
            }

            double plain = 0.0;
            for (int j = 1; j < length; ++j) plain += instance->distances_[route[j - 1]][route[j]];
            if (first_time && plain > preprocessor->max_cruise_distance_) {
                ShadowStage& st = shadow_stages[shadow_stage()];
                ++st.charging_routes;
                if (cost_SE < 0) ++st.se_failed;
                if (exact < 0) {
                    if (heuristic_cost < 0) ++st.both_infeasible;
                } else if (heuristic_cost < 0) {
                    ++st.false_infeasible;
                } else {
                    const double gap = std::max(0.0, 100.0 * (heuristic_cost - exact) / exact);
                    if (gap <= 1e-7) ++st.se_optimal;
                    st.gap_sum += gap;
                    st.gap_max = std::max(st.gap_max, gap);
                }
                // Reservoir sample of charging routes (private RNG).
                ++shadow_routes_seen;
                std::size_t slot = shadow_routes.size();
                if (slot >= shadow_route_capacity) {
                    slot = std::uniform_int_distribution<std::uint64_t>(0, shadow_routes_seen - 1)(shadow_rng);
                }
                if (slot < shadow_route_capacity) {
                    ShadowRoute sample{shadow_stage(), heuristic_cost, exact, std::vector<int>(route, route + length)};
                    if (slot == shadow_routes.size()) shadow_routes.push_back(std::move(sample));
                    else shadow_routes[slot] = std::move(sample);
                }
            }
            shadow_cost_per_route_total += (exact < 0) ? static_cast<double>(INFEASIBLE) : exact;
        }
    }

    lower_cost = std::accumulate(lower_cost_per_route,
                                 lower_cost_per_route + num_routes,
                                 0.0);

    if (verify_candidates) {
        ++verify_evaluations;
        double optimistic = 0.0;
        for (int i = 0; i < num_routes; ++i) {
            if (lower_cost_per_route[i] < INFEASIBLE) {
                optimistic += lower_cost_per_route[i];
            } else {
                for (int j = 1; j < ind->num_nodes_per_route[i]; ++j) {
                    optimistic += instance->distances_[ind->routes[i][j - 1]][ind->routes[i][j]];
                }
            }
        }
        if (optimistic <= verify_threshold) {
            ++verify_calls;
            const bool was_infeasible = lower_cost >= INFEASIBLE;
            for (int i = 0; i < num_routes; ++i) {
                const int length = ind->num_nodes_per_route[i];
                if (length <= 2) continue;
                const double exact = eval_optimal_cost(ind->routes[i], length);
                lower_cost_per_route[i] = (exact < 0) ? INFEASIBLE : exact;
            }
            const double exact_total = std::accumulate(lower_cost_per_route, lower_cost_per_route + num_routes, 0.0);
            if (was_infeasible && exact_total < INFEASIBLE) ++verify_rescued;
            else if (exact_total < lower_cost - 1e-9) ++verify_improved;
            lower_cost = exact_total;
        }
    }

    if (shadow_optimal) {
        ++shadow_stages[shadow_stage()].calls;
        ind->shadow_lower_cost = shadow_cost_per_route_total;
        shadow_best_seen = std::min(shadow_best_seen, shadow_cost_per_route_total);
        shadow_cost_per_route_total = 0.0;
    }

    export_individual(ind);
}

void Follower::write_verify_log(const std::filesystem::path& directory, const std::string& instance_name) const {
    std::filesystem::create_directories(directory);
    std::ofstream out(directory / ("verify." + instance_name + ".csv"));
    out << "evaluations,verified,improved,rescued\n"
        << verify_evaluations << ',' << verify_calls << ',' << verify_improved << ',' << verify_rescued << "\n";
}

int Follower::shadow_stage() const {
    const double budget = preprocessor->max_evals_ * static_cast<double>(instance->problem_size_);
    if (budget <= 0.0) return 0;
    const int stage = static_cast<int>(10.0 * static_cast<double>(instance->get_distance_calls()) / budget);
    return std::clamp(stage, 0, 9);
}

void Follower::record_decision(const int kind, const bool heuristic_update, const bool exact_update) {
    ShadowStage& st = shadow_stages[shadow_stage()];
    ++st.decisions[kind];
    if (heuristic_update != exact_update) {
        ++st.disagreements[kind];
        if (heuristic_update) ++st.se_only[kind];
        else ++st.dp_only[kind];
    }
}

void Follower::write_shadow_log(const std::filesystem::path& directory, const std::string& instance_name,
                                const double heuristic_final_cost, const double exact_cost_of_heuristic_final) const {
    std::filesystem::create_directories(directory);
    {
        std::ofstream out(directory / ("shadow." + instance_name + ".csv"));
        out << "stage,calls,charging_routes,se_optimal,se_failed,false_infeasible,both_infeasible,gap_sum,gap_max";
        for (int k = 0; k < 3; ++k) {
            out << ",decisions_" << k << ",disagreements_" << k << ",se_only_" << k << ",dp_only_" << k;
        }
        out << "\n";
        out << std::setprecision(10);
        for (int stage = 0; stage < 10; ++stage) {
            const ShadowStage& st = shadow_stages[stage];
            out << stage << ',' << st.calls << ',' << st.charging_routes << ',' << st.se_optimal << ','
                << st.se_failed << ',' << st.false_infeasible << ',' << st.both_infeasible << ','
                << st.gap_sum << ',' << st.gap_max;
            for (int k = 0; k < 3; ++k) {
                out << ',' << st.decisions[k] << ',' << st.disagreements[k] << ',' << st.se_only[k] << ',' << st.dp_only[k];
            }
            out << "\n";
        }
    }
    {
        std::ofstream out(directory / ("shadow_summary." + instance_name + ".csv"));
        out << std::setprecision(10)
            << "heuristic_final_cost,exact_cost_of_heuristic_final,best_exact_cost_evaluated\n"
            << heuristic_final_cost << ',' << exact_cost_of_heuristic_final << ',' << shadow_best_seen << "\n";
    }
    {
        std::ofstream out(directory / ("shadow_routes." + instance_name + ".csv"));
        out << std::setprecision(10) << "stage,heuristic_cost,exact_cost,route\n";
        for (const ShadowRoute& r : shadow_routes) {
            out << r.stage << ',' << r.heuristic_cost << ',' << r.exact_cost << ',';
            for (std::size_t j = 0; j < r.route.size(); ++j) out << (j ? "-" : "") << r.route[j];
            out << "\n";
        }
    }
}

void Follower::load_individual(const Individual* ind) {
    this->num_routes = ind->num_routes;
    for (int i = 0; i < num_routes; ++i) {
        this->lower_num_nodes_per_route[i] = ind->num_nodes_per_route[i];

        // An individual may use a smaller node capacity than the follower (e.g. MA's population);
        // copy its whole row and clear the remainder.
        const int copied = std::min(node_cap, ind->node_cap);
        memcpy(this->lower_routes[i], ind->routes[i], sizeof(int) * copied);
        if (copied < lower_node_cap) {
            memset(this->lower_routes[i] + copied, 0, sizeof(int) * (lower_node_cap - copied));
        }
    }
}

void Follower::export_individual(Individual* ind) const {
    ind->lower_cost = this->lower_cost;
}

double Follower::eval_simple_enum_cost(const int* route, const int length) const {
    if (length <= 2) return 0.0;

    prepare_temp_buffers(length);

    // Copy route into temp buffer (length-sized, not node_cap)
    std::memcpy(temp_route, route, sizeof(int) * length);

    // accumulated distance
    temp_accumulated_distance[0] = 0.0;
    for (int i = 1; i < length; ++i) {
        temp_accumulated_distance[i] =
            temp_accumulated_distance[i - 1] + instance->get_distance(temp_route[i], temp_route[i - 1]);
    }

    const double total_distance = temp_accumulated_distance[length - 1];

    if (total_distance <= preprocessor->max_cruise_distance_) {
        return total_distance;
    }

    // Station counts as in the paper: l_min = ceil(f_U / (Q_b / h)) - 1, then l_min and l_min + 1.
    const int lower_bound = static_cast<int>(std::ceil(total_distance / preprocessor->max_cruise_distance_)) - 1;
    const int upper_bound = lower_bound + 1;

    double final_cost = std::numeric_limits<double>::max();

    // Same rule as insert_station_by_simple_enum (see se_compare_station_counts).
    for (int k = lower_bound; k <= upper_bound; ++k) {
        recursive_charging_placement(
            0, k,
            temp_chosen_pos,
            temp_best_chosen_pos,
            final_cost,
            k,
            temp_route,
            length,
            temp_accumulated_distance
        );

        if (!se_compare_station_counts && final_cost != std::numeric_limits<double>::max()) {
            break;  // original behaviour: stop at the first feasible station count
        }
    }

    return (final_cost != std::numeric_limits<double>::max()) ? final_cost : -1.0;
}

// Cheapest-detour station reachable within max_dis from `from` and within the full range of `to`
// (same selection as Preprocessor::get_best_and_feasible_station), with every distance read charged.
int Follower::best_feasible_station_charged(const int from, const int to, const double max_dis) const {
    int target_station = -1;
    double min_dis = std::numeric_limits<double>::max();
    for (const int station : preprocessor->station_ids_) {
        if (station == from || station == to) continue;
        const double to_station = instance->get_distance(from, station);
        const double from_station = instance->get_distance(station, to);
        if (to_station < max_dis && min_dis > to_station + from_station
            && from_station < preprocessor->max_cruise_distance_) {
            target_station = station;
            min_dis = to_station + from_station;
        }
    }
    return target_station;
}

double Follower::eval_remove_enum_cost(const int* route, const int length) const {
    if (length <= 2) return 0.0;

    prepare_temp_buffers(length);
    std::memcpy(temp_route, route, sizeof(int) * length);

    temp_station_inserted.clear();

    // Step 1: greedy insertion (same logic)
    for (int i = 0; i < length - 1; ++i) {
        double allowed_dis = preprocessor->max_cruise_distance_;
        if (!temp_station_inserted.empty()) {
            allowed_dis -= instance->get_distance(temp_station_inserted.back().station_id, temp_route[i]);
        }

        const int station = best_feasible_station_charged(
            temp_route[i], temp_route[i + 1], allowed_dis
        );
        if (station == -1) return -1.0;
        temp_station_inserted.push_back({i, station});
    }

    // Step 2: remove redundant stations (same logic)
    while (true) {
        int best_idx = -1;
        double max_saved = 0.0;

        for (int k = 0; k < static_cast<int>(temp_station_inserted.size()); ++k) {
            const int from_node = (k == 0) ? temp_route[0] : temp_station_inserted[k - 1].station_id;
            const int to_node   = (k + 1 < static_cast<int>(temp_station_inserted.size()))
                                    ? temp_station_inserted[k + 1].station_id
                                    : temp_route[length - 1];

            const int begin_idx = (k == 0) ? 0 : temp_station_inserted[k - 1].pos + 1;
            const int end_idx   = (k + 1 < static_cast<int>(temp_station_inserted.size()))
                                    ? temp_station_inserted[k + 1].pos
                                    : length - 1;

            double segment_dis = instance->get_distance(from_node, temp_route[begin_idx]);
            for (int i = begin_idx; i < end_idx; ++i) {
                segment_dis += instance->get_distance(temp_route[i], temp_route[i + 1]);
            }
            segment_dis += instance->get_distance(temp_route[end_idx], to_node);

            if (segment_dis <= preprocessor->max_cruise_distance_) {
                const int pos = temp_station_inserted[k].pos;
                const int station = temp_station_inserted[k].station_id;

                const double with_station =
                    instance->get_distance(temp_route[pos], station) +
                    instance->get_distance(station, temp_route[pos + 1]);
                const double direct = instance->get_distance(temp_route[pos], temp_route[pos + 1]);

                const double saved = with_station - direct;
                if (saved > max_saved) {
                    max_saved = saved;
                    best_idx = k;
                }
            }
        }

        if (best_idx == -1) break;
        temp_station_inserted.erase(temp_station_inserted.begin() + best_idx);
    }

    // Step 3: compute total cost without building repaired_route
    double total_cost = 0.0;
    for (int i = 0; i < length - 1; ++i) {
        total_cost += instance->get_distance(temp_route[i], temp_route[i + 1]);
    }

    for (const auto& [pos, station_id] : temp_station_inserted) {
        total_cost -= instance->get_distance(temp_route[pos], temp_route[pos + 1]);
        total_cost += instance->get_distance(temp_route[pos], station_id) +
                      instance->get_distance(station_id, temp_route[pos + 1]);
    }

    return total_cost;
}

double Follower::insert_station_by_simple_enum(int* repaired_route, int& repaired_length) const {
    const int length = repaired_length;

    // Copy route to temp buffer
    memcpy(temp_route, repaired_route, sizeof(int) * length);

    // Compute accumulated distance
    temp_accumulated_distance[0] = 0.0;
    for (int i = 1; i < length; ++i) {
        temp_accumulated_distance[i] = temp_accumulated_distance[i - 1] +
            instance->get_distance(temp_route[i], temp_route[i - 1]);
    }

    const double total_distance = temp_accumulated_distance[length - 1];

    // Case 1: No station needed
    if (total_distance <= preprocessor->max_cruise_distance_) {
        repaired_length = length;
        return total_distance;
    }


    // Compute lower and upper bound for number of stations
    // Station counts as in the paper: l_min = ceil(f_U / (Q_b / h)) - 1, then l_min and l_min + 1.
    const int lower_bound = static_cast<int>(std::ceil(total_distance / preprocessor->max_cruise_distance_)) - 1;
    const int upper_bound = lower_bound + 1;

    double final_cost = numeric_limits<double>::max();

    // With se_compare_station_counts, both l_min and l_min + 1 stations are tried and the cheaper placement is
    // kept (final_cost carries the best over both counts); stopping at the first feasible count can miss a much
    // cheaper placement with one more station (e.g. R103-10: 2 stations 170.22 vs 3 stations 160.41).
    // Without it, the enumeration stops at the first feasible count (original B-LAHC behaviour).
    for (int i = lower_bound; i <= upper_bound; ++i) {
        const double cost_before = final_cost;
        recursive_charging_placement(
            0,
            i,
            temp_chosen_pos,
            temp_best_chosen_pos,
            final_cost,
            i,
            temp_route,
            length,
            temp_accumulated_distance);

        if (final_cost < cost_before) {
            // Build repaired route with inserted stations
            int current_index = 0;
            int idx = 0;

            for (int j = 0; j < i; ++j) {
                const int from = temp_route[temp_best_chosen_pos[j]];
                const int to = temp_route[temp_best_chosen_pos[j] + 1];
                const int station = preprocessor->best_station_[from][to];

                const int num_elements_to_copy = temp_best_chosen_pos[j] + 1 - idx;
                memcpy(&repaired_route[current_index], &temp_route[idx], num_elements_to_copy * sizeof(int));

                current_index += num_elements_to_copy;
                repaired_route[current_index++] = station;
                idx = temp_best_chosen_pos[j] + 1;
            }

            // Copy remaining nodes
            const int remaining_elements_to_copy = length - idx;
            memcpy(&repaired_route[current_index], &temp_route[idx], remaining_elements_to_copy * sizeof(int));
            repaired_length = current_index + remaining_elements_to_copy;
        }

        if (!se_compare_station_counts && final_cost != numeric_limits<double>::max()) {
            break;  // original behaviour: stop at the first feasible station count
        }
    }

    return final_cost != numeric_limits<double>::max() ? final_cost : -1;
}

double Follower::insert_station_by_remove_enum(int* repaired_route, int& repaired_length) const {
    const int length = repaired_length;
    memcpy(temp_route, repaired_route, sizeof(int) * length);

    temp_station_inserted.clear();

    // Step 1: Initial greedy insertion
    for (int i = 0; i < length - 1; ++i) {
        double allowed_dis = preprocessor->max_cruise_distance_;
        if (!temp_station_inserted.empty()) {
            allowed_dis -= instance->get_distance(temp_station_inserted.back().station_id, temp_route[i]);
        }

        const int station = best_feasible_station_charged(temp_route[i], temp_route[i + 1], allowed_dis);
        if (station == -1) return -1;
        temp_station_inserted.push_back({i, station});
    }

    // Step 2: Remove redundant stations
    while (true) {
        int best_idx = -1;
        double max_saved = 0.0;

        for (int k = 0; k < temp_station_inserted.size(); ++k) {
            const int from_node = k == 0 ? temp_route[0] : temp_station_inserted[k - 1].station_id;
            const int to_node   = k + 1 < temp_station_inserted.size() ? temp_station_inserted[k + 1].station_id : temp_route[length - 1];
            const int begin_idx = k == 0 ? 0 : temp_station_inserted[k - 1].pos + 1;
            const int end_idx   = k + 1 < temp_station_inserted.size() ? temp_station_inserted[k + 1].pos : length - 1;

            double segment_dis = instance->get_distance(from_node, temp_route[begin_idx]);
            for (int i = begin_idx; i < end_idx; ++i) {
                segment_dis += instance->get_distance(temp_route[i], temp_route[i + 1]);
            }
            segment_dis += instance->get_distance(temp_route[end_idx], to_node);

            if (segment_dis <= preprocessor->max_cruise_distance_) {
                const int pos = temp_station_inserted[k].pos;
                const int station = temp_station_inserted[k].station_id;
                const double with_station = instance->get_distance(temp_route[pos], station) + instance->get_distance(station, temp_route[pos + 1]);
                const double direct = instance->get_distance(temp_route[pos], temp_route[pos + 1]);

                if (const double saved = with_station - direct; saved > max_saved) {
                    max_saved = saved;
                    best_idx = k;
                }
            }
        }

        if (best_idx == -1) break;
        temp_station_inserted.erase(temp_station_inserted.begin() + best_idx);
    }

    // Step 3: Reconstruct repaired route
    int current_idx = 0;
    int last_pos = 0;
    double total_cost = 0.0;

    for (const auto&[pos, station_id] : temp_station_inserted) {
        const int len = pos + 1 - last_pos;
        memcpy(repaired_route + current_idx, temp_route + last_pos, len * sizeof(int));
        current_idx += len;
        repaired_route[current_idx++] = station_id;
        last_pos = pos + 1;

        total_cost -= instance->get_distance(temp_route[pos], temp_route[pos + 1]);
        total_cost += instance->get_distance(temp_route[pos], station_id) + instance->get_distance(station_id, temp_route[pos + 1]);
    }

    const int remain_len = length - last_pos;
    memcpy(repaired_route + current_idx, temp_route + last_pos, remain_len * sizeof(int));
    repaired_length = current_idx + remain_len;

    // Add base route cost (only once)
    for (int i = 0; i < length - 1; ++i) {
        total_cost += instance->get_distance(temp_route[i], temp_route[i + 1]);
    }

    return total_cost;
}

//void Follower::recursive_charging_placement(int m_len, int n_len, int* chosen_pos, int* best_chosen_pos, double& final_cost, int cur_upper_bound, int* route, int length, vector<double>& accumulated_distance) {
//    for (int i = m_len; i <= length - 1 - n_len; i++) {
//        if (cur_upper_bound == n_len) {
//            double one_dis = instance->get_distance(route[i], preprocessor->best_station_[route[i]][route[i + 1]]);
//            if (accumulated_distance[i] + one_dis > preprocessor->max_cruise_distance_) {
//                break;
//            }
//        }
//        else {
//            int last_pos = chosen_pos[cur_upper_bound - n_len - 1];
//            double one_dis = instance->get_distance(route[last_pos + 1], preprocessor->best_station_[route[last_pos]][route[last_pos + 1]]);
//            double two_dis = instance->get_distance(route[i], preprocessor->best_station_[route[i]][route[i + 1]]);
//            if (accumulated_distance[i] - accumulated_distance[last_pos + 1] + one_dis + two_dis > preprocessor->max_cruise_distance_) {
//                break;
//            }
//        }
//        if (n_len == 1) {
//            double one_dis = accumulated_distance.back() - accumulated_distance[i + 1] + instance->get_distance(preprocessor->best_station_[route[i]][route[i + 1]], route[i + 1]);
//            if (one_dis > preprocessor->max_cruise_distance_) {
//                continue;
//            }
//        }
//
//        chosen_pos[cur_upper_bound - n_len] = i;
//        if (n_len > 1) {
//            recursive_charging_placement(i + 1, n_len - 1, chosen_pos,  best_chosen_pos, final_cost, cur_upper_bound, route, length, accumulated_distance);
//        }
//        else {
//            double dis_sum = accumulated_distance.back();
//            for (int j = 0; j < cur_upper_bound; j++) {
//                int first_node = route[chosen_pos[j]];
//                int second_node = route[chosen_pos[j] + 1];
//                int the_station = preprocessor->best_station_[first_node][second_node];
//                dis_sum -= instance->get_distance(first_node, second_node);
//                dis_sum += instance->get_distance(first_node, the_station);
//                dis_sum += instance->get_distance(second_node, the_station);
//            }
//            if (dis_sum < final_cost) {
//                final_cost = dis_sum;
//                for (int j = 0; j < length; ++j) {
//                    best_chosen_pos[j] = chosen_pos[j];
//                }
//            }
//        }
//    }
//}

/*
 * This stack-based implementation consumes more memory than the recursive one in my experiments.
 * This probably due to the fact that the recursive one will be automatically optimised by the compiler, C++, machine.
 */
void Follower::recursive_charging_placement(const int m_len, const int n_len, int* chosen_pos, int* best_chosen_pos,
                                            double& final_cost, const int cur_upper_bound, const int* route,
                                            const int length, const double* accumulated_distance) const {
    struct State {
        int m_len;
        int n_len;
        int i;
    };

    stack<State> stk;
    stk.push({m_len, n_len, m_len});

    while (!stk.empty()) {
        auto& s = stk.top();

        if (s.n_len == 0) {
            double dis_sum = accumulated_distance[length - 1];
            for (int j = 0; j < cur_upper_bound; ++j) {
                const int first_node = route[chosen_pos[j]];
                const int second_node = route[chosen_pos[j] + 1];
                const int station = preprocessor->best_station_[first_node][second_node];

                // Every distance read is charged (WCCI2020 rule: each read costs 1/n of an evaluation).
                dis_sum -= instance->get_distance(first_node, second_node);
                dis_sum += instance->get_distance(first_node, station);
                dis_sum += instance->get_distance(station, second_node);
            }
            if (dis_sum < final_cost) {
                final_cost = dis_sum;
                for (int j = 0; j < cur_upper_bound; ++j) {
                    best_chosen_pos[j] = chosen_pos[j];
                }
            }
            stk.pop(); // pop at the end
            continue;
        }

        if (s.i <= length - 1 - s.n_len) {
            bool should_pop = false;
            const int first_node = route[s.i];
            const int second_node = route[s.i + 1];
            const int station = preprocessor->best_station_[first_node][second_node]; // has been cached

            if (cur_upper_bound == s.n_len) {
                const double one_dis = instance->get_distance(first_node, station);
                if (accumulated_distance[s.i] + one_dis > preprocessor->max_cruise_distance_) {
                    should_pop = true;
                }
            } else {
                const int last_pos = chosen_pos[cur_upper_bound - s.n_len - 1];
                const double one_dis = instance->get_distance(route[last_pos + 1], preprocessor->best_station_[route[last_pos]][route[last_pos + 1]]);
                const double two_dis = instance->get_distance(first_node, station);

                const double dist = accumulated_distance[s.i] - accumulated_distance[last_pos + 1];
                if (dist + one_dis + two_dis > preprocessor->max_cruise_distance_) {
                    should_pop = true;
                }
            }

            if (!should_pop && s.n_len == 1) {
                const double one_dis = accumulated_distance[length - 1] - accumulated_distance[s.i + 1]
                                + instance->get_distance(station, route[s.i + 1]);
                if (one_dis > preprocessor->max_cruise_distance_) {
                    s.i++;
                    continue;
                }
            }

            if (!should_pop) {
                chosen_pos[cur_upper_bound - s.n_len] = s.i;
                stk.push({s.i + 1, s.n_len - 1, s.i + 1});
                s.i++;
                continue;
            }
        }

        stk.pop(); // only pop here
    }
}

namespace {

// Optimal charging-station placement for a fixed customer sequence (shortest path).
//
// The vehicle recharges fully at every station, so the route splits into independent pieces between
// charging points, and a placement is feasible iff every piece is within the cruise range. A label is
// "charged at station s, which is inserted on edge e = (route[e], route[e + 1])", with the shortest
// distance travelled from the start depot. Labels are processed edge by edge; inside one edge a vehicle
// may also drive from one station straight to another, so several consecutive stations between two
// customers are allowed. Any number of stations is allowed, and among placements of equal cost the one
// with fewer stations is kept. Time O(edges * (window * S^2 + S^2)) with S stations.
class ChargingPlacementDp {
public:
    struct Placement {
        double cost = std::numeric_limits<double>::max();  // max if there is no feasible placement
        std::vector<int> edge;     // edge index of each inserted station, in route order
        std::vector<int> station;  // inserted station ids, in route order
    };

    ChargingPlacementDp(Case& instance, const Preprocessor& preprocessor, const int* route,
                        const int length, const std::vector<double>& accumulated_distance)
        : route_(route),
          edges_(length - 1),
          stations_(instance.num_station_),
          range_(preprocessor.max_cruise_distance_),
          accumulated_(accumulated_distance),
          station_ids_(preprocessor.station_ids_),
          between_stations_(preprocessor.station_distances_) {
        // Route-to-station distances depend on the route and are charged to the budget (2 * edges * |S| reads).
        // Station-to-station distances are instance data, computed once by the preprocessor and not charged.
        to_station_.resize(static_cast<size_t>(edges_) * stations_);
        from_station_.resize(static_cast<size_t>(edges_) * stations_);
        for (int e = 0; e < edges_; ++e) {
            for (int s = 0; s < stations_; ++s) {
                to_station_[index(e, s)] = instance.get_distance(route_[e], station_ids_[s]);
                from_station_[index(e, s)] = instance.get_distance(station_ids_[s], route_[e + 1]);
            }
        }
    }

    Placement solve() const {
        Placement best;
        if (edges_ <= 0 || stations_ <= 0) return best;

        const size_t labels = static_cast<size_t>(edges_) * stations_;
        std::vector<double> distance(labels, kUnreached);
        std::vector<int> count(labels, 0);
        std::vector<int> parent(labels, -1);  // previous label, -1 = start depot
        const double total = accumulated_.back();
        int best_label = -1;
        int best_count = 0;

        for (int e = 0; e < edges_; ++e) {
            // Arrive at a station on edge e from the start depot or from a station on an earlier edge.
            for (int s = 0; s < stations_; ++s) {
                const size_t at = index(e, s);
                if (accumulated_[e] + to_station_[at] <= range_) {
                    relax(distance, count, parent, at, accumulated_[e] + to_station_[at], 1, -1);
                }
                for (int i = e - 1; i >= 0; --i) {
                    const double between = accumulated_[e] - accumulated_[i + 1];
                    if (between > range_) break;  // earlier edges are even further away
                    for (int q = 0; q < stations_; ++q) {
                        const size_t from = index(i, q);
                        if (distance[from] == kUnreached) continue;
                        const double piece = from_station_[from] + between + to_station_[at];
                        if (piece > range_) continue;
                        relax(distance, count, parent, at, distance[from] + piece, count[from] + 1,
                              static_cast<int>(from));
                    }
                }
            }
            // Station-to-station moves inside edge e (Dijkstra over the stations of this edge).
            std::vector<char> settled(stations_, 0);
            for (int round = 0; round < stations_; ++round) {
                int q = -1;
                for (int s = 0; s < stations_; ++s) {
                    const size_t at = index(e, s);
                    if (settled[s] || distance[at] == kUnreached) continue;
                    if (q < 0 || better(distance[at], count[at], distance[index(e, q)], count[index(e, q)])) q = s;
                }
                if (q < 0) break;
                settled[q] = 1;
                const size_t from = index(e, q);
                for (int s = 0; s < stations_; ++s) {
                    if (settled[s]) continue;
                    const double hop = between_stations_[static_cast<size_t>(q) * stations_ + s];
                    if (hop > range_) continue;
                    relax(distance, count, parent, index(e, s), distance[from] + hop, count[from] + 1,
                          static_cast<int>(from));
                }
            }
            // Finish at the end depot from a station on edge e.
            for (int s = 0; s < stations_; ++s) {
                const size_t at = index(e, s);
                if (distance[at] == kUnreached) continue;
                const double piece = from_station_[at] + total - accumulated_[e + 1];
                if (piece > range_) continue;
                const double cost = distance[at] + piece;
                if (best_label < 0 || better(cost, count[at], best.cost, best_count)) {
                    best.cost = cost;
                    best_count = count[at];
                    best_label = static_cast<int>(at);
                }
            }
        }
        if (best_label < 0) return best;

        for (int label = best_label; label >= 0; label = parent[label]) {
            best.edge.push_back(label / stations_);
            best.station.push_back(station_ids_[label % stations_]);
        }
        std::reverse(best.edge.begin(), best.edge.end());
        std::reverse(best.station.begin(), best.station.end());
        return best;
    }

private:
    static constexpr double kUnreached = std::numeric_limits<double>::infinity();
    static constexpr double kTieTolerance = 1e-9;

    const int* route_;
    int edges_;
    int stations_;
    double range_;
    const std::vector<double>& accumulated_;
    const std::vector<int>& station_ids_;
    std::vector<double> to_station_;
    std::vector<double> from_station_;
    const std::vector<double>& between_stations_;  // preprocessor.station_distances_

    size_t index(const int e, const int s) const { return static_cast<size_t>(e) * stations_ + s; }

    // Shorter distance first; on (near) equal distance, fewer stations.
    static bool better(const double distance, const int count, const double other_distance, const int other_count) {
        if (distance < other_distance - kTieTolerance) return true;
        if (distance > other_distance + kTieTolerance) return false;
        return count < other_count;
    }

    static void relax(std::vector<double>& distance, std::vector<int>& count, std::vector<int>& parent,
                      const size_t at, const double value, const int stations, const int from) {
        if (distance[at] == kUnreached || better(value, stations, distance[at], count[at])) {
            distance[at] = value;
            count[at] = stations;
            parent[at] = from;
        }
    }
};

} // namespace

double Follower::eval_optimal_cost(const int* route, const int length) const {
    if (length <= 2) return 0.0;
    std::vector<double> accumulated_distance(length, 0.0);
    for (int i = 1; i < length; ++i) {
        accumulated_distance[i] = accumulated_distance[i - 1] + instance->get_distance(route[i], route[i - 1]);
    }
    if (accumulated_distance.back() <= preprocessor->max_cruise_distance_) {
        return accumulated_distance.back();
    }
    const ChargingPlacementDp placement(*instance, *preprocessor, route, length, accumulated_distance);
    const ChargingPlacementDp::Placement best = placement.solve();
    return best.cost == std::numeric_limits<double>::max() ? -1.0 : best.cost;
}

double Follower::insert_station_by_all_enumeration(int* repaired_route, int& repaired_length) const {
    const int original_length = repaired_length;
    std::vector<int> route(repaired_route, repaired_route + original_length);
    std::vector<double> accumulated_distance(original_length, 0.0);
    for (int i = 1; i < original_length; ++i) {
        accumulated_distance[i] = accumulated_distance[i - 1] + instance->get_distance(route[i], route[i - 1]);
    }
    if (accumulated_distance.back() <= preprocessor->max_cruise_distance_) {
        return accumulated_distance.back();
    }

    // Optimal placement over any number of stations, including consecutive stations on one edge.
    const ChargingPlacementDp placement(*instance, *preprocessor, route.data(), original_length, accumulated_distance);
    const ChargingPlacementDp::Placement best = placement.solve();
    if (best.cost != std::numeric_limits<double>::max()
        && original_length + static_cast<int>(best.edge.size()) <= lower_node_cap) {
        repaired_length = 0;
        size_t next = 0;
        for (int i = 0; i < original_length; ++i) {
            repaired_route[repaired_length++] = route[i];
            while (next < best.edge.size() && best.edge[next] == i) {
                repaired_route[repaired_length++] = best.station[next++];
            }
        }
        double cost = 0.0;
        for (int i = 1; i < repaired_length; ++i) {
            cost += instance->get_distance(repaired_route[i - 1], repaired_route[i]);
        }
        return cost;
    }

    // No feasible placement exists; keep the standard follower path as a safety net.
    std::vector<int> standard_fallback_route(lower_node_cap, 0);
    std::memcpy(standard_fallback_route.data(), repaired_route, sizeof(int) * original_length);
    int standard_fallback_length = original_length;
    double standard_fallback_cost = insert_station_by_simple_enum(
        standard_fallback_route.data(), standard_fallback_length);
    if (standard_fallback_cost < 0) {
        std::memcpy(standard_fallback_route.data(), repaired_route, sizeof(int) * original_length);
        standard_fallback_length = original_length;
        standard_fallback_cost = insert_station_by_remove_enum(
            standard_fallback_route.data(), standard_fallback_length);
        if (standard_fallback_cost < 0) {
            return -1.0;
        }
    }

    std::memcpy(repaired_route, standard_fallback_route.data(), sizeof(int) * standard_fallback_length);
    repaired_length = standard_fallback_length;
    return standard_fallback_cost;
}

std::ostream& operator<<(std::ostream& os, const Follower& follower) {
    os << "Number of Routes: " << follower.num_routes << "\n";
    os << "Lower Cost: " << follower.lower_cost << "\n";

    os << "Number of Nodes per route (lower): ";
    for (int i = 0; i < follower.route_cap; ++i) {
        os << follower.lower_num_nodes_per_route[i] << " ";
    }
    os << "\n";

    os << "Lower Routes: \n";
    for (int i = 0; i < follower.num_routes; ++i) {
        os << "Route " << i << ": ";
        for (int j = 0; j < follower.node_cap; ++j) {
            os << follower.lower_routes[i][j] << " ";
        }
        os << "\n";
    }
    os << "\n";

    return os;
}
