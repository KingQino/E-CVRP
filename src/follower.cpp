//
// Created by Yinghao Qin on 09/06/2025.
//

#include "follower.hpp"
#include <algorithm>
#include <list>
#include <stack>
#include <cassert>
#include <limits>
#include <vector>

using namespace std;

Follower::Follower(Case* instance, Preprocessor* preprocessor) {
    this->instance = instance;
    this->preprocessor = preprocessor;

    route_cap = preprocessor->route_cap_;
    node_cap = preprocessor->node_cap_;

    prepare_temp_buffers(node_cap);

    this->num_routes = 0;
    this->lower_routes = new int *[route_cap];
    for (int i = 0; i < route_cap; ++i) {
        this->lower_routes[i] = new int [node_cap];
        memset(this->lower_routes[i], 0, sizeof(int) * node_cap);
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
                memcpy(lower_routes[idx2], lower_routes[last], sizeof(int) * node_cap);
                lower_num_nodes_per_route[idx2] = lower_num_nodes_per_route[last];
                lower_cost_per_route[idx2] = lower_cost_per_route[last];
            }
            memset(lower_routes[last], 0, sizeof(int) * node_cap);
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
                memcpy(lower_routes[idx1], lower_routes[last], sizeof(int) * node_cap);
                lower_num_nodes_per_route[idx1] = lower_num_nodes_per_route[last];
                lower_cost_per_route[idx1] = lower_cost_per_route[last];
            }
            memset(lower_routes[last], 0, sizeof(int) * node_cap);
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

        const double cost_SE = eval_simple_enum_cost(route, length);
        if (cost_SE < 0) {
            const double cost_RE = eval_remove_enum_cost(route, length);
            lower_cost_per_route[i] = (cost_RE < 0) ? INFEASIBLE : cost_RE;
        } else {
            lower_cost_per_route[i] = cost_SE;
        }
    }

    lower_cost = std::accumulate(lower_cost_per_route,
                                 lower_cost_per_route + num_routes,
                                 0.0);

    export_individual(ind);
}

void Follower::load_individual(const Individual* ind) {
    this->num_routes = ind->num_routes;
    for (int i = 0; i < num_routes; ++i) {
        this->lower_num_nodes_per_route[i] = ind->num_nodes_per_route[i];

        // An individual may use a smaller node capacity than the follower (e.g. MA's population);
        // copy its whole row and clear the remainder.
        const int copied = std::min(node_cap, ind->node_cap);
        memcpy(this->lower_routes[i], ind->routes[i], sizeof(int) * copied);
        if (copied < node_cap) {
            memset(this->lower_routes[i] + copied, 0, sizeof(int) * (node_cap - copied));
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

    const int lower_bound = static_cast<int>(total_distance / preprocessor->max_cruise_distance_);
    const int upper_bound = static_cast<int>(total_distance / preprocessor->max_cruise_distance_ + 1);

    double final_cost = std::numeric_limits<double>::max();

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

        if (final_cost != std::numeric_limits<double>::max()) {
            break;
        }
    }

    return (final_cost != std::numeric_limits<double>::max()) ? final_cost : -1.0;
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

        const int station = preprocessor->get_best_and_feasible_station(
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
    const int lower_bound = static_cast<int>(total_distance / preprocessor->max_cruise_distance_);
    const int upper_bound = static_cast<int>(total_distance / preprocessor->max_cruise_distance_ + 1);

    double final_cost = numeric_limits<double>::max();

    for (int i = lower_bound; i <= upper_bound; ++i) {
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

        if (final_cost != numeric_limits<double>::max()) {
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

            break;
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

        const int station = preprocessor->get_best_and_feasible_station(temp_route[i], temp_route[i + 1], allowed_dis);
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

                // side nodes to the best station has been cached
                dis_sum -= instance->get_distance(first_node, second_node);
                dis_sum += instance->distances_[first_node][station];
                dis_sum += instance->distances_[station][second_node];
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
                const double one_dis = instance->distances_[first_node][station];
                if (accumulated_distance[s.i] + one_dis > preprocessor->max_cruise_distance_) {
                    should_pop = true;
                }
            } else {
                const int last_pos = chosen_pos[cur_upper_bound - s.n_len - 1];
                const double one_dis = instance->get_distance(route[last_pos + 1], preprocessor->best_station_[route[last_pos]][route[last_pos + 1]]);
                const double two_dis = instance->distances_[first_node][station];

                const double dist = accumulated_distance[s.i] - accumulated_distance[last_pos + 1];
                if (dist + one_dis + two_dis > preprocessor->max_cruise_distance_) {
                    should_pop = true;
                }
            }

            if (!should_pop && s.n_len == 1) {
                const double one_dis = accumulated_distance[length - 1] - accumulated_distance[s.i + 1]
                                + instance->distances_[station][route[s.i + 1]];
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

// Optimal charging-station placement for a fixed route with exactly k stations, by dynamic programming.
//
// The vehicle recharges fully at every station, so the pieces between two consecutive stations are
// independent: a placement is feasible iff every piece is within the cruise range, and its cost is the
// route length plus the sum of the station detours. The DP state is "k stations placed, the last one is
// station s on edge e = (route[e], route[e + 1])". It applies exactly the same pruning and feasibility
// rules as the former exhaustive enumeration (at most one station per edge, edges in route order), so it
// finds the same best cost for every k in O(k * edges * window * stations^2) time instead of
// O(C(edges, k) * stations^k). Layers are built on demand, so only the station counts that are tried
// are computed.
class ChargingPlacementDp {
public:
    ChargingPlacementDp(Case& instance, const Preprocessor& preprocessor, const int* route,
                        const int length, const std::vector<double>& accumulated_distance)
        : instance_(instance),
          route_(route),
          edges_(length - 1),
          stations_(instance.num_station_),
          range_(preprocessor.max_cruise_distance_),
          accumulated_(accumulated_distance),
          station_ids_(preprocessor.station_ids_) {
        to_station_.resize(static_cast<size_t>(edges_) * stations_);
        from_station_.resize(static_cast<size_t>(edges_) * stations_);
        detour_.resize(static_cast<size_t>(edges_) * stations_);
        for (int e = 0; e < edges_; ++e) {
            const double edge_length = accumulated_[e + 1] - accumulated_[e];
            for (int s = 0; s < stations_; ++s) {
                const size_t idx = index(e, s);
                to_station_[idx] = instance.get_distance(route_[e], station_ids_[s]);
                from_station_[idx] = instance.get_distance(station_ids_[s], route_[e + 1]);
                detour_[idx] = to_station_[idx] + from_station_[idx] - edge_length;
            }
        }
    }

    // Best placement with exactly k stations; meta.cost == DBL max if there is none.
    ChargingMeta best_with(const int k) {
        ChargingMeta meta;
        meta.cost = std::numeric_limits<double>::max();
        if (k <= 0 || k > edges_ || stations_ <= 0) return meta;
        extend_to(k);

        const std::vector<double>& layer = value_[k - 1];
        const double total = accumulated_.back();
        int best_idx = -1;
        double best_value = std::numeric_limits<double>::max();
        for (int e = 0; e < edges_; ++e) {
            // the last piece runs from the station on edge e to the end depot
            if (total - accumulated_[e + 1] >= range_) continue;
            for (int s = 0; s < stations_; ++s) {
                const size_t idx = index(e, s);
                if (layer[idx] == kInfeasible) continue;
                if (total - accumulated_[e + 1] + from_station_[idx] > range_) continue;
                if (layer[idx] < best_value) {
                    best_value = layer[idx];
                    best_idx = static_cast<int>(idx);
                }
            }
        }
        if (best_idx < 0) return meta;

        meta.num_stations = k;
        meta.chosen_pos.assign(k, 0);
        meta.chosen_sta.assign(k, 0);
        int idx = best_idx;
        for (int layer_k = k; layer_k >= 1; --layer_k) {
            meta.chosen_pos[layer_k - 1] = idx / stations_;
            meta.chosen_sta[layer_k - 1] = station_ids_[idx % stations_];
            idx = parent_[layer_k - 1][idx];
        }
        // Same formula and summation order as the former enumeration, so equal-cost ties resolve the same way.
        double cost = total;
        for (int j = 0; j < k; ++j) {
            const int first_node = route_[meta.chosen_pos[j]];
            const int second_node = route_[meta.chosen_pos[j] + 1];
            cost -= instance_.get_distance(first_node, second_node);
            cost += instance_.get_distance(first_node, meta.chosen_sta[j]);
            cost += instance_.get_distance(meta.chosen_sta[j], second_node);
        }
        meta.cost = cost;
        return meta;
    }

private:
    static constexpr double kInfeasible = std::numeric_limits<double>::infinity();

    Case& instance_;
    const int* route_;
    int edges_;
    int stations_;
    double range_;
    const std::vector<double>& accumulated_;
    const std::vector<int>& station_ids_;
    std::vector<double> to_station_;
    std::vector<double> from_station_;
    std::vector<double> detour_;
    std::vector<std::vector<double>> value_;  // value_[k-1][e*S+s]: min total detour, k stations, last one (e, s)
    std::vector<std::vector<int>> parent_;    // parent_[k-1][e*S+s]: state of station k-1 (-1 for k = 1)

    size_t index(const int e, const int s) const { return static_cast<size_t>(e) * stations_ + s; }

    void extend_to(const int k) {
        while (static_cast<int>(value_.size()) < k) {
            const int layer_k = static_cast<int>(value_.size()) + 1;
            std::vector<double> layer(static_cast<size_t>(edges_) * stations_, kInfeasible);
            std::vector<int> parent(layer.size(), -1);
            if (layer_k == 1) {
                for (int e = 0; e < edges_; ++e) {
                    if (accumulated_[e] >= range_) break;  // the first piece already exceeds the range
                    for (int s = 0; s < stations_; ++s) {
                        const size_t idx = index(e, s);
                        if (accumulated_[e] + to_station_[idx] <= range_) layer[idx] = detour_[idx];
                    }
                }
            } else {
                const std::vector<double>& previous = value_[layer_k - 2];
                for (int j = layer_k - 1; j < edges_; ++j) {
                    for (int t = 0; t < stations_; ++t) {
                        const size_t to = index(j, t);
                        double best = kInfeasible;
                        int best_parent = -1;
                        for (int i = j - 1; i >= layer_k - 2; --i) {
                            const double between = accumulated_[j] - accumulated_[i + 1];
                            if (between >= range_) break;  // earlier edges are even further away
                            const double base = between + to_station_[to];
                            if (base > range_) continue;
                            for (int s = 0; s < stations_; ++s) {
                                const size_t from = index(i, s);
                                if (previous[from] == kInfeasible) continue;
                                if (base + from_station_[from] > range_) continue;
                                const double candidate = previous[from] + detour_[to];
                                if (candidate < best) {
                                    best = candidate;
                                    best_parent = static_cast<int>(from);
                                }
                            }
                        }
                        layer[to] = best;
                        parent[to] = best_parent;
                    }
                }
            }
            value_.push_back(std::move(layer));
            parent_.push_back(std::move(parent));
        }
    }
};

} // namespace

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

    const int lower_bound = static_cast<int>(std::floor(accumulated_distance.back() / preprocessor->max_cruise_distance_));
    const int upper_bound = static_cast<int>(std::ceil(accumulated_distance.back() / preprocessor->max_cruise_distance_));

    ChargingPlacementDp placement(*instance, *preprocessor, route.data(), original_length, accumulated_distance);
    ChargingMeta best_meta;
    best_meta.cost = std::numeric_limits<double>::max();

    const auto try_station_count = [&](const int station_count) {
        ChargingMeta iter_meta = placement.best_with(station_count);
        const bool feasible = iter_meta.cost != std::numeric_limits<double>::max();
        if (feasible && iter_meta.cost < best_meta.cost) {
            best_meta = std::move(iter_meta);
        }
        return feasible;
    };

    const auto materialize_best_meta = [&]() {
        std::memcpy(repaired_route, route.data(), sizeof(int) * original_length);
        repaired_length = original_length;
        for (int k = best_meta.num_stations - 1; k >= 0; --k) {
            const int insert_pos = best_meta.chosen_pos[k] + 1;
            for (int i = repaired_length; i > insert_pos; --i) {
                repaired_route[i] = repaired_route[i - 1];
            }
            repaired_route[insert_pos] = best_meta.chosen_sta[k];
            ++repaired_length;
        }

        return best_meta.cost;
    };

    // Preserve the original exact refinement range first to avoid perturbing instances that already work well.
    for (int k = lower_bound; k <= upper_bound; ++k) {
        try_station_count(k);
    }
    if (best_meta.cost != std::numeric_limits<double>::max()) {
        return materialize_best_meta();
    }

    // Only if the original range fails, use a feasible remove-enum route to extend the station-count search safely.
    std::vector<int> feasible_bound_route(node_cap, 0);
    std::memcpy(feasible_bound_route.data(), repaired_route, sizeof(int) * original_length);
    int feasible_bound_length = original_length;
    const double feasible_bound_cost = insert_station_by_remove_enum(
        feasible_bound_route.data(), feasible_bound_length);

    if (feasible_bound_cost >= 0) {
        const int feasible_upper_bound = feasible_bound_length - original_length;
        const int expanded_lower_bound = upper_bound + 1;
        int first_feasible_k = -1;

        for (int k = expanded_lower_bound; k <= feasible_upper_bound; ++k) {
            if (try_station_count(k)) {
                first_feasible_k = k;
                break;
            }
        }

        if (first_feasible_k != -1) {
            for (int k = first_feasible_k + 1; k <= feasible_upper_bound; ++k) {
                try_station_count(k);
            }
            return materialize_best_meta();
        }
    }

    // Fall back to the standard follower path as a safety net when exact refinement still fails.
    std::vector<int> standard_fallback_route(node_cap, 0);
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
