//
// Created by Yinghao Qin on 09/06/2025.
//

#include "follower.hpp"
#include <list>
#include <stack>
#include <cassert>

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
    memset(this->lower_cost_per_route, 0.0, sizeof(double) * route_cap);
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

void Follower::clean() {
    this->num_routes = 0;
    this->lower_cost = 0.;
    for (int i = 0; i < route_cap; ++i) {
        memset(this->lower_routes[i], 0, sizeof(int) * node_cap);
    }
    memset(this->lower_num_nodes_per_route, 0, sizeof(int) * route_cap);
    memset(this->lower_cost_per_route, 0.0, sizeof(double) * route_cap);
}

void Follower::run_for_single_route(const int route_idx) const {
    if (lower_num_nodes_per_route[route_idx] <= 2) {
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
        lower_cost_per_route[i] = cost;
        lower_cost += cost;
    }

    export_individual(ind);
}

void Follower::run(Individual *ind) {
    load_individual(ind);

    for (int i = 0; i < num_routes; ++i) {
        run_for_single_route(i);
    }
    lower_cost = std::accumulate(lower_cost_per_route, lower_cost_per_route + num_routes, 0.0);

    export_individual(ind);
}

void Follower::load_individual(const Individual* ind) {
    clean();

    this->num_routes = ind->num_routes;
    for (int i = 0; i < num_routes; ++i) {
        this->lower_num_nodes_per_route[i] = ind->num_nodes_per_route[i];

        memcpy(this->lower_routes[i], ind->routes[i], sizeof(int) * node_cap);
    }
}

void Follower::export_individual(Individual* ind) const {
    ind->lower_cost = this->lower_cost;
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
            const int station = preprocessor->best_station_[first_node][second_node];

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

double Follower::insert_station_by_all_enumeration(int* repaired_route, int& repaired_length) const {
    const int length = repaired_length;
    const auto route = new int[length];
    memcpy(route, repaired_route, sizeof(int) * length);

    vector<double> accumulated_distance(length, 0);
    for (int i = 1; i < length; i++) {
        accumulated_distance[i] = accumulated_distance[i - 1] + instance->get_distance(route[i], route[i - 1]);
    }
    if (accumulated_distance.back() <= preprocessor->max_cruise_distance_) {
        delete[] route;
        return accumulated_distance.back();
    }

    const int upper_bound = ceil(accumulated_distance.back() / preprocessor->max_cruise_distance_);
    const int lower_bound = floor(accumulated_distance.back() / preprocessor->max_cruise_distance_);
    const auto chosen_pos = new int[length];
    const auto chosen_sta = new int[length];
    double cost = numeric_limits<double>::max();
    ChargingMeta meta;
    meta.cost = numeric_limits<double>::max();
    for (int i = lower_bound; i <= upper_bound; i++) {
        ChargingMeta iter_meta = try_enumerate_n_stations_to_route(0, i, chosen_sta, chosen_pos,cost, i, route, length, accumulated_distance);
        if (cost != numeric_limits<double>::max() && cost < meta.cost) {
            meta = iter_meta;
        }
    }
    delete[] chosen_pos;
    delete[] chosen_sta;
    delete[] route;

    if (cost != numeric_limits<double>::max()) {
        for (int k = meta.num_stations - 1; k >= 0; k--) {
            const int insertPos = meta.chosen_pos[k] + 1;
            for (int i = repaired_length; i > insertPos; i--) {
                repaired_route[i] = repaired_route[i - 1];
            }
            repaired_route[insertPos] = meta.chosen_sta[k];
            repaired_length++;
        }

        return cost;
    }
    return -1;
}

ChargingMeta Follower::try_enumerate_n_stations_to_route(const int m_len, const int n_len, int *chosen_sta,
    int *chosen_pos, double &cost, const int cur_upper_bound, const int *route, const int length,
    const vector<double> &accumulated_distance) const {

    ChargingMeta meta;

    // This structure is used in the "enumerate stations"
    struct State {
        int m_len{}, n_len{}, i{}, stationIdx{}; // Current state variables
    };

    stack<State> stk;

    // Push the initial state
    stk.push({m_len, n_len, m_len, 0});

    while (!stk.empty()) {
        auto& s = stk.top(); // Get the current state

        // If n_len == 0, evaluate the solution
        if (s.n_len == 0) {
            stk.pop(); // Backtrack
            bool feasible = true;
            double piece_distance = accumulated_distance[chosen_pos[0]] + instance->get_distance(route[chosen_pos[0]], chosen_sta[0]);
            if (piece_distance > preprocessor->max_cruise_distance_) feasible = false;

            for (int k = 1; feasible && k < cur_upper_bound; k++) {
                piece_distance = accumulated_distance[chosen_pos[k]] - accumulated_distance[chosen_pos[k - 1] + 1];
                piece_distance += instance->get_distance(chosen_sta[k - 1], route[chosen_pos[k - 1] + 1]);
                piece_distance += instance->get_distance(chosen_sta[k], route[chosen_pos[k]]);
                if (piece_distance > preprocessor->max_cruise_distance_) feasible = false;
            }

            piece_distance = accumulated_distance.back() - accumulated_distance[chosen_pos[cur_upper_bound - 1] + 1];
            piece_distance += instance->get_distance(route[chosen_pos[cur_upper_bound - 1] + 1], chosen_sta[cur_upper_bound - 1]);
            if (piece_distance > preprocessor->max_cruise_distance_) feasible = false;

            if (feasible) {
                double total_distance = accumulated_distance.back();
                for (int k = 0; k < cur_upper_bound; k++) {
                    const int first_node = route[chosen_pos[k]];
                    const int second_node = route[chosen_pos[k] + 1];
                    total_distance -= instance->get_distance(first_node, second_node);
                    total_distance += instance->get_distance(first_node, chosen_sta[k]);
                    total_distance += instance->get_distance(chosen_sta[k], second_node);
                }
                // produce the repaired route
                if (total_distance < cost) {
                    cost = total_distance;
                    meta.cost = cost;
                    meta.num_stations = cur_upper_bound;
                    meta.chosen_pos.assign(chosen_pos, chosen_pos + meta.num_stations);
                    meta.chosen_sta.assign(chosen_sta, chosen_sta + meta.num_stations);
                }
            }
            continue;
        }

        // Iterate through route positions
        if (s.i <= length - 1 - s.n_len) {
            if (cur_upper_bound == s.n_len) {
                if (accumulated_distance[s.i] >= preprocessor->max_cruise_distance_) {
                    s.i++;
                    continue;
                }
            } else {
                if (accumulated_distance[s.i] - accumulated_distance[chosen_pos[cur_upper_bound - s.n_len - 1] + 1] >= preprocessor->max_cruise_distance_) {
                    s.i++;
                    continue;
                }
            }

            if (s.n_len == 1) {
                if (accumulated_distance.back() - accumulated_distance[s.i + 1] >= preprocessor->max_cruise_distance_) {
                    s.i++;
                    continue;
                }
            }

            // Iterate through stations
            while (s.stationIdx < instance->num_station_) {
                chosen_sta[cur_upper_bound - s.n_len] = preprocessor->station_ids_[s.stationIdx];
                chosen_pos[cur_upper_bound - s.n_len] = s.i;

                // Push the next state for further exploration
                stk.push({s.i + 1, s.n_len - 1, s.i + 1, 0});
                s.stationIdx++;
                break; // Go deeper
            }

            // If all stations are processed for the current position
            if (s.stationIdx == instance->num_station_) {
                s.stationIdx = 0; // Reset for the next iteration
                s.i++;            // Move to the next route position
            }
        } else {
            stk.pop(); // Backtrack if no more positions are left
        }
    }

    return std::move(meta);
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
