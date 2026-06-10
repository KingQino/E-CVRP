//
// Created by Yinghao Qin on 09/06/2025.
//

#include "leader_single.hpp"
#include <chrono>
#include <iomanip>

using namespace std;

LeaderSingle::LeaderSingle(std::mt19937& engine, Case *instance, Preprocessor *preprocessor)
    : Leader(engine, instance, preprocessor),
      op_selector(0, kNumOps - 1) {

    this->max_attempts = preprocessor->params.max_attempts;
    this->enable_op_profile = preprocessor->params.enable_logging;

    ops = {
        &LeaderSingle::op_relocate_intra,
        &LeaderSingle::op_relocate_inter,
        &LeaderSingle::op_swap_intra,
        &LeaderSingle::op_swap_inter,
        &LeaderSingle::op_2opt_intra,
        &LeaderSingle::op_2opt_inter_h2h,
        &LeaderSingle::op_2opt_inter_h2t,
        &LeaderSingle::op_relocate_inter_empty
    };
}

// random walk
bool LeaderSingle::neighbourhood_explore(Individual* ind, const double &history_val) {
    load_individual(ind);
    history_cost = history_val;

    const int op = op_selector(random_engine);
    bool has_moved;
    if (enable_op_profile) {
        const auto t0 = std::chrono::high_resolution_clock::now();
        has_moved = (this->*ops[op])();
        op_time_sec[op] += std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();
        ++op_call_counts[op];
        if (has_moved) ++op_accept_counts[op];
    } else {
        has_moved = (this->*ops[op])();
    }

    if (has_moved) export_individual(ind);
    return has_moved;
}

void LeaderSingle::write_operator_profile(std::ostream& os) const {
    os << "leader_single_operator_profile\n";
    os << std::left
       << std::setw(24) << "operator"
       << std::right
       << std::setw(10) << "calls"
       << std::setw(10) << "accepts"
       << std::setw(16) << "accept_rate(%)"
       << std::setw(14) << "time_sec"
       << std::setw(16) << "avg_us/call"
       << "\n";
    os << std::string(90, '-') << "\n";

    for (int op = 0; op < kNumOps; ++op) {
        const long calls = op_call_counts[op];
        const long accepts = op_accept_counts[op];
        const double accept_rate_pct = calls > 0 ? (100.0 * static_cast<double>(accepts) / static_cast<double>(calls)) : 0.0;
        const double avg_us_per_call = calls > 0 ? (1e6 * op_time_sec[op] / static_cast<double>(calls)) : 0.0;

        os << std::left << std::setw(24) << kOpLabels[op]
           << std::right
           << std::setw(10) << calls
           << std::setw(10) << accepts
           << std::setw(16) << std::fixed << std::setprecision(3) << accept_rate_pct
           << std::setw(14) << std::fixed << std::setprecision(6) << op_time_sec[op]
           << std::setw(16) << std::fixed << std::setprecision(3) << avg_us_per_call
           << "\n";
    }
}

bool LeaderSingle::op_relocate_intra() {
    return perform_intra_move(&LeaderSingle::node_relocate_intra); // M1
}
bool LeaderSingle::op_relocate_inter() {
    return perform_inter_move(&LeaderSingle::node_relocate_inter); // M2
}
bool LeaderSingle::op_swap_intra() {
    return perform_intra_move(&LeaderSingle::node_swap_intra);     // M3
}
bool LeaderSingle::op_swap_inter() {
    return perform_inter_move(&LeaderSingle::node_swap_inter);     // M4
}
bool LeaderSingle::op_2opt_intra() {
    return perform_intra_move(&LeaderSingle::two_opt_intra);       // M5
}
bool LeaderSingle::op_2opt_inter_h2h() {
    return perform_inter_move(&LeaderSingle::two_opt_inter_head_to_head);  // M6
}
bool LeaderSingle::op_2opt_inter_h2t() {
    return perform_inter_move(&LeaderSingle::two_opt_inter_head_to_tail);  // M7
}
bool LeaderSingle::op_relocate_inter_empty() {
    return perform_inter_move_with_empty_route(&LeaderSingle::node_relocate_inter_with_empty_route); // M8
}

bool LeaderSingle::perform_intra_move(const IntraMemFn fn) {
    if (num_routes < 1) return false;

    bool is_moved = false;
    int search_depth = 0;

    dist_route.param(Param(0, num_routes - 1));
    while (!is_moved && search_depth < max_attempts) {
        const int r = dist_route(random_engine);

        is_moved = (this->*fn)(routes[r], num_nodes_per_route[r], AcceptRule::Late);
        search_depth++;
    }

    return is_moved;
}

bool LeaderSingle::perform_inter_move(const InterMemFn fn) {
    if (num_routes <= 1) return false;

    bool is_moved = false;
    int search_depth = 0;

    auto set_route_dist_param = [&](){
        dist_route.param(Param(0, num_routes - 1));
    };
    set_route_dist_param();

    while (!is_moved && search_depth < max_attempts) {
        // randomly generate two different route indices
        const int r1 = dist_route(random_engine);
        int r2;
        do {
            r2 = dist_route(random_engine);
        } while (r1 == r2);

        is_moved = (this->*fn)(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],
                            demand_sum_per_route[r1], demand_sum_per_route[r2], AcceptRule::Late);

        if (is_moved) {
            clean_empty_routes(r1, r2);
            set_route_dist_param(); // reset the route distribution range after possible route removal
        }

        search_depth++;
    }

    return is_moved;
}

bool LeaderSingle::perform_inter_move_with_empty_route(const InterMemFn fn) {
    if (num_routes < 1) return false;

    bool is_moved = false;
    int search_depth = 0;

    while (!is_moved && search_depth < max_attempts) {
        dist_route.param(Param(0, num_routes - 1));
        const int r1 = dist_route(random_engine);
        const int r2 = num_routes; // empty route

        is_moved = (this->*fn)(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],
            demand_sum_per_route[r1], demand_sum_per_route[r2], AcceptRule::Late);

        if (is_moved) num_routes++;

        search_depth++;
    }

    return is_moved;
}

void LeaderSingle::clean_empty_routes(const int r1, const int r2) {
    auto remove_if_empty = [&](const int route) {
        if (demand_sum_per_route[route] == 0 && num_routes > 0) {
            const int last = num_routes - 1;
            std::swap(routes[route], routes[last]);
            std::swap(demand_sum_per_route[route], demand_sum_per_route[last]);
            std::swap(num_nodes_per_route[route], num_nodes_per_route[last]);
            memset(routes[last], 0, sizeof(int) * node_cap);
            num_nodes_per_route[last] = 0;
            num_routes--;
        }
    };

    if (demand_sum_per_route[r1] == 0 || demand_sum_per_route[r2] == 0) {
        const int empty_r = demand_sum_per_route[r1] == 0 ? r1 : r2;
        remove_if_empty(empty_r);
    }

    // Clear unused routes beyond num_routes
    std::fill(num_nodes_per_route + num_routes, num_nodes_per_route + route_cap, 0);
    std::fill(demand_sum_per_route + num_routes, demand_sum_per_route + route_cap, 0);
}

bool LeaderSingle::node_relocate_intra(int *route, int length, AcceptRule rule) {
    if (length <= 4) return false;

    bool is_accepted = false;

    dist_i.param(Param(1, length - 2));
    const int i = dist_i(random_engine);

    double original_cost, modified_cost;
    for (int j = 1; j < length - 1; j++) {
        if (i == j) continue;

        if (i < j) {
            original_cost = instance->get_distance(route[i - 1], route[i]) +
                            instance->get_distance(route[i], route[i + 1]) +
                            instance->get_distance(route[j], route[j + 1]);
            modified_cost = instance->get_distance(route[i - 1], route[i + 1]) +
                            instance->get_distance(route[j], route[i]) +
                            instance->get_distance(route[i], route[j + 1]);
        } else {
            original_cost = instance->get_distance(route[i - 1], route[i]) +
                            instance->get_distance(route[i], route[i + 1]) +
                            instance->get_distance(route[j - 1], route[j]);
            modified_cost = instance->get_distance(route[j - 1], route[i]) +
                            instance->get_distance(route[i], route[j]) +
                            instance->get_distance(route[i - 1], route[i + 1]);
        }

        const double change = modified_cost - original_cost;
        if (accept(rule, change)) {
            relocate_pos(route, i, j);
            upper_cost += change;

            is_accepted = true;
            break;
        }
    }

    return is_accepted;
}

bool LeaderSingle::node_relocate_inter(int *route1, int *route2, int &length1, int &length2, int &loading1,
    int &loading2, AcceptRule rule) {
    if (length1 < 3 || length2 < 3) return false;

    bool is_accepted = false;

    // count all possible r1 indices
    int count_possible_r1_idx = 0;
    for (int i = 1; i < length1 - 1; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) <= instance->max_vehicle_capa_) {
            ++count_possible_r1_idx;
        }
    }
    if (count_possible_r1_idx == 0) return false; // no possible r1 index, then return false

    dist_i.param(Param(0, count_possible_r1_idx - 1));
    int pick = dist_i(random_engine);

    // select the r1 index based on the random pick
    int i_selected = -1;
    for (int i = 1; i < length1 - 1; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) <= instance->max_vehicle_capa_) {
            if (pick-- == 0) { i_selected = i; break; }
        }
    }
    const int i = i_selected;

    for (int j = 0; j < length2 - 1; ++j) {
        const double original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                                     instance->get_distance(route1[i], route1[i + 1]) +
                                     instance->get_distance(route2[j], route2[j + 1]);
        const double modified_cost = instance->get_distance(route1[i - 1], route1[i + 1]) +
                                     instance->get_distance(route2[j], route1[i]) +
                                     instance->get_distance(route1[i], route2[j + 1]);

        const double change = modified_cost - original_cost;

        if (accept(rule, change)) {
            const int x = route1[i];
            for (int p = i; p < length1 - 1; p++) {
                route1[p] = route1[p + 1];
            }
            length1--;
            loading1 -= instance->get_customer_demand_(x);
            for (int q = length2; q > j + 1; q--) {
                route2[q] = route2[q - 1];
            }
            route2[j + 1] = x;
            length2++;
            loading2 += instance->get_customer_demand_(x);
            upper_cost += change;

            is_accepted = true;
            break;
        }
    }

    return is_accepted;
}

bool LeaderSingle::node_swap_intra(int *route, int length, AcceptRule rule) {
    if (length < 6) return false;

    bool is_accepted = false;

    dist_i.param(Param(1, length - 4));
    const int i = dist_i(random_engine);
    for (int j = i + 2; j < length - 1; ++j) {
        const double original_cost = instance->get_distance(route[i - 1], route[i]) +
                                     instance->get_distance(route[i], route[i + 1]) +
                                     instance->get_distance(route[j - 1], route[j]) +
                                     instance->get_distance(route[j], route[j + 1]);
        const double modified_cost = instance->get_distance(route[i - 1], route[j]) +
                                     instance->get_distance(route[j], route[i + 1]) +
                                     instance->get_distance(route[j - 1], route[i]) +
                                     instance->get_distance(route[i], route[j + 1]);

        const double change = modified_cost - original_cost;
        if (accept(rule, change)) {
            swap(route[i], route[j]);
            upper_cost += change;

            is_accepted = true;
            break;
        }

    }

    return is_accepted;
}

bool LeaderSingle::node_swap_inter(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2,
    AcceptRule rule) {
    if (length1 < 3 || length2 < 3) return false;

    bool is_accepted = false;

    dist_i.param(Param(1, length1 - 2));
    const int i = dist_i(random_engine);
    for (int j = 1; j < length2 - 1; ++j) {
        const int demand_I = instance->get_customer_demand_(route1[i]);
        const int demand_J = instance->get_customer_demand_(route2[j]);
        if (loading1 - demand_I + demand_J <= instance->max_vehicle_capa_ && loading2 - demand_J + demand_I <= instance->max_vehicle_capa_) {
            const double original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                                         instance->get_distance(route1[i], route1[i + 1]) +
                                         instance->get_distance(route2[j - 1], route2[j]) +
                                         instance->get_distance(route2[j], route2[j + 1]);
            const double modified_cost = instance->get_distance(route1[i - 1], route2[j]) +
                                         instance->get_distance(route2[j], route1[i + 1]) +
                                         instance->get_distance(route2[j - 1], route1[i]) +
                                         instance->get_distance(route1[i], route2[j + 1]);

            const double change = modified_cost - original_cost;
            if (accept(rule, change)) {
                swap(route1[i], route2[j]);
                loading1 = loading1 - demand_I + demand_J;
                loading2 = loading2 - demand_J + demand_I;
                upper_cost += change;

                is_accepted = true;
                break;
            }

        }
    }

    return is_accepted;
}

bool LeaderSingle::two_opt_intra(int *route, int length, AcceptRule rule) {
    if (length < 5) return false;

    bool is_accepted = false;

    dist_i.param(Param(1, length - 3));
    const int i = dist_i(random_engine);

    for (int j = i + 1; j < length - 1; ++j) {
        // Calculate the cost difference between the old route and the new route obtained by swapping arcs
        const double original_cost = instance->get_distance(route[i - 1], route[i]) +
                                     instance->get_distance(route[j], route[j + 1]);
        const double modified_cost = instance->get_distance(route[i - 1], route[j]) +
                                     instance->get_distance(route[i], route[j + 1]);

        const double change = modified_cost - original_cost;
        if (accept(rule, change)) {
            // update current solution
            reverse(route + i, route + j + 1);
            upper_cost += change;

            is_accepted = true;
            break;
        }
    }

    return is_accepted;
}

bool LeaderSingle::two_opt_inter_head_to_head(int *route1, int *route2, int &length1, int &length2, int &loading1,
    int &loading2, AcceptRule rule) {
    if (length1 < 3 || length2 < 3) return false;

    bool has_moved = false;

    dist_i.param(Param(0, length1 - 2));
    const int n1 = dist_i(random_engine);
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int i = 0; i <= n1; i++) {
        partial_dem_r1 += instance->get_customer_demand_(route1[i]);
    }

    int partial_dem_r2 = 0;
    for (int n2 = 0; n2 < length2 - 1; ++n2) {
        if ((n1 == length1 - 2 && n2 == 0) || (n1 == 0 && n2 == length2 - 2)) continue; // the same as the current situation, just skip it.
        partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

        if (partial_dem_r1 + partial_dem_r2 > instance->max_vehicle_capa_ || loading1 - partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_) continue;

        const double original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) +
                                     instance->get_distance(route2[n2], route2[n2 + 1]);
        const double modified_cost = instance->get_distance(route1[n1], route2[n2]) +
                                     instance->get_distance(route1[n1 + 1], route2[n2 + 1]);

        const double change = modified_cost - original_cost;

        if (accept(rule, change)) {
            // update
            upper_cost += change;
            memcpy(temp_r1, route1, sizeof(int) * node_cap);
            int counter1 = n1 + 1;
            for (int i = n2; i >= 0; i--) {
                route1[counter1++] = route2[i];
            }
            int counter2 = 0;
            for (int i = length1 - 1; i >= n1 + 1; i--) {
                temp_r2[counter2++] = temp_r1[i];
            }
            for (int i = n2 + 1; i < length2; i++) {
                temp_r2[counter2++] = route2[i];
            }
            memcpy(route2, temp_r2, sizeof(int) * node_cap);
            length1 = counter1;
            length2 = counter2;
            const int new_dem_sum_1 = partial_dem_r1 + partial_dem_r2;
            const int new_dem_sum_2 = loading1 - partial_dem_r1 + loading2 - partial_dem_r2;
            loading1 = new_dem_sum_1;
            loading2 = new_dem_sum_2;

            std::fill(route1 + length1, route1 + node_cap, 0);
            std::fill(route2 + length2, route2 + node_cap, 0);

            has_moved = true;
            break;
        }
    }

    return has_moved;
}

bool LeaderSingle::two_opt_inter_head_to_tail(int *route1, int *route2, int &length1, int &length2, int &loading1,
    int &loading2, AcceptRule rule) {
    if (length1 < 3 || length2 < 3) return false;

    bool has_moved = false;

    dist_i.param(Param(0, length1 - 2));
    const int n1 = dist_i(random_engine);
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int i = 0; i <= n1; i++) {
        partial_dem_r1 += instance->get_customer_demand_(route1[i]);
    }

    int partial_dem_r2 = 0;
    for (int n2 = 0; n2 < length2 - 1; ++n2) {
        if ((n1 == 0 && n2 == 0) || (n1 == length1 - 2 && n2 == length2 - 2)) continue;
        partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

        if (partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_ || partial_dem_r2 + loading1 - partial_dem_r1 > instance->max_vehicle_capa_) continue;

        const double original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) +
                                     instance->get_distance(route2[n2], route2[n2 + 1]);
        const double modified_cost = instance->get_distance(route1[n1], route2[n2 + 1]) +
                                     instance->get_distance(route2[n2], route1[n1 + 1]);

        const double change = modified_cost - original_cost;

        if (accept(rule, change)) {
            // update
            upper_cost += change;
            memcpy(temp_r1, route1, sizeof(int) * node_cap);
            int counter1 = n1 + 1;
            for (int i = n2 + 1; i < length2; i++) {
                route1[counter1++] = route2[i];
            }
            int counter2 = n2 + 1;
            for (int i = n1 + 1; i < length1; i++) {
                route2[counter2++] = temp_r1[i];
            }
            length1 = counter1;
            length2 = counter2;
            const int new_dem_sum_1 = partial_dem_r1 + loading2 - partial_dem_r2;
            const int new_dem_sum_2 = partial_dem_r2 + loading1 - partial_dem_r1;
            loading1 = new_dem_sum_1;
            loading2 = new_dem_sum_2;

            std::fill(route1 + length1, route1 + node_cap, 0);
            std::fill(route2 + length2, route2 + node_cap, 0);

            has_moved = true;
            break;
        }
    }

    return has_moved;
}

bool LeaderSingle::node_relocate_inter_with_empty_route(int *route1, int *route2, int &length1, int &length2,
    int &loading1, int &loading2, AcceptRule rule) {
    if (length1 < 4 || length2 != 0) return false; // make sure it won't generate empty route

    bool has_moved = false;

    dist_i.param(Param(1, length1 - 2));
    const int i = dist_i(random_engine);
    const int x = route1[i];
    route2[0] = instance->depot_;
    route2[1] = instance->depot_;

    const double original_cost = instance->get_distance(route1[i - 1], x) +
                                 instance->get_distance(x, route1[i + 1]);
    const double modified_cost = instance->get_distance(route1[i - 1], route1[i + 1]) +
                                 instance->get_distance(route2[0], x) +
                                 instance->get_distance(x, route2[1]);

    const double change = modified_cost - original_cost;
    if (accept(rule, change)) {
        for (int p = i; p < length1 - 1; p++) {
            route1[p] = route1[p + 1];
        }
        length1--;
        loading1 -= instance->get_customer_demand_(x);

        route2[1] = x;
        route2[2] = instance->depot_;
        length2 = 3;
        loading2 += instance->get_customer_demand_(x);
        upper_cost += change;

        has_moved = true;
    }

    return has_moved;
}
