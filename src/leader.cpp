//
// Created by Yinghao Qin on 22/01/2026.
//

#include "leader.hpp"

using namespace std;

Leader::Leader(std::mt19937& engine, Case *instance, Preprocessor *preprocessor)
    : instance(instance),
      preprocessor(preprocessor),
      random_engine(engine) {
    this->route_cap = preprocessor->route_cap_;
    this->node_cap  = preprocessor->node_cap_;

    prepare_temp_buffers(node_cap);

    prepare_greedy_moves();
    std::iota(greedy_move_indices.begin(), greedy_move_indices.end(), 0);

    // ---- reserve for route_pairs (do once) ----
    route_pairs.max_load_factor(0.7f);

    // upper-bound number of pairs ~ num_routes*(num_routes-1)/2
    // use route_cap as conservative bound, but cap it to avoid huge memory
    const size_t cap_routes = static_cast<size_t>(std::min(route_cap, 512)); // 512 or 1024 as a safe cap
    const size_t expected_pairs = cap_routes * (cap_routes - 1) / 2;

    route_pairs.reserve(expected_pairs);
    vec_route_order.reserve(route_cap);
    vec_route_pairs.reserve(expected_pairs);
}

Leader::~Leader() noexcept {
    delete[] temp_r1; temp_r1 = nullptr;
    delete[] temp_r2; temp_r2 = nullptr;
}

void Leader::load_individual(const Individual* ind) {
    this->upper_cost = ind->upper_cost;
    this->num_routes = ind->num_routes;

    this->routes = ind->routes;
    this->num_nodes_per_route = ind->num_nodes_per_route;
    this->demand_sum_per_route = ind->demand_sum_per_route;

    on_after_load(ind);   // hook
}

void Leader::export_individual(Individual* ind) const {
    ind->upper_cost = this->upper_cost;
    ind->num_routes = this->num_routes;
}

void Leader::greedy_descent(Individual *ind) {
    load_individual(ind);

    bool improvement_found = true;
    int loop_count = 0;

    while (improvement_found) {
        std::shuffle(greedy_move_indices.begin(), greedy_move_indices.end(), random_engine);

        improvement_found = false;

        for (const int choice : greedy_move_indices) {
            bool move_succeeded = false;

            if (choice < kNumIntraGreedy) {
                move_succeeded = perform_intra_move_greedy(intra_greedy_moves[choice]);
            } else {
                const int inter_idx = choice - kNumIntraGreedy;
                if (inter_idx < kNumInterGreedy) {
                    move_succeeded = perform_inter_move_greedy(inter_greedy_moves[inter_idx]);
                } else {
                    std::cerr << "Invalid greedy move index: " << inter_idx << '\n';
                    std::exit(1);
                }
            }

            if (move_succeeded) {
                improvement_found = true;
            }
        }

        ++loop_count;
    }

    export_individual(ind);
}

void Leader::sweep_descent(Individual *ind) {
    load_individual(ind);

    perform_intra_move_sweep(&Leader::move7_intra_impro_sweep);
    perform_inter_move_sweep(&Leader::move8_inter_impro_sweep);
    perform_intra_move_sweep(&Leader::move7_intra_impro_sweep);

    export_individual(ind);
}

void Leader::light_descent(Individual *ind) {
    load_individual(ind);

    // ===== 1. move budget =====
    const int n = instance->num_customer_;  // 或 problem_size_
    const int move_budget = std::clamp(static_cast<int>(2 * std::sqrt(n)), 5, 100);
    const int attempt_budget = move_budget * 5;

    int intra_moves = 0;
    int inter_moves = 0;
    int attempts = 0;

    // ===== 4. 主循环 =====
    while (intra_moves + inter_moves < move_budget && attempts < attempt_budget) {
        ++attempts;

        bool improved = false;

        if (light_inter_choice_dist(random_engine)) {
            const int id = light_inter_move_dist(random_engine);
            improved = perform_inter_move_once(inter_greedy_moves[id]);
            if (improved) inter_moves++;
        } else {
            const int id = light_intra_move_dist(random_engine);
            improved = perform_intra_move_once(intra_greedy_moves[id]);
            if (improved) intra_moves++;
        }
    }

    export_individual(ind);
}

void Leader::prepare_greedy_moves() {
    intra_greedy_moves = {
        &Leader::move1_intra_impro,
        &Leader::move2_intra_impro,
        &Leader::move3_intra_impro,
        &Leader::move4_intra_impro,
        &Leader::move5_intra_impro,
        &Leader::move6_intra_impro,
        &Leader::move7_intra_impro
    };

    inter_greedy_moves = {
        &Leader::move1_inter_impro,
        &Leader::move2_inter_impro,
        &Leader::move3_inter_impro,
        &Leader::move4_inter_impro,
        &Leader::move5_inter_impro,
        &Leader::move6_inter_impro,
        &Leader::move8_inter_impro,
        &Leader::move9_inter_impro
    };
}

void Leader::prepare_temp_buffers(const int required_size) {
    if (temp_buffer_size >= required_size) return;

    delete[] temp_r1;
    delete[] temp_r2;

    temp_r1 = new int[required_size];
    temp_r2 = new int[required_size];
    temp_buffer_size = required_size;
}

bool Leader::perform_intra_move_greedy(const IntraGreedyMemFn fn) {
    if (num_routes < 1) return false;

    bool is_successful = false;

    for (int i = 0; i < num_routes; ++i) {
        bool is_improved;
        do {
            is_improved = (this->*fn)(routes[i], num_nodes_per_route[i], AcceptRule::Greedy);
            is_successful = is_improved || is_successful;
        } while (is_improved);
    }

    return is_successful;
}

bool Leader::perform_inter_move_greedy(const InterGreedyMemFn fn) {
    if (num_routes <= 1) return false;

    bool is_successful = false;

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

    route_pairs.clear();
    for (int i = 0; i < num_routes - 1; i++) {
        for (int j = i + 1; j < num_routes; j++) {
            route_pairs.insert(make_pair(i, j));
        }
    }

    while (!route_pairs.empty()) {
        auto [r1, r2] = *route_pairs.begin();
        route_pairs.erase(route_pairs.begin());

        bool is_improved;
        do {
            is_improved = (this->*fn)(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],
                                    demand_sum_per_route[r1], demand_sum_per_route[r2],  AcceptRule::Greedy);
            is_successful = is_improved || is_successful;

            if (is_improved) {
                // clean the route beyond the length
                std::fill(routes[r1] + num_nodes_per_route[r1], routes[r1] + node_cap, 0);
                std::fill(routes[r2] + num_nodes_per_route[r2], routes[r2] + node_cap, 0);

                if (demand_sum_per_route[r1] == 0 || demand_sum_per_route[r2] == 0) {
                    const int empty_r = demand_sum_per_route[r1] == 0 ? r1 : r2;
                    remove_if_empty(empty_r);
                    for (int i = 0; i < num_routes; i++) {
                        route_pairs.erase({i, num_routes});
                    }

                    if (r1 <= num_routes - 1) {
                        for (int i = 0; i < r1; i++) {
                            route_pairs.emplace(i, r1);
                        }
                    }
                    if (r2 <= num_routes - 1) {
                        for (int i = 0; i < r2; i++) {
                            route_pairs.emplace(i, r2);
                        }
                    }

                    // Clear unused routes beyond num_routes
                    std::fill(num_nodes_per_route + num_routes, num_nodes_per_route + route_cap, 0);
                    std::fill(demand_sum_per_route + num_routes, demand_sum_per_route + route_cap, 0);

                    break;
                }
            }
        } while (is_improved);

    }

    return is_successful;
}

void Leader::relocate_pos(int* route, const int from, const int to) {
    if (from == to) return;

    const int x = route[from];

    if (from < to) {
        for (int i = from; i < to; ++i)
            route[i] = route[i + 1];
    } else { // a > b
        for (int i = from; i > to; --i)
            route[i] = route[i - 1];
    }

    route[to] = x;
}

bool Leader::move1_intra_impro(int* route, const int length, AcceptRule rule) {
    if (length <= 4) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 1; ++i) {
        for (int j = 1; j < length - 1; ++j) {
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

            change = modified_cost - original_cost;
            if (accept(rule, change)) {
                relocate_pos(route, i, j);
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }
    end_loops:;

    return has_moved;
}

bool Leader::move1_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2,
    AcceptRule rule) {
    if (length1 < 3 || length2 < 3) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length1 - 1; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) > instance->max_vehicle_capa_) {
            continue;
        }
        for (int j = 0; j < length2 - 1; ++j) {
            original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                            instance->get_distance(route1[i], route1[i + 1]) +
                            instance->get_distance(route2[j], route2[j + 1]);
            modified_cost = instance->get_distance(route1[i - 1], route1[i + 1]) +
                            instance->get_distance(route2[j], route1[i]) +
                            instance->get_distance(route1[i], route2[j + 1]);

            change = modified_cost - original_cost;
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

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool Leader::move2_intra_impro(int *route, const int length, AcceptRule rule) {
    if (length <= 4) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 2; ++i) {
        for (int j = 0; j < length - 1; j++) {
            if (j == i - 1 || j == i || j == i + 1) continue;

            if (i < j) {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[i + 2]) +
                                instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[i - 1], route[i + 2]) +
                                instance->get_distance(route[j], route[i]) +
                                instance->get_distance(route[i + 1], route[j + 1]);
            } else {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[i + 2]) +
                                instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[j], route[i]) +
                                instance->get_distance(route[i + 1], route[j + 1]) +
                                instance->get_distance(route[i - 1], route[i + 2]);
            }

            change = modified_cost - original_cost;
            if (accept(rule, change)) {
                const int u = route[i];
                const int x = route[i + 1];
                if (i < j) {
                    for (int k = i; k < j - 1; k++) {
                        route[k] = route[k + 2];
                    }
                    route[j - 1] = u;
                    route[j] = x;
                }
                else if (i > j) {
                    for (int k = i + 1; k > j + 2; k--) {
                        route[k] = route[k - 2];
                    }
                    route[j + 1] = u;
                    route[j + 2] = x;
                }
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool Leader::move2_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2,
    AcceptRule rule) {
    if (length1 < 4 || length2 < 3) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length1 - 2; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) + instance->get_customer_demand_(route1[ i + 1])
            > instance->max_vehicle_capa_) {
            continue;
        }

        for (int j = 0; j < length2 - 1; ++j) {
            original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                            instance->get_distance(route1[i + 1], route1[i + 2]) +
                            instance->get_distance(route2[j], route2[j + 1]);
            modified_cost = instance->get_distance(route1[i - 1], route1[i + 2]) +
                            instance->get_distance(route2[j], route1[i]) +
                            instance->get_distance(route1[i + 1], route2[j + 1]);

            change = modified_cost - original_cost;
            if (accept(rule, change)) {
                const int u = route1[i];
                const int x = route1[i + 1];
                for (int p = i; p < length1 - 2; ++p) {
                    route1[p] = route1[p + 2];
                }
                length1 -= 2;
                loading1 -= instance->get_customer_demand_(u);
                loading1 -= instance->get_customer_demand_(x);

                for (int q = length2 + 1; q > j + 2; q--) {
                    route2[q] = route2[q - 2];
                }
                route2[j + 1] = u;
                route2[j + 2] = x;
                length2 += 2;
                loading2 += instance->get_customer_demand_(u);
                loading2 += instance->get_customer_demand_(x);
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }
    end_loops:;

    return has_moved;
}

bool Leader::move3_intra_impro(int *route, const int length, AcceptRule rule) {
    if (length <= 4) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 2; ++i) {
        for (int j = 0; j < length - 1; j++) {
            if (j == i - 1 || j == i || j == i + 1) continue;

            if (i < j) {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[i + 2]) +
                                instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[i - 1], route[i + 2]) +
                                instance->get_distance(route[j], route[i + 1]) +
                                instance->get_distance(route[i], route[j + 1]);
            } else {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[i + 2]) +
                                instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[j], route[i + 1]) +
                                instance->get_distance(route[i], route[j + 1]) +
                                instance->get_distance(route[i - 1], route[i + 2]);
            }

            change = modified_cost - original_cost;
            if (accept(rule, change)) {
                const int u = route[i];
                const int x = route[i + 1];
                if (i < j) {
                    for (int k = i; k < j - 1; k++) {
                        route[k] = route[k + 2];
                    }
                    route[j - 1] = x;
                    route[j] = u;
                }
                else if (i > j) {
                    for (int k = i + 1; k > j + 2; k--) {
                        route[k] = route[k - 2];
                    }
                    route[j + 1] = x;
                    route[j + 2] = u;
                }
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool Leader::move3_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2,
    AcceptRule rule) {
    if (length1 < 4 || length2 < 3) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length1 - 2; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) + instance->get_customer_demand_(route1[ i + 1]) > instance->max_vehicle_capa_) {
            continue;
        }

        for (int j = 0; j < length2 - 1; ++j) {
            original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                            instance->get_distance(route1[i + 1], route1[i + 2]) +
                            instance->get_distance(route2[j], route2[j + 1]);
            modified_cost = instance->get_distance(route1[i - 1], route1[i + 2]) +
                            instance->get_distance(route2[j], route1[i + 1]) +
                            instance->get_distance(route1[i], route2[j + 1]);

            change = modified_cost - original_cost;
            if (accept(rule, change)) {
                const int u = route1[i];
                const int x = route1[i + 1];
                for (int p = i; p < length1 - 2; ++p) {
                    route1[p] = route1[p + 2];
                }
                length1 -= 2;
                loading1 -= instance->get_customer_demand_(u);
                loading1 -= instance->get_customer_demand_(x);

                for (int q = length2 + 1; q > j + 2; q--) {
                    route2[q] = route2[q - 2];
                }
                route2[j + 1] = x;
                route2[j + 2] = u;
                length2 += 2;
                loading2 += instance->get_customer_demand_(u);
                loading2 += instance->get_customer_demand_(x);
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool Leader::move4_intra_impro(int* route, const int length, AcceptRule rule) {
    if (length < 5) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 2; ++i) {
        for (int j = i + 1; j < length - 1; ++j) {
            if (j == i + 1) {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[i - 1], route[j]) +
                                instance->get_distance(route[i], route[j + 1]);
            } else {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i], route[i + 1]) +
                                instance->get_distance(route[j - 1], route[j]) +
                                instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[i - 1], route[j]) +
                                instance->get_distance(route[j], route[i + 1]) +
                                instance->get_distance(route[j - 1], route[i]) +
                                instance->get_distance(route[i], route[j + 1]);
            }

            change = modified_cost - original_cost;
            if (accept(rule, change)) {
                swap(route[i], route[j]);
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }
    end_loops:;

    return has_moved;
}

bool Leader::move4_inter_impro(int *route1, int *route2, int& length1, int& length2, int &loading1,
    int &loading2, AcceptRule rule) {
    if (length1 < 3 || length2 < 3) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length1 - 1; ++i) {
        for (int j = 1; j < length2 - 1; ++j) {
            const int demand_I = instance->get_customer_demand_(route1[i]);
            const int demand_J = instance->get_customer_demand_(route2[j]);
            if (loading1 - demand_I + demand_J <= instance->max_vehicle_capa_ && loading2 - demand_J + demand_I <= instance->max_vehicle_capa_) {
                original_cost = instance->get_distance(route1[i - 1], route1[i]) + instance->get_distance(route1[i], route1[i + 1]) +
                                instance->get_distance(route2[j - 1], route2[j]) + instance->get_distance(route2[j], route2[j + 1]);
                modified_cost = instance->get_distance(route1[i - 1], route2[j]) + instance->get_distance(route2[j], route1[i + 1]) +
                                instance->get_distance(route2[j - 1], route1[i]) + instance->get_distance(route1[i], route2[j + 1]);

                change = modified_cost - original_cost;
                if (accept(rule, change)) {
                    swap(route1[i], route2[j]);
                    loading1 = loading1 - demand_I + demand_J;
                    loading2 = loading2 - demand_J + demand_I;
                    upper_cost += change;

                    has_moved = true;
                    goto end_loops;
                }
            }
        }
    }
    end_loops:;

    return has_moved;
}

bool Leader::move5_intra_impro(int *route, const int length, AcceptRule rule) {
    if (length < 5) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 3; ++i) {
        for (int j = i + 2; j < length - 1; ++j) {
            if (j == i + 2) {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[j]) +
                                instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[i - 1], route[j]) +
                                instance->get_distance(route[j], route[i]) +
                                instance->get_distance(route[i + 1], route[j + 1]);
            } else {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[i + 2]) +
                                instance->get_distance(route[j - 1], route[j]) +
                                instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[i - 1], route[j]) +
                                instance->get_distance(route[j], route[i + 2]) +
                                instance->get_distance(route[j - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[j + 1]);
            }
            change = modified_cost - original_cost;

            if (accept(rule, change)) {
                // node pair (route[i], route[i + 1]) will be changed with node route[j]
                if (j == i + 2) {
                    std::swap(route[i], route[j]);
                    std::swap(route[i + 1], route[j]);
                } else {
                    std::swap(route[i], route[j]);
                    const int x = route[i + 1];
                    for (int k = i + 1; k < j; ++k) {
                        route[k] = route[k + 1];
                    }
                    route[j] = x;
                }

                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool Leader::move5_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2,
    AcceptRule rule) {
    if (length1 < 4 || length2 < 4) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length1 - 2; ++i) {
        for (int j = 1; j < length2 - 1; ++j) {
            const int demand_pair = instance->get_customer_demand_(route1[i]) + instance->get_customer_demand_(route1[i + 1]);
            const int demand_J = instance->get_customer_demand_(route2[j]);
            if (loading1 - demand_pair + demand_J > instance->max_vehicle_capa_ || loading2 - demand_J + demand_pair
                > instance->max_vehicle_capa_) {
                continue;
            }

            original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                            instance->get_distance(route1[i + 1], route1[i + 2]) +
                            instance->get_distance(route2[j - 1], route2[j]) +
                            instance->get_distance(route2[j], route2[j + 1]);
            modified_cost = instance->get_distance(route1[i - 1], route2[j]) +
                            instance->get_distance(route2[j], route1[i + 2]) +
                            instance->get_distance(route2[j - 1], route1[i]) +
                            instance->get_distance(route1[i + 1], route2[j + 1]);

            change = modified_cost - original_cost;
            if (accept(rule, change)) {
                // node pair (route1[i], route1[i + 1]) will be changed with node route2[j]
                const int u = route1[i];
                const int x = route1[i + 1];
                route1[i] = route2[j];
                for (int p = i + 1; p < length1 - 1; ++p) {
                    route1[p] = route1[p + 1];
                }
                length1--;
                loading1 = loading1 - demand_pair + demand_J;

                for (int q = length2; q > j + 1; q--) {
                    route2[q] = route2[q - 1];
                }
                route2[j] = u;
                route2[j + 1] = x;
                length2++;
                loading2 = loading2 - demand_J + demand_pair;

                upper_cost += change;
                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool Leader::move6_intra_impro(int *route, const int length, AcceptRule rule) {
    if (length < 6) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 4; ++i) {
        for (int j = i + 2; j < length - 2; ++j) {
            if (j == i + 2) {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[j]) +
                                instance->get_distance(route[j + 1], route[j + 2]);
                modified_cost = instance->get_distance(route[i - 1], route[j]) +
                                instance->get_distance(route[j + 1], route[i]) +
                                instance->get_distance(route[i + 1], route[j + 2]);
            } else {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[i + 2]) +
                                instance->get_distance(route[j - 1], route[j]) +
                                instance->get_distance(route[j + 1], route[j + 2]);
                modified_cost = instance->get_distance(route[i - 1], route[j]) +
                                instance->get_distance(route[j + 1], route[i + 2]) +
                                instance->get_distance(route[j - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[j + 2]);
            }
            change = modified_cost - original_cost;

            if (accept(rule, change)) {
                swap(route[i], route[j]);
                swap(route[i + 1], route[j + 1]);
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool Leader::move6_inter_impro(int *route1, int *route2, int& length1, int& length2, int &loading1,
    int &loading2, AcceptRule rule) {
    if (length1 < 4 || length2 < 4 || (length1 == 4 && length2 == 4)) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length1 - 2; ++i) {
        for (int j = 1; j < length2 - 2; ++j) {
            const int demand_I_pair = instance->get_customer_demand_(route1[i]) +
                                      instance->get_customer_demand_(route1[i + 1]);
            const int demand_J_pair = instance->get_customer_demand_(route2[j]) +
                                      instance->get_customer_demand_(route2[j + 1]);
            if (loading1 - demand_I_pair + demand_J_pair > instance->max_vehicle_capa_ ||
                loading2 - demand_J_pair + demand_I_pair > instance->max_vehicle_capa_) {
                continue;
            }

            original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                            instance->get_distance(route1[i + 1], route1[i + 2]) +
                            instance->get_distance(route2[j - 1], route2[j]) +
                            instance->get_distance(route2[j + 1], route2[j + 2]);
            modified_cost = instance->get_distance(route1[i - 1], route2[j]) +
                            instance->get_distance(route2[j + 1], route1[i + 2]) +
                            instance->get_distance(route2[j - 1], route1[i]) +
                            instance->get_distance(route1[i + 1], route2[j + 2]);
            change = modified_cost - original_cost;

            if (accept(rule, change)) {
                swap(route1[i], route2[j]);
                swap(route1[i + 1], route2[j + 1]);
                loading1 = loading1 - demand_I_pair + demand_J_pair;
                loading2 = loading2 - demand_J_pair + demand_I_pair;
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool Leader::move7_intra_impro(int *route, const int length, AcceptRule rule) {
    if (length < 5) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 2; ++i) {
        for (int j = i + 1; j < length - 1; ++j) {
            // Calculate the cost difference between the old route and the new route obtained by swapping arcs
            original_cost = instance->get_distance(route[i - 1], route[i]) +
                            instance->get_distance(route[j], route[j + 1]);
            modified_cost = instance->get_distance(route[i - 1], route[j]) +
                            instance->get_distance(route[i], route[j + 1]);

            change = modified_cost - original_cost; // negative represents the cost reduction
            if (accept(rule, change)) {
                // update current solution
                reverse(route + i, route + j + 1);
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool Leader::move8_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2,
    AcceptRule rule) {
    if (length1 < 3 || length2 < 3) return false;

    memset(temp_r1, 0, sizeof(int) * node_cap);
    memset(temp_r2, 0, sizeof(int) * node_cap);

    bool has_moved = false;

    double original_cost, modified_cost, change;


    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int n1 = 0; n1 < length1 - 1; ++n1) {
        partial_dem_r1 += instance->get_customer_demand_(route1[n1]);

        int partial_dem_r2 = 0;
        for (int n2 = 0; n2 < length2 - 1; ++n2) {
            if ((n1 == length1 - 2 && n2 == 0) || (n1 == 0 && n2 == length2 - 2)) continue; // the same as the current situation, just skip it.
            partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

            if (partial_dem_r1 + partial_dem_r2 > instance->max_vehicle_capa_ ||
                loading1 - partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_) continue;

            original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) +
                            instance->get_distance(route2[n2], route2[n2 + 1]);
            modified_cost = instance->get_distance(route1[n1], route2[n2]) +
                            instance->get_distance(route1[n1 + 1], route2[n2 + 1]);

            change = modified_cost - original_cost;
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

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool Leader::move9_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2,
    AcceptRule rule) {
    if (length1 < 3 || length2 < 3) return false;

    memset(temp_r1, 0, sizeof(int) * node_cap);

    bool has_moved = false;

    double original_cost, modified_cost, change;

    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int n1 = 0; n1 < length1 - 1; ++n1) {
        partial_dem_r1 += instance->get_customer_demand_(route1[n1]);

        int partial_dem_r2 = 0;
        for (int n2 = 0; n2 < length2 - 1; ++n2) {
            if ((n1 == 0 && n2 == 0) || (n1 == length1 - 2 && n2 == length2 - 2)) continue;
            partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

            if (partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_
                || partial_dem_r2 + loading1 - partial_dem_r1 > instance->max_vehicle_capa_) continue;

            original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) +
                            instance->get_distance(route2[n2], route2[n2 + 1]);
            modified_cost = instance->get_distance(route1[n1], route2[n2 + 1]) +
                            instance->get_distance(route2[n2], route1[n1 + 1]);

            change = modified_cost - original_cost;
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

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool Leader::perform_intra_move_sweep(IntraGreedyMemFn fn) {
    if (num_routes < 1) return false;

    bool is_successful = false;

    for (int i = 0; i < num_routes; ++i) {
        const bool is_improved = (this->*fn)(routes[i], num_nodes_per_route[i], AcceptRule::Greedy);
        is_successful = is_improved || is_successful;
    }

    return is_successful;
}

bool Leader::perform_inter_move_sweep(InterGreedyMemFn fn) {
    if (num_routes <= 1) return false;

    bool is_successful = false;

    // 1. 一次性生成所有路线对（推荐用 vector 而不是 set，顺序更可控）
    vec_route_pairs.clear();

    for (int i = 0; i < num_routes - 1; ++i) {
        for (int j = i + 1; j < num_routes; ++j) {
            vec_route_pairs.emplace_back(i, j);
        }
    }

    // 可选：随机打乱顺序（效果通常更好，推荐加上）
    std::shuffle(vec_route_pairs.begin(), vec_route_pairs.end(), random_engine);

    // 2. 只遍历一次所有路线对
    for (auto& p : vec_route_pairs) {
        int r1 = p.first;
        int r2 = p.second;

        if (r1 >= num_routes || r2 >= num_routes) continue;
        if (demand_sum_per_route[r1] == 0 || demand_sum_per_route[r2] == 0) continue;
        if (num_nodes_per_route[r1] < 3 || num_nodes_per_route[r2] < 3) continue;

        // Sweep Greedy Move
        bool improved = (this->*fn)(routes[r1], routes[r2],
                                    num_nodes_per_route[r1], num_nodes_per_route[r2],
                                    demand_sum_per_route[r1], demand_sum_per_route[r2],
                                    AcceptRule::Greedy);

        if (improved) {
            is_successful = true;

            // clean up
            std::fill(routes[r1] + num_nodes_per_route[r1], routes[r1] + node_cap, 0);
            std::fill(routes[r2] + num_nodes_per_route[r2], routes[r2] + node_cap, 0);
        }
    }

    int write_idx = 0;
    for (int read_idx = 0; read_idx < num_routes; ++read_idx) {
        if (demand_sum_per_route[read_idx] == 0) continue;

        if (write_idx != read_idx) {
            std::swap(routes[write_idx], routes[read_idx]);
            std::swap(num_nodes_per_route[write_idx], num_nodes_per_route[read_idx]);
            std::swap(demand_sum_per_route[write_idx], demand_sum_per_route[read_idx]);
        }
        ++write_idx;
    }

    for (int route_idx = write_idx; route_idx < route_cap; ++route_idx) {
        memset(routes[route_idx], 0, sizeof(int) * node_cap);
        num_nodes_per_route[route_idx] = 0;
        demand_sum_per_route[route_idx] = 0;
    }
    num_routes = write_idx;

    return is_successful;
}

bool Leader::move7_intra_impro_sweep(int *route, int length, AcceptRule rule) {
    if (length < 5) return false;

    bool has_moved = false;

restart_route:
    for (int i = 1; i < length - 2; ++i) {
        for (int j = i + 1; j < length - 1; ++j) {
            // Calculate the cost difference between the old route and the new route obtained by swapping arcs
            const double original_cost = instance->get_distance(route[i - 1], route[i]) +
                                         instance->get_distance(route[j], route[j + 1]);
            const double modified_cost = instance->get_distance(route[i - 1], route[j]) +
                                         instance->get_distance(route[i], route[j + 1]);

            const double change = modified_cost - original_cost; // negative represents the cost reduction
            if (accept(rule, change)) {
                // Rebuild the current route immediately after accepting a move
                // so the remaining scan sees the updated local order.
                reverse(route + i, route + j + 1);
                upper_cost += change;

                has_moved = true;
                goto restart_route;
            }
        }
    }

    return has_moved;
}

bool Leader::move8_inter_impro_sweep(int *r1, int *r2, int &len1, int &len2, int &load1, int &load2,
    AcceptRule rule) {
    if (len1 < 3 || len2 < 3) return false;

    bool has_moved = false;

restart_pair:
    if (len1 < 3 || len2 < 3) return has_moved;

    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int n1 = 0; n1 < len1 - 1; ++n1) {
        partial_dem_r1 += instance->get_customer_demand_(r1[n1]);

        int partial_dem_r2 = 0;
        for (int n2 = 0; n2 < len2 - 1; ++n2) {
            if ((n1 == len1 - 2 && n2 == 0) || (n1 == 0 && n2 == len2 - 2)) continue; // the same as the current situation, just skip it.
            partial_dem_r2 += instance->get_customer_demand_(r2[n2]);

            if (partial_dem_r1 + partial_dem_r2 > instance->max_vehicle_capa_ ||
                load1 - partial_dem_r1 + load2 - partial_dem_r2 > instance->max_vehicle_capa_) continue;

            const double original_cost = instance->get_distance(r1[n1], r1[n1 + 1]) +
                                         instance->get_distance(r2[n2], r2[n2 + 1]);
            const double modified_cost = instance->get_distance(r1[n1], r2[n2]) +
                                         instance->get_distance(r1[n1 + 1], r2[n2 + 1]);

            const double change = modified_cost - original_cost;
            if (accept(rule, change)) {
                // Rebuild the current route pair immediately after accepting a move
                // so the remaining scan sees a consistent local state.
                const int old_len1 = len1;
                const int old_len2 = len2;
                const int old_load1 = load1;
                const int old_load2 = load2;

                memcpy(temp_r1, r1, sizeof(int) * node_cap);

                int new_len1 = n1 + 1;
                for (int i = n2; i >= 0; --i) {
                    r1[new_len1++] = r2[i];
                }

                int new_len2 = 0;
                for (int i = old_len1 - 1; i >= n1 + 1; --i) {
                    temp_r2[new_len2++] = temp_r1[i];
                }
                for (int i = n2 + 1; i < old_len2; ++i) {
                    temp_r2[new_len2++] = r2[i];
                }
                memcpy(r2, temp_r2, sizeof(int) * node_cap);

                len1 = new_len1;
                len2 = new_len2;
                load1 = partial_dem_r1 + partial_dem_r2;
                load2 = old_load1 - partial_dem_r1 + old_load2 - partial_dem_r2;
                upper_cost += change;

                has_moved = true;
                goto restart_pair;
            }
        }
    }

    return has_moved;
}

bool Leader::perform_intra_move_once(const IntraGreedyMemFn fn) {
    if (num_routes < 1) return false;

    bool is_successful = false;

    // Reuse the scratch buffer to avoid allocating a fresh route-order vector
    // on every light-descent attempt.
    vec_route_order.resize(num_routes);
    std::iota(vec_route_order.begin(), vec_route_order.end(), 0);
    std::shuffle(vec_route_order.begin(), vec_route_order.end(), random_engine);

    for (const int r : vec_route_order) {
        if ((this->*fn)(routes[r], num_nodes_per_route[r], AcceptRule::Greedy)) {
            is_successful = true;
            break;  // ⭐ 一次就停（关键！）
        }
    }

    return is_successful;
}

bool Leader::perform_inter_move_once(const InterGreedyMemFn fn) {
    if (num_routes <= 1) return false;

    bool is_successful = false;

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

    // 1. 一次性生成所有路线对（推荐用 vector 而不是 set，顺序更可控）
    vec_route_pairs.clear();

    for (int i = 0; i < num_routes - 1; ++i) {
        for (int j = i + 1; j < num_routes; ++j) {
            vec_route_pairs.emplace_back(i, j);
        }
    }

    // 可选：随机打乱顺序（效果通常更好，推荐加上）
    std::shuffle(vec_route_pairs.begin(), vec_route_pairs.end(), random_engine);

    // 2. 只遍历一次所有路线对
    for (auto&[r1, r2] : vec_route_pairs) {

        if (r1 >= num_routes || r2 >= num_routes) continue;
        if (demand_sum_per_route[r1] == 0 || demand_sum_per_route[r2] == 0) continue;
        if (num_nodes_per_route[r1] < 3 || num_nodes_per_route[r2] < 3) continue;

        if ((this->*fn)(routes[r1], routes[r2],
                        num_nodes_per_route[r1], num_nodes_per_route[r2],
                        demand_sum_per_route[r1], demand_sum_per_route[r2],
                        AcceptRule::Greedy)) {
            is_successful = true;
            std::fill(routes[r1] + num_nodes_per_route[r1], routes[r1] + node_cap, 0);
            std::fill(routes[r2] + num_nodes_per_route[r2], routes[r2] + node_cap, 0);

            if (demand_sum_per_route[r1] == 0 || demand_sum_per_route[r2] == 0) {
                const int empty_r = demand_sum_per_route[r1] == 0 ? r1 : r2;
                remove_if_empty(empty_r);

                // Clear unused routes beyond num_routes
                std::fill(num_nodes_per_route + num_routes, num_nodes_per_route + route_cap, 0);
                std::fill(demand_sum_per_route + num_routes, demand_sum_per_route + route_cap, 0);
            }

            break; // ⭐ 一次就停（关键！）
        }
    }

    return is_successful;
}

std::ostream& operator<<(std::ostream& os, const Leader& leader) {
    os << "Route Capacity: " << leader.route_cap << "\n";
    os << "Node Capacity: " << leader.node_cap << "\n";
    os << "Number of Routes: " << leader.num_routes << "\n";
    os << "Upper Cost: " << leader.upper_cost << "\n";

    os << "Number of Nodes per route (upper): ";
    for (int i = 0; i < leader.route_cap; ++i) {
        os << leader.num_nodes_per_route[i] << " ";
    }
    os << "\n";

    os << "Demand sum per route: ";
    for (int i = 0; i < leader.route_cap; ++i) {
        os << leader.demand_sum_per_route[i] << " ";
    }
    os << "\n";

    os << "Upper Routes: \n";
    for (int i = 0; i < leader.route_cap; ++i) {
        os << "Route " << i << ": ";
        for (int j = 0; j < leader.node_cap; ++j) {
            os << leader.routes[i][j] << " ";
        }
        os << "\n";
    }

    return os;
}
