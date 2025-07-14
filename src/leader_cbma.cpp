//
// Created by Yinghao Qin on 10/06/2025.
//

#include "leader_cbma.hpp"

namespace {
    thread_local std::mt19937 thread_rng(std::random_device{}());
}

bool LeaderCbma::greedy_accept(const double &change, [[maybe_unused]] const double &temperature) {
    return change < -EPSILON;
}

bool LeaderCbma::neighbor_accept(const double &change, const double &temperature) {
    if (change < -EPSILON) return true; // Always accept a move that improves the solution

    std::uniform_real_distribution dis(0.0, 1.0);
    const double probability = std::exp(-change / temperature);
    return dis(thread_rng) < probability;
}

LeaderCbma::LeaderCbma(Case *instance, Preprocessor *preprocessor)
        : instance(instance),
          preprocessor(preprocessor) {

    prepare_moves();

    this->partial_sol = nullptr;
    this->route_cap = preprocessor->route_cap_;
    this->node_cap  = preprocessor->node_cap_;
    this->greedy_moves_count = 15;
    this->greedy_move_indices.resize(greedy_moves_count);
    std::iota(greedy_move_indices.begin(), greedy_move_indices.end(), 0);
    this->num_routes = 0;
    this->upper_cost = 0.;
    this->routes = new int *[route_cap];
    for (int i = 0; i < route_cap; ++i) {
        this->routes[i] = new int[node_cap];
        memset(this->routes[i], 0, sizeof(int) * node_cap);
    }
    this->num_nodes_per_route = new int[route_cap];
    memset(this->num_nodes_per_route, 0, sizeof(int) * route_cap);
    this->demand_sum_per_route = new int [route_cap];
    memset(this->demand_sum_per_route, 0, sizeof(int) * route_cap);

    prepare_temp_buffers(node_cap);
    perturb_move_dist = uniform_int_distribution<int>(0, 17);
    neigh_search_dist = uniform_int_distribution<int>(0, 14);
}

LeaderCbma::~LeaderCbma() {
    for (int i = 0; i < route_cap; ++i) {
        delete[] routes[i];
    }
    delete[] routes;
    delete[] num_nodes_per_route;
    delete[] demand_sum_per_route;

    delete[] temp_r1;
    delete[] temp_r2;
}

void LeaderCbma::prepare_temp_buffers(const int required_size) {
    if (temp_buffer_size >= required_size) return;

    delete[] temp_r1;
    delete[] temp_r2;

    temp_r1 = new int[required_size];
    temp_r2 = new int[required_size];
    temp_buffer_size = required_size;
}

void LeaderCbma::prepare_moves() {
    intra_perturb_moves = {
        [this](int* r, const int l){return move1_intra_pert(r, l);},
        [this](int* r, const int l){return move2_intra_pert(r, l);},
        [this](int* r, const int l){return move3_intra_pert(r, l);},
        [this](int* r, const int l){return move4_intra_pert(r, l);},
        [this](int* r, const int l){return move5_intra_pert(r, l);},
        [this](int* r, const int l){return move6_intra_pert(r, l);},
        [this](int* r, const int l){return move7_intra_pert(r, l);}
    };

    inter_perturb_moves = {
        [this](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
            return move1_inter_pert(r1, r2, l1, l2, ld1, ld2);
        },
        [this](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
            return move2_inter_pert(r1, r2, l1, l2, ld1, ld2);
        },
        [this](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
            return move3_inter_pert(r1, r2, l1, l2, ld1, ld2);
        },
        [this](int* r1, int* r2, const int& l1, const int& l2, int& ld1, int& ld2) {
            return move4_inter_pert(r1, r2, l1, l2, ld1, ld2);
        },
        [this](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
            return move5_inter_pert(r1, r2, l1, l2, ld1, ld2);
        },
        [this](int* r1, int* r2, const int& l1, const int& l2, int& ld1, int& ld2) {
            return move6_inter_pert(r1, r2, l1, l2, ld1, ld2);
        },
        [this](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
            return move8_inter_pert(r1,r2,l1,l2 ,ld1 ,ld2);
        },
        [this](int* r1,int *r2,int &l1,int &l2,int &ld1,int &ld2){
            return move9_inter_pert(r1,r2,l1,l2 ,ld1 ,ld2);
        }
    };

    inter_perturb_with_empty_moves = {
        [this](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
            return move1_inter_with_empty_route_pert(r1, r2, l1, l2, ld1, ld2);
        },
        [this](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
            return move8_inter_with_empty_route_pert(r1, r2, l1, l2, ld1, ld2);
        },
        [this](int* r1,int *r2,int &l1,int &l2,int &ld1,int &ld2){
            return move9_inter_with_empty_route_pert(r1,r2,l1,l2 ,ld1 ,ld2);
        }
    };


    intra_greedy_moves = {
    [this](int* r, const int l, const double t){return move1_intra_impro(r, l, greedy_accept, t);},
    [this](int* r, const int l, const double t){return move2_intra_impro(r, l, greedy_accept, t);},
    [this](int* r, const int l, const double t){return move3_intra_impro(r, l, greedy_accept, t);},
    [this](int* r, const int l, const double t){return move4_intra_impro(r, l, greedy_accept, t);},
    [this](int* r, const int l, const double t){return move5_intra_impro(r, l, greedy_accept, t);},
    [this](int* r, const int l, const double t){return move6_intra_impro(r, l, greedy_accept, t);},
    [this](int* r, const int l, const double t){return move7_intra_impro(r, l, greedy_accept, t);}
    };

    inter_greedy_moves = {
        [this](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2, const double t) {
            return move1_inter_impro(r1, r2, l1, l2, ld1, ld2, greedy_accept, t);
        },
        [this](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2, const double t) {
            return move2_inter_impro(r1, r2, l1, l2, ld1, ld2, greedy_accept, t);
        },
        [this](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2, const double t) {
            return move3_inter_impro(r1, r2, l1, l2, ld1, ld2, greedy_accept, t);
        },
        [this](int* r1, int* r2, const int& l1, const int& l2, int& ld1, int& ld2, const double t) {
            return move4_inter_impro(r1, r2, l1, l2, ld1, ld2,greedy_accept, t);
        },
        [this](int* r1,int *r2,int &l1,int &l2,int &ld1,int &ld2, const double t){
            return move5_inter_impro(r1,r2,l1,l2 ,ld1 ,ld2 ,greedy_accept, t);
        },
        [this](int* r1,int *r2, const int &l1, const int &l2,int &ld1,int &ld2, const double t){
            return move6_inter_impro(r1,r2,l1,l2 ,ld1 ,ld2 ,greedy_accept, t);
        },
        [this](int* r1,int *r2,int &l1,int &l2,int &ld1,int &ld2, const double t){
            return move8_inter_impro(r1,r2,l1,l2 ,ld1 ,ld2 ,greedy_accept, t);
        },
        [this](int* r1,int *r2,int &l1,int &l2,int &ld1,int &ld2, const double t){
            return move9_inter_impro(r1,r2,l1,l2 ,ld1 ,ld2 ,greedy_accept, t);
        }
    };


    intra_neigh_moves = {
    [this](int* r, const int l, const double t) { return move1_intra_impro(r, l, neighbor_accept, t); },
    [this](int* r, const int l, const double t) { return move2_intra_impro(r, l, neighbor_accept, t); },
    [this](int* r, const int l, const double t) { return move3_intra_impro(r, l, neighbor_accept, t); },
    [this](int* r, const int l, const double t) { return move4_intra_impro(r, l, neighbor_accept, t); },
    [this](int* r, const int l, const double t) { return move5_intra_impro(r, l, neighbor_accept, t); },
    [this](int* r, const int l, const double t) { return move6_intra_impro(r, l, neighbor_accept, t); },
    [this](int* r, const int l, const double t) { return move7_intra_impro(r, l, neighbor_accept, t); },
    };

    inter_neigh_moves = {
        [this](int* r1, int* r2, int& l1, int& l2, int& d1, int& d2, const double t) {
            return move1_inter_impro(r1, r2, l1, l2, d1, d2, neighbor_accept, t);
        },
        [this](int* r1, int* r2, int& l1, int& l2, int& d1, int& d2, const double t) {
            return move2_inter_impro(r1, r2, l1, l2, d1, d2, neighbor_accept, t);
        },
        [this](int* r1, int* r2, int& l1, int& l2, int& d1, int& d2, const double t) {
            return move3_inter_impro(r1, r2, l1, l2, d1, d2, neighbor_accept, t);
        },
        [this](int* r1, int* r2, const int& l1, const int& l2, int& d1, int& d2, const double t) {
            return move4_inter_impro(r1, r2, l1, l2, d1, d2, neighbor_accept, t);
        },
        [this](int* r1, int* r2, int& l1, int& l2, int& d1, int& d2, const double t) {
            return move5_inter_impro(r1, r2, l1, l2, d1, d2, neighbor_accept, t);
        },
        [this](int* r1, int* r2, const int& l1, const int& l2, int& d1, int& d2, const double t) {
            return move6_inter_impro(r1, r2, l1, l2, d1, d2, neighbor_accept, t);
        },
        [this](int* r1, int* r2, int& l1, int& l2, int& d1, int& d2, const double t) {
            return move8_inter_impro(r1, r2, l1, l2, d1, d2, neighbor_accept, t);
        },
        [this](int* r1, int* r2, int& l1, int& l2, int& d1, int& d2, const double t) {
            return move9_inter_impro(r1, r2, l1, l2, d1, d2, neighbor_accept, t);
        }
    };
}


void LeaderCbma::clean() {
    this->num_routes = 0;
    this->upper_cost = 0.;

    memset(this->num_nodes_per_route, 0, sizeof(int) * this->route_cap);
    memset(this->demand_sum_per_route, 0, sizeof(int) * this->route_cap);
    for (int i = 0; i < this->route_cap; ++i) {
        memset(this->routes[i], 0, sizeof(int) * this->node_cap);
    }
}

void LeaderCbma::clean_empty_routes(const int r1, const int r2) {
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

    // clean the route beyond the length
    std::fill(routes[r1] + num_nodes_per_route[r1], routes[r1] + node_cap, 0);
    std::fill(routes[r2] + num_nodes_per_route[r2], routes[r2] + node_cap, 0);

    if (demand_sum_per_route[r1] == 0 || demand_sum_per_route[r2] == 0) {
        const int empty_r = demand_sum_per_route[r1] == 0 ? r1 : r2;
        remove_if_empty(empty_r);
    }

    // Clear unused routes beyond num_routes
    std::fill(num_nodes_per_route + num_routes, num_nodes_per_route + route_cap, 0);
    std::fill(demand_sum_per_route + num_routes, demand_sum_per_route + route_cap, 0);
}

void LeaderCbma::moveItoJ(int* route, const int a, const int b) {
    const int x = route[a];
    if (a < b) {
        for (int i = a; i < b; i++) {
            route[i] = route[i + 1];
        }
        route[b] = x;
    }
    else if (a > b) {
        for (int i = a; i > b; i--) {
            route[i] = route[i - 1];
        }
        route[b] = x;
    }
}

std::pair<int, int> LeaderCbma::randomly_select_diff_route_pair() const noexcept {
    if (num_routes < 2) {
        std::cerr << "Error at selecting two different route" << '\n';
        std::exit(1);
    }

    std::uniform_int_distribution<> dist(0, num_routes - 1);
    int r1 = dist(thread_rng);
    int r2;
    do {
        r2 = dist(thread_rng);
    } while (r1 == r2);

    return {r1, r2};
}

void LeaderCbma::load_individual(const Individual* ind) {
    clean();

    this->upper_cost = ind->upper_cost;
    this->num_routes = ind->num_routes;
    memcpy(this->num_nodes_per_route, ind->num_nodes_per_route, sizeof(int) * ind->route_cap);
    memcpy(this->demand_sum_per_route, ind->demand_sum_per_route, sizeof(int) * ind->route_cap);
    for (int i = 0; i < ind->num_routes; ++i) {
        memcpy(this->routes[i], ind->routes[i], sizeof(int) * ind->node_cap);
    }
}

void LeaderCbma::export_individual(Individual* ind) const {
    ind->upper_cost = this->upper_cost;
    ind->num_routes = this->num_routes;
    memcpy(ind->num_nodes_per_route, this->num_nodes_per_route, sizeof(int) * this->route_cap);
    memcpy(ind->demand_sum_per_route, this->demand_sum_per_route, sizeof(int) * this->route_cap);
    for (int i = 0; i < this->num_routes; ++i) {
        memcpy(ind->routes[i], this->routes[i], sizeof(int) * this->node_cap);
    }
}

bool LeaderCbma::perturbation(const int strength) {
    bool at_least_one_move = false;

    for (int i = 0; i < strength; ++i) {
        const int choice = perturb_move_dist(thread_rng);
        bool move_succeeded = false;

        // Intra moves (0-6)
        if (choice < 7) {
            move_succeeded = perform_intra_move_pert(intra_perturb_moves[choice]);
        }
        // Inter moves (7-14)
        else if (choice < 15) {
            const int inter_idx = choice - 7;
            move_succeeded = perform_inter_move_pert(inter_perturb_moves[inter_idx]);
        }
        // Inter moves with empty (15-17)
        else {
            const int empty_idx = choice - 15;
            move_succeeded = perform_inter_move_with_empty_pert(
                inter_perturb_with_empty_moves[empty_idx]);
        }

        at_least_one_move = at_least_one_move || move_succeeded;
    }

    return at_least_one_move;
}

void LeaderCbma::fully_greedy_local_optimum(Individual *ind) {
    load_individual(ind);

    bool improvement_found = true;
    int loop_count = 0;

    while (improvement_found) {
        std::shuffle(greedy_move_indices.begin(), greedy_move_indices.end(), thread_rng);

        improvement_found = false;

        for (const int choice : greedy_move_indices) {
            bool move_succeeded = false;

            if (choice < 7) {
                move_succeeded = perform_intra_move_greedy(intra_greedy_moves[choice]);
            } else {
                const int inter_idx = choice - 7;
                if (inter_idx < 8) {
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

bool LeaderCbma::neighbourhood_search(PartialSolution* partial_solution, const double& temperature) {
    partial_sol = partial_solution;

    const int choice = neigh_search_dist(thread_rng);
    constexpr int num_intra = 7;

    if (choice < num_intra) {
        return perform_intra_move_neigh(intra_neigh_moves[choice], temperature);
    }

    const int idx = choice - num_intra;
    return perform_inter_move_neigh(inter_neigh_moves[idx], temperature);
}


bool LeaderCbma::perform_intra_move_pert(const IntraFunc& move_func) const {
    if (num_routes < 1) return false;

    std::uniform_int_distribution dist(0, num_routes - 1);
    const int random_route_idx = dist(thread_rng);

    const bool is_moved = move_func(routes[random_route_idx], num_nodes_per_route[random_route_idx]);

    return is_moved;
}

bool LeaderCbma::perform_inter_move_pert(const InterFunc& move_func) {
    if (num_routes <= 1) return false;

    auto [r1, r2] = randomly_select_diff_route_pair();

    const bool is_moved = move_func(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],
                                    demand_sum_per_route[r1], demand_sum_per_route[r2]);

    if (is_moved) {
        clean_empty_routes(r1, r2);
    }

    return is_moved;
}

bool LeaderCbma::perform_inter_move_with_empty_pert(const InterFunc& move_func) {
    if (num_routes < 1) return false;

    std::uniform_int_distribution dist(0, num_routes - 1);
    const int r1 = dist(thread_rng);
    const int r2 = num_routes; // empty route

    const bool is_moved = move_func(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],
                                    demand_sum_per_route[r1], demand_sum_per_route[r2]);

    if (is_moved) {
        num_routes++;
    }

    return is_moved;
}

bool LeaderCbma::move1_intra_pert(int *route, const int length) {
    if (length <= 4) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> dist(1, length - 2);
    const int i = dist(thread_rng);
    int j;
    do {
        j = dist(thread_rng);
    } while (i == j);

    double original_cost, modified_cost;
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

    moveItoJ(route, i, j);
    upper_cost += change;
    has_moved = true;

    return has_moved;
}

bool LeaderCbma::move1_inter_pert(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    bool has_moved = false;

    std::vector<int> candidates;
    for (int i = 1; i < length1 - 1; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) <= instance->max_vehicle_capa_) {
            candidates.push_back(i);
        }
    }
    if (candidates.empty()) return false;
    std::shuffle(candidates.begin(), candidates.end(), thread_rng);  // Shuffle to pick one randomly
    const int i = candidates.front();  // Pick the first after shuffling
    std::uniform_int_distribution<int> dist(0, length2 - 2);
    const int j = dist(thread_rng);

    const double original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                                 instance->get_distance(route1[i], route1[i + 1]) +
                                 instance->get_distance(route2[j], route2[j + 1]);
    const double modified_cost = instance->get_distance(route1[i - 1], route1[i + 1]) +
                                 instance->get_distance(route2[j], route1[i]) +
                                 instance->get_distance(route1[i], route2[j + 1]);
    const double change = modified_cost - original_cost;

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

    return has_moved;
}

bool LeaderCbma::move1_inter_with_empty_route_pert(int *route1, int *route2, int &length1, int &length2, int &loading1,
                                                   int &loading2) {
    if (length1 < 4 || length2 != 0) return false; // make sure it won't generate empty route

    bool has_moved = false;

    std::uniform_int_distribution<int> dist(1, length1 - 2);
    const int i = dist(thread_rng);
    const int x = route1[i];

    const double original_cost = instance->get_distance(route1[i - 1], x) +
                                 instance->get_distance(x, route1[i + 1]);
    const double modified_cost = instance->get_distance(route1[i - 1], route1[i + 1]) +
                                 instance->get_distance(route2[0], x) +
                                 instance->get_distance(x, route2[1]);
    const double change = modified_cost - original_cost;

    for (int p = i; p < length1 - 1; p++) {
        route1[p] = route1[p + 1];
    }
    length1--;
    loading1 -= instance->get_customer_demand_(x);
    route2[1] = x;
    length2 = 3;
    loading2 += instance->get_customer_demand_(x);
    upper_cost += change;

    has_moved = true;

    return has_moved;
}

bool LeaderCbma::move2_intra_pert(int *route, const int length) {
    if (length <= 4) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> dist1(1, length - 3);
    std::uniform_int_distribution<int> dist2(0, length - 2);
    const int i = dist1(thread_rng);
    int j;
    do {
        j = dist2(thread_rng);
    } while (j == i - 1 || j == i || j == i + 1);


    double original_cost, modified_cost;
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
    const double change = modified_cost - original_cost;

    const int u = route[i];
    const int x = route[i + 1];
    if (i < j) {
        for (int k = i; k < j - 1; k++) {
            route[k] = route[k + 2];
        }
        route[j - 1] = u;
        route[j] = x;
    } else if (i > j) {
        for (int k = i + 1; k > j + 2; k--) {
            route[k] = route[k - 2];
        }
        route[j + 1] = u;
        route[j + 2] = x;
    }
    upper_cost += change;

    has_moved = true;

    return has_moved;
}

bool LeaderCbma::move2_inter_pert(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 4 || length2 < 3) return false;

    bool has_moved = false;

    std::vector<int> candidates;
    for (int i = 1; i < length1 - 2; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) + instance->get_customer_demand_(route1[ i + 1]) <= instance->max_vehicle_capa_) {
            candidates.push_back(i);
        }
    }
    if (candidates.empty()) return false;
    std::shuffle(candidates.begin(), candidates.end(), thread_rng);  // Shuffle to pick one randomly
    const int i = candidates.front();  // Pick the first after shuffling
    std::uniform_int_distribution<int> dist(0, length2 - 2);
    const int j = dist(thread_rng);

    const double original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                                 instance->get_distance(route1[i + 1], route1[i + 2]) +
                                 instance->get_distance(route2[j], route2[j + 1]);
    const double modified_cost = instance->get_distance(route1[i - 1], route1[i + 2]) +
                                 instance->get_distance(route2[j], route1[i]) +
                                 instance->get_distance(route1[i + 1], route2[j + 1]);
    const double change = modified_cost - original_cost;

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

    return has_moved;
}

bool LeaderCbma::move3_intra_pert(int *route, const int length) {
    if (length <= 4) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> dist(1, length - 3);
    const int i = dist(thread_rng);
    std::uniform_int_distribution<int> dist2(0, length - 2);
    int j;
    do {
        j = dist2(thread_rng);
    } while (j == i - 1 || j == i || j == i + 1);

    double original_cost, modified_cost;
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
    const double change = modified_cost - original_cost;

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

    return has_moved;
}

bool LeaderCbma::move3_inter_pert(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 4 || length2 < 3) return false;

    bool has_moved = false;

    std::vector<int> candidates;
    for (int i = 1; i < length1 - 2; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) +
            instance->get_customer_demand_(route1[ i + 1]) <= instance->max_vehicle_capa_) {
            candidates.push_back(i);
        }
    }
    if (candidates.empty()) return false;
    std::shuffle(candidates.begin(), candidates.end(), thread_rng);  // Shuffle to pick one randomly
    const int i = candidates.front();  // Pick the first after shuffling
    std::uniform_int_distribution<int> dist(0, length2 - 2);
    const int j = dist(thread_rng);

    const double original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                                 instance->get_distance(route1[i + 1], route1[i + 2]) +
                                 instance->get_distance(route2[j], route2[j + 1]);
    const double modified_cost = instance->get_distance(route1[i - 1], route1[i + 2]) +
                                 instance->get_distance(route2[j], route1[i + 1]) +
                                 instance->get_distance(route1[i], route2[j + 1]);
    const double change = modified_cost - original_cost;

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

    return has_moved;
}

bool LeaderCbma::move4_intra_pert(int *route, const int length) {
    if (length < 5) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length - 3);
    const int i = distI(thread_rng);
    std::uniform_int_distribution<int> distJ(i + 1, length - 2);
    const int j = distJ(thread_rng);

    double original_cost, modified_cost;
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
    const double change = modified_cost - original_cost;

    swap(route[i], route[j]);
    upper_cost += change;

    has_moved = true;

    return has_moved;
}

bool LeaderCbma::move4_inter_pert(int *route1, int *route2, const int length1, const int length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length1 - 2);
    const int i = distI(thread_rng);
    std::uniform_int_distribution<int> distJ(1, length2 - 2);
    const int j = distJ(thread_rng);

    const int demand_I = instance->get_customer_demand_(route1[i]);
    const int demand_J = instance->get_customer_demand_(route2[j]);
    if (loading1 - demand_I + demand_J <= instance->max_vehicle_capa_
        && loading2 - demand_J + demand_I <= instance->max_vehicle_capa_) {
        const double original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                                     instance->get_distance(route1[i], route1[i + 1]) +
                                     instance->get_distance(route2[j - 1], route2[j]) +
                                     instance->get_distance(route2[j], route2[j + 1]);
        const double modified_cost = instance->get_distance(route1[i - 1], route2[j]) +
                                     instance->get_distance(route2[j], route1[i + 1]) +
                                     instance->get_distance(route2[j - 1], route1[i]) +
                                     instance->get_distance(route1[i], route2[j + 1]);

        const double change = modified_cost - original_cost;

        swap(route1[i], route2[j]);
        loading1 = loading1 - demand_I + demand_J;
        loading2 = loading2 - demand_J + demand_I;
        upper_cost += change;

        has_moved = true;
    }

    return has_moved;
}

bool LeaderCbma::move5_intra_pert(int *route, const int length) {
    if (length < 5) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length - 4);
    const int i = distI(thread_rng);
    std::uniform_int_distribution<int> distJ(i + 2, length - 2);
    const int j = distJ(thread_rng);

    double original_cost, modified_cost;
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
    const double change = modified_cost - original_cost;

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

    return has_moved;
}

bool LeaderCbma::move5_inter_pert(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 4 || length2 < 4) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length1 - 3);
    const int i = distI(thread_rng);
    std::uniform_int_distribution<int> distJ(1, length2 - 2);
    const int j = distJ(thread_rng);

    const int demand_pair = instance->get_customer_demand_(route1[i]) + instance->get_customer_demand_(route1[i + 1]);
    const int demand_J = instance->get_customer_demand_(route2[j]);
    if (loading1 - demand_pair + demand_J > instance->max_vehicle_capa_ || loading2 - demand_J + demand_pair > instance->max_vehicle_capa_) {
        return false;
    }

    const double original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                                 instance->get_distance(route1[i + 1], route1[i + 2]) +
                                 instance->get_distance(route2[j - 1], route2[j]) +
                                 instance->get_distance(route2[j], route2[j + 1]);
    const double modified_cost = instance->get_distance(route1[i - 1], route2[j]) +
                                 instance->get_distance(route2[j], route1[i + 2]) +
                                 instance->get_distance(route2[j - 1], route1[i]) +
                                 instance->get_distance(route1[i + 1], route2[j + 1]);

    const double change = modified_cost - original_cost;

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

    return has_moved;
}

bool LeaderCbma::move6_intra_pert(int *route, const int length) {
    if (length < 6) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length - 5);
    const int i = distI(thread_rng);
    std::uniform_int_distribution<int> distJ(i + 2, length - 3);
    const int j = distJ(thread_rng);

    double original_cost, modified_cost;
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
    const double change = modified_cost - original_cost;

    swap(route[i], route[j]);
    swap(route[i + 1], route[j + 1]);
    upper_cost += change;

    has_moved = true;

    return has_moved;
}

bool LeaderCbma::move6_inter_pert(int *route1, int *route2, const int length1, const int length2, int &loading1, int &loading2) {
    if (length1 < 4 || length2 < 4 || (length1 == 4 && length2 == 4)) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length1 - 3);
    const int i = distI(thread_rng);
    std::uniform_int_distribution<int> distJ(1, length2 - 3);
    const int j = distJ(thread_rng);

    const int demand_I_pair = instance->get_customer_demand_(route1[i]) + instance->get_customer_demand_(route1[i + 1]);
    const int demand_J_pair = instance->get_customer_demand_(route2[j]) + instance->get_customer_demand_(route2[j + 1]);
    if (loading1 - demand_I_pair + demand_J_pair > instance->max_vehicle_capa_ || loading2 - demand_J_pair + demand_I_pair > instance->max_vehicle_capa_) {
        return false;
    }

    const double original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                                 instance->get_distance(route1[i + 1], route1[i + 2]) +
                                 instance->get_distance(route2[j - 1], route2[j]) +
                                 instance->get_distance(route2[j + 1], route2[j + 2]);
    const double modified_cost = instance->get_distance(route1[i - 1], route2[j]) +
                                 instance->get_distance(route2[j + 1], route1[i + 2]) +
                                 instance->get_distance(route2[j - 1], route1[i]) +
                                 instance->get_distance(route1[i + 1], route2[j + 2]);
    const double change = modified_cost - original_cost;

    swap(route1[i], route2[j]);
    swap(route1[i + 1], route2[j + 1]);
    loading1 = loading1 - demand_I_pair + demand_J_pair;
    loading2 = loading2 - demand_J_pair + demand_I_pair;
    upper_cost += change;

    has_moved = true;

    return has_moved;
}

bool LeaderCbma::move7_intra_pert(int *route, const int length) {
    if (length < 5) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length - 3);
    const int i = distI(thread_rng);
    std::uniform_int_distribution<int> distJ(i + 1, length - 2);
    const int j = distJ(thread_rng);

    const double original_cost = instance->get_distance(route[i - 1], route[i]) +
                                 instance->get_distance(route[j], route[j + 1]);
    const double modified_cost = instance->get_distance(route[i - 1], route[j]) +
                                 instance->get_distance(route[i], route[j + 1]);

    const double change = modified_cost - original_cost; // negative represents the cost reduction`

    reverse(route + i, route + j + 1);
    upper_cost += change;

    has_moved = true;

    return has_moved;
}

bool LeaderCbma::move8_inter_pert(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    memset(temp_r1, 0, sizeof(int) * node_cap);
    memset(temp_r2, 0, sizeof(int) * node_cap);

    bool has_moved = false;

    std::uniform_int_distribution<int> distN1(0, length1 - 2);
    const int n1 = distN1(thread_rng);
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int i = 0; i <= n1; i++) {
        partial_dem_r1 += instance->get_customer_demand_(route1[i]);
    }

    int partial_dem_r2 = 0;
    for (int n2 = 0; n2 < length2 - 1; ++n2) {
        if ((n1 == length1 - 2 && n2 == 0) || (n1 == 0 && n2 == length2 - 2)) continue; // the same as the current situation, just skip it.
        partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

        if (partial_dem_r1 + partial_dem_r2 > instance->max_vehicle_capa_ ||
            loading1 - partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_) continue;

        const double original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) +
                                     instance->get_distance(route2[n2], route2[n2 + 1]);
        const double modified_cost = instance->get_distance(route1[n1], route2[n2]) +
                                     instance->get_distance(route1[n1 + 1], route2[n2 + 1]);

        const double change = modified_cost - original_cost;

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
        break;
    }

    return has_moved;
}

bool LeaderCbma::move8_inter_with_empty_route_pert(int *route1, int *route2, int &length1, int &length2, int &loading1,
    int &loading2) {
    if (length1 < 4 || length2 != 0) return false;

    memset(temp_r1, 0, sizeof(int) * node_cap);
    memset(temp_r2, 0, sizeof(int) * node_cap);

    bool has_moved = false;

    std::uniform_int_distribution<int> distN1(1, length1 - 3);
    const int n1 = distN1(thread_rng);
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int i = 0; i <= n1; i++) {
        partial_dem_r1 += instance->get_customer_demand_(route1[i]);
    }


    constexpr int n2 = 0;
    constexpr int partial_dem_r2 = 0;

    const double original_cost = instance->get_distance(route1[n1], route1[n1 + 1]);
    const double modified_cost = instance->get_distance(route1[n1], route2[n2]) + instance->get_distance(route1[n1 + 1], route2[n2 + 1]);

    const double change = modified_cost - original_cost;

    length2 = 2; // two depots

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

    return has_moved;
}

bool LeaderCbma::move9_inter_pert(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    memset(temp_r1, 0, sizeof(int) * node_cap);

    bool has_moved = false;

    std::uniform_int_distribution<int> distN1(0, length1 - 2);
    const int n1 = distN1(thread_rng);
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int i = 0; i <= n1; i++) {
        partial_dem_r1 += instance->get_customer_demand_(route1[i]);
    }

    int partial_dem_r2 = 0;
    for (int n2 = 0; n2 < length2 - 1; ++n2) {
        if ((n1 == 0 && n2 == 0) || (n1 == length1 - 2 && n2 == length2 - 2)) continue;
        partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

        if (partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_ ||
            partial_dem_r2 + loading1 - partial_dem_r1 > instance->max_vehicle_capa_) continue;

        const double original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) +
                                     instance->get_distance(route2[n2], route2[n2 + 1]);
        const double modified_cost = instance->get_distance(route1[n1], route2[n2 + 1]) +
                                     instance->get_distance(route2[n2], route1[n1 + 1]);

        const double change = modified_cost - original_cost;


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
        break;
    }

    return has_moved;
}

bool LeaderCbma::move9_inter_with_empty_route_pert(int *route1, int *route2, int &length1, int &length2, int &loading1,
    int &loading2) {
    if (length1 < 4) return false;

    memset(temp_r1, 0, sizeof(int) * node_cap);

    bool has_moved = false;

    std::uniform_int_distribution<int> distN1(1, length1 - 3);
    const int n1 = distN1(thread_rng);
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int i = 0; i <= n1; i++) {
        partial_dem_r1 += instance->get_customer_demand_(route1[i]);
    }

    constexpr int n2 = 0;
    constexpr int partial_dem_r2 = 0;

    const double original_cost = instance->get_distance(route1[n1], route1[n1 + 1]);
    const double modified_cost = instance->get_distance(route1[n1], route2[n2 + 1]) +
                                 instance->get_distance(route2[n2], route1[n1 + 1]);

    const double change = modified_cost - original_cost;

    length2 = 2; // two depots

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

    return  has_moved;
}

bool LeaderCbma::perform_intra_move_neigh(const IntraImproFunc &move_func, const double temperature) const {
    if (num_routes < 1) return false;

    bool is_moved = false;

    std::uniform_int_distribution<int> dist(0, num_routes - 1);
    const int random_route_idx = dist(thread_rng);

    // bool is_improved;
    // do {
    //     is_improved = move_func(routes[random_route_idx], num_nodes_per_route[random_route_idx]);
    //     is_moved = is_moved || is_improved;
    // } while (is_improved);

    is_moved = move_func(routes[random_route_idx], num_nodes_per_route[random_route_idx], temperature);
    if (is_moved) {
        partial_sol->set_intra_route(random_route_idx, routes[random_route_idx],num_nodes_per_route[random_route_idx]);
        partial_sol->num_routes = num_routes;
    }

    return is_moved;
}

bool LeaderCbma::perform_inter_move_neigh(const InterImproFunc &move_func, const double temperature) {
    if (routes == nullptr || num_routes <= 1) return false;

    bool is_moved = false;

    int r2;
    std::uniform_int_distribution<int> dist(0, num_routes - 1);
    const int r1 = dist(thread_rng);
    do {
        r2 = dist(thread_rng);
    } while (r1 == r2);

    is_moved = move_func(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],
                        demand_sum_per_route[r1], demand_sum_per_route[r2], temperature);

    // bool is_improved;
    // do {
    //     is_improved = move_func(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2], demand_sum_per_route[r1], demand_sum_per_route[r2]);
    //     is_moved = is_improved || is_moved;
    //
    //     if (is_improved) {
    //         // clean the route beyond the length
    //         std::fill(routes[r1] + num_nodes_per_route[r1], routes[r1] + node_cap, 0);
    //         std::fill(routes[r2] + num_nodes_per_route[r2], routes[r2] + node_cap, 0);
    //
    //         if (demand_sum_per_route[r1] == 0 || demand_sum_per_route[r2] == 0) {
    //             int empty_r = demand_sum_per_route[r1] == 0 ? r1 : r2;
    //             // remove_if_empty(empty_r);
    //             clean_empty_routes(r1, r2 );
    //
    //             // Clear unused routes beyond num_routes
    //             std::fill(num_nodes_per_route + num_routes, num_nodes_per_route + route_cap, 0);
    //             std::fill(demand_sum_per_route + num_routes, demand_sum_per_route + route_cap, 0);
    //
    //             break;
    //         }
    //     }
    // } while (is_improved);

    if (is_moved) {
        // clean the route beyond the length
        partial_sol->set_inter_route(r1, routes[r1], num_nodes_per_route[r1], demand_sum_per_route[r1] == 0,
                                     r2, routes[r2], num_nodes_per_route[r2], demand_sum_per_route[r2] == 0);
        clean_empty_routes(r1, r2);
        partial_sol->num_routes = num_routes;
    }

    return is_moved;
}

bool LeaderCbma::perform_intra_move_greedy(const IntraImproFunc& move_func) const {
    if (num_routes < 1) return false;

    bool is_successful = false;

    for (int i = 0; i < num_routes; ++i) {
        bool is_improved;
        do {
            is_improved = move_func(routes[i], num_nodes_per_route[i], 0.0);
            is_successful = is_improved || is_successful;
        } while (is_improved);
    }

    return is_successful;
}

bool LeaderCbma::perform_inter_move_greedy(const InterImproFunc& move_func) {
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
            is_improved = move_func(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],
                                    demand_sum_per_route[r1], demand_sum_per_route[r2], 0.0);
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


bool LeaderCbma::move1_intra_impro(int* route, const int length, const AcceptanceFunc& accept_func, const double temperature) {
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
            if (accept_func(change, temperature)) {
                moveItoJ(route, i, j);
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }
    end_loops:;

    return has_moved;
}

bool LeaderCbma::move1_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2,
    const AcceptanceFunc& accept_func, const double temperature) {
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
            if (accept_func(change, temperature)) {
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

bool LeaderCbma::move2_intra_impro(int *route, const int length, const AcceptanceFunc& accept_func,
    const double temperature) {
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
            if (accept_func(change, temperature)) {
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

bool LeaderCbma::move4_intra_impro(int* route, const int length, const AcceptanceFunc& accept_func,
    const double temperature) {
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
            if (accept_func(change, temperature)) {
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

bool LeaderCbma::move2_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2,
    const AcceptanceFunc& accept_func, const double temperature) {
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
            if (accept_func(change, temperature)) {
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

bool LeaderCbma::move3_intra_impro(int *route, const int length, const AcceptanceFunc& accept_func,
    const double temperature) {
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
            if (accept_func(change, temperature)) {
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

bool LeaderCbma::move3_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2,
    const AcceptanceFunc& accept_func, const double temperature) {
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
            if (accept_func(change, temperature)) {
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

bool LeaderCbma::move4_inter_impro(int *route1, int *route2, const int length1, const int length2, int &loading1,
    int &loading2, const AcceptanceFunc& accept_func, const double temperature) {
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
                if (accept_func(change, temperature)) {
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

bool LeaderCbma::move5_intra_impro(int *route, const int length, const AcceptanceFunc& accept_func,
    const double temperature) {
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

            if (accept_func(change, temperature)) {
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

bool LeaderCbma::move5_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2,
    const AcceptanceFunc& accept_func, const double temperature) {
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
            if (accept_func(change, temperature)) {
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

bool LeaderCbma::move6_intra_impro(int *route, const int length, const AcceptanceFunc& accept_func,
    const double temperature) {
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

            if (accept_func(change, temperature)) {
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

bool LeaderCbma::move6_inter_impro(int *route1, int *route2, const int length1, const int length2, int &loading1,
    int &loading2, const AcceptanceFunc& accept_func, const double temperature) {
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

            if (accept_func(change, temperature)) {
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

bool LeaderCbma::move7_intra_impro(int *route, const int length, const AcceptanceFunc& accept_func,
    const double temperature) {
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
            if (accept_func(change, temperature)) {
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

bool LeaderCbma::move8_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2,
    const AcceptanceFunc& accept_func, const double temperature) {
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

            if (partial_dem_r1 + partial_dem_r2 > instance->max_vehicle_capa_ || loading1 - partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_) continue;

            original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) + instance->get_distance(route2[n2], route2[n2 + 1]);
            modified_cost = instance->get_distance(route1[n1], route2[n2]) + instance->get_distance(route1[n1 + 1], route2[n2 + 1]);

            change = modified_cost - original_cost;
            if (accept_func(change, temperature)) {
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

bool LeaderCbma::move9_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2,
    const AcceptanceFunc& accept_func, const double temperature) {
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
            if (accept_func(change, temperature)) {
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