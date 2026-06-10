//
// Created by Yinghao Qin on 08/06/2025.
//

#include "initializer.hpp"

#include <algorithm>
#include <limits>

using namespace std;

namespace {

struct GreedySeedPriority {
    int customer;
    double priority;

    bool operator<(const GreedySeedPriority& other) const {
        return priority > other.priority;
    }
};

} // namespace

Initializer::Initializer(std::mt19937& engine, Case *instance, Preprocessor *preprocessor)
    : instance(instance), preprocessor(preprocessor), random_engine(engine) {

    n = instance->num_customer_;
    temp_x.resize(n + 1);
    temp_vv.resize(n + 1);
    temp_pp.resize(n + 1);


    // cache the distance from depot to each customer node
    depot_dist.resize(n + 1, 0.0);
    for (int i = 1; i <= n; ++i) {
        // 'i' represents the customer id
        depot_dist[i] = instance->get_distance(instance->depot_, preprocessor->customer_ids_[i - 1]);
    }
}

Initializer::~Initializer() = default;

// Prins, C., 2004. A simple and effective evolutionary algorithm for the vehicle routing problem. Computers & operations research, 31(12), pp.1985-2002.
vector<vector<int>> Initializer::prins_split(const vector<int>& chromosome) {
    // giant tour starts from depot (node 0)
    temp_x[0] = 0;
    copy(chromosome.begin(), chromosome.end(), temp_x.begin() + 1);

    fill(temp_vv.begin(), temp_vv.begin() + n + 1, numeric_limits<double>::max());
    fill(temp_pp.begin(), temp_pp.begin() + n + 1, 0);
    temp_vv[0] = 0.0;

    const int max_capa = instance->max_vehicle_capa_;

    // dynamic programming to find the shortest path
    for (int i = 1; i <= n; ++i) {
        int load = 0;
        double cost = depot_dist[temp_x[i]] * 2;  // initial cost = 2 by dist(depot, x[1])
        for (int j = i; j <= n; ++j) {
            load += instance->get_customer_demand_(temp_x[j]);
            if (load > max_capa) break;

            if (j > i) {
                const int from = temp_x[j - 1];
                const int to = temp_x[j];
                cost -= depot_dist[from];
                cost += instance->get_distance(from, to);
                cost += depot_dist[to];
            }

            if (temp_vv[i - 1] + cost < temp_vv[j]) {
                temp_vv[j] = temp_vv[i - 1] + cost;
                temp_pp[j] = i - 1;
            }
        }
    }

    vector<vector<int>> all_routes;
    int j = n;
    while (j > 0) {
        const int i = temp_pp[j];
        all_routes.emplace_back(temp_x.begin() + i + 1, temp_x.begin() + j + 1);
        j = i;
    }

    return all_routes;
}

void Initializer::prins_split(const vector<int> &chromosome, vector<vector<int>> &out_routes) {
    // giant tour starts from depot (node 0)
    temp_x[0] = 0;
    copy(chromosome.begin(), chromosome.end(), temp_x.begin() + 1);

    fill(temp_vv.begin(), temp_vv.begin() + n + 1, numeric_limits<double>::max());
    fill(temp_pp.begin(), temp_pp.begin() + n + 1, 0);
    temp_vv[0] = 0.0;

    const int max_capa = instance->max_vehicle_capa_;

    // dynamic programming to find the shortest path
    for (int i = 1; i <= n; ++i) {
        int load = 0;
        double cost = depot_dist[temp_x[i]] * 2;  // initial cost = 2 by dist(depot, x[1])
        for (int j = i; j <= n; ++j) {
            load += instance->get_customer_demand_(temp_x[j]);
            if (load > max_capa) break;

            if (j > i) {
                const int from = temp_x[j - 1];
                const int to = temp_x[j];
                cost -= depot_dist[from];
                cost += instance->get_distance(from, to);
                cost += depot_dist[to];
            }

            if (temp_vv[i - 1] + cost < temp_vv[j]) {
                temp_vv[j] = temp_vv[i - 1] + cost;
                temp_pp[j] = i - 1;
            }
        }
    }

    out_routes.clear();
    int j = n;
    while (j > 0) {
        const int i = temp_pp[j];
        auto first = temp_x.begin() + i + 1;
        auto last = temp_x.begin() + j + 1;

        out_routes.emplace_back(first, last);
        out_routes.back().insert(out_routes.back().begin(), 0);  // add depot to the beginning
        out_routes.back().push_back(0);                          // add depot to the end
        j = i;
    }
}

// Hien et al., "A greedy search based evolutionary algorithm for electric vehicle routing problem", 2023.
vector<vector<int>> Initializer::hien_clustering() const {
    vector<int> chromosome = preprocessor->customer_ids_;

    // Clustering
    std::shuffle(chromosome.begin(),chromosome.end(), random_engine);
    vector<vector<int>> routes;
    vector<int> route;
    while (!chromosome.empty()) {
        route.clear();

        int anchor = chromosome.front();
        chromosome.erase(chromosome.begin());
        route.push_back(anchor);

        int cap = instance->get_customer_demand_(anchor);
        const auto& nearby_customers = preprocessor->sorted_nearby_customers_[anchor];

        for (int node : nearby_customers) {
            auto it = find(chromosome.begin(), chromosome.end(), node);
            if (it == chromosome.end()) continue;

            if (const int demand = instance->get_customer_demand_(node); cap + demand <= instance->max_vehicle_capa_) {
                cap += demand;
                route.push_back(node);
                chromosome.erase(it);
            }

            if (cap >= instance->max_vehicle_capa_) {
                break;
            }
        }

        routes.push_back(route);
    }

    return routes;
}

void Initializer::hien_balancing(vector<vector<int>>& routes) const {
    vector<int>& last_route = routes.back();

    uniform_int_distribution<int> distribution(0, static_cast<int>(last_route.size() - 1));
    const int customer = last_route[distribution(random_engine)];  // Randomly choose a customer from the last route

    int cap1 = 0;
    for (const int node : last_route) {
        cap1 += instance->get_customer_demand_(node);
    }
    const int size = instance->num_customer_ - 1; // the size of nearby_customers

    for (int i = 0; i < size; ++i) {
        int x = preprocessor->sorted_nearby_customers_[customer][i];
        if (find(last_route.begin(), last_route.end(), x) != last_route.end()) {
            continue;
        }
        auto route2It = find_if(routes.begin(), routes.end(), [x](const vector<int>& route) {
            return find(route.begin(), route.end(), x) != route.end();
        });

        if (route2It != routes.end()) {
            vector<int>& route2 = *route2It;
            int cap2 = 0;
            for (const int node : route2) {
                cap2 += instance->get_customer_demand_(node);
            }

            if (const int demand_X = instance->get_customer_demand_(x); demand_X + cap1 <= instance->max_vehicle_capa_
                && abs((cap1 + demand_X) - (cap2 - demand_X)) < abs(cap1 - cap2)) {
                route2.erase(remove(route2.begin(), route2.end(), x), route2.end());
                last_route.push_back(x);
                cap1 += demand_X;
            } else {
                break;
            }
        }
    }
}

vector<vector<int>> Initializer::routes_constructor_with_split() {
    vector<int> a_giant_tour(preprocessor->customer_ids_);

    shuffle(a_giant_tour.begin(), a_giant_tour.end(), random_engine);

    vector<vector<int>> all_routes = prins_split(a_giant_tour);
    for (auto& route : all_routes) {
        route.insert(route.begin(), instance->depot_);
        route.push_back(instance->depot_);
    }

    return all_routes;
}

vector<vector<int>> Initializer::routes_constructor_with_greedy_seq(const bool random_seed_customers) const {
    vector<int> unassigned(preprocessor->customer_ids_);
    vector<char> is_unassigned(instance->problem_size_, 0);
    for (const int customer : unassigned) {
        is_unassigned[customer] = 1;
    }

    vector<GreedySeedPriority> seed_order;
    seed_order.reserve(unassigned.size());
    for (const int customer : unassigned) {
        const double priority = random_seed_customers
            ? std::uniform_real_distribution<>(0.0, 10.0)(random_engine)
            : instance->euclidean_distance(instance->depot_, customer) + instance->euclidean_distance(customer, instance->depot_);
        seed_order.push_back({customer, priority});
    }
    std::sort(seed_order.begin(), seed_order.end());

    auto erase_unassigned = [&](const int customer) {
        is_unassigned[customer] = 0;
        unassigned.erase(std::remove(unassigned.begin(), unassigned.end(), customer), unassigned.end());
    };

    auto select_next_seed = [&]() {
        for (const auto& item : seed_order) {
            if (is_unassigned[item.customer]) {
                return item.customer;
            }
        }
        return -1;
    };

    auto best_insertion_for_route = [&](const vector<int>& route, const int customer, int& best_pos, double& best_delta) {
        best_pos = -1;
        best_delta = std::numeric_limits<double>::max();
        for (int pos = 1; pos < static_cast<int>(route.size()); ++pos) {
            const int prev = route[pos - 1];
            const int next = route[pos];
            const double delta = instance->get_distance(prev, customer) +
                                 instance->get_distance(customer, next) -
                                 instance->get_distance(prev, next);
            if (delta < best_delta) {
                best_delta = delta;
                best_pos = pos;
            }
        }
        return best_pos != -1;
    };

    vector<vector<int>> routes;
    routes.reserve(std::min(instance->num_customer_, preprocessor->route_cap_));

    while (!unassigned.empty()) {
        const int seed_customer = select_next_seed();
        if (seed_customer < 0) {
            break;
        }

        vector<int> route = {instance->depot_, seed_customer, instance->depot_};
        int route_load = instance->get_customer_demand_(seed_customer);
        erase_unassigned(seed_customer);

        while (!unassigned.empty()) {
            int best_customer = -1;
            int best_pos = -1;
            double best_delta = std::numeric_limits<double>::max();

            for (const int customer : unassigned) {
                if (route_load + instance->get_customer_demand_(customer) > instance->max_vehicle_capa_) {
                    continue;
                }

                int insert_pos = -1;
                double route_delta = std::numeric_limits<double>::max();
                if (!best_insertion_for_route(route, customer, insert_pos, route_delta)) {
                    continue;
                }

                if (route_delta < best_delta) {
                    best_delta = route_delta;
                    best_pos = insert_pos;
                    best_customer = customer;
                }
            }

            if (best_customer < 0) {
                break;
            }

            route.insert(route.begin() + best_pos, best_customer);
            route_load += instance->get_customer_demand_(best_customer);
            erase_unassigned(best_customer);
        }

        routes.push_back(std::move(route));
    }

    return routes;
}

vector<vector<int>> Initializer::routes_constructor_with_hien_method() const{
    vector<vector<int>> routes = hien_clustering();
    hien_balancing(routes);

    for (auto& route : routes) {
        route.insert(route.begin(), instance->depot_);
        route.push_back(instance->depot_);
    }

    return routes;
}

// Jia Ya-Hui, et al., "Confidence-Based Ant Colony Optimization for Capacitated Electric Vehicle Routing Problem With Comparison of Different Encoding Schemes", 2022
vector<vector<int>> Initializer::routes_constructor_with_direct_encoding() const {
    vector<int> customers_(preprocessor->customer_ids_);

    int vehicle_idx = 0; // vehicle index - starts from the vehicle 0
    int load_of_one_route = 0; // the load of the current vehicle
    vector<int> route = {instance->depot_}; // the first route starts from depot_ 0

    vector<vector<int>> all_routes;
    while(!customers_.empty()) {
        vector<int> all_temp;
        for(int i : customers_) {
            if(instance->get_customer_demand_(i) <= instance->max_vehicle_capa_ - load_of_one_route) {
                all_temp.push_back(i);
            }
        }

        const int remain_total_demand_ = accumulate(customers_.begin(), customers_.end(), 0,
            [&](const int total, const int i) {
            return total + instance->get_customer_demand_(i);
        });
        if(remain_total_demand_ <= instance->max_vehicle_capa_ * (instance->num_vehicle_ - vehicle_idx - 1) || all_temp.empty()) {
            all_temp.push_back(instance->depot_); // add depot_ node into the all_temp
        }

        const int cur = route.back();
        uniform_int_distribution<> distribution(0, static_cast<int>(all_temp.size()) - 1);
        int next = all_temp[distribution(random_engine)]; // int next = roulette_wheel_selection(all_temp, cur);
        route.push_back(next);

        if (next == instance->depot_) {
            if (cur == instance->depot_) continue; // fix-bug
            all_routes.push_back(route);
            vehicle_idx += 1;
            route = {0};
            load_of_one_route = 0;
        } else {
            load_of_one_route += instance->get_customer_demand_(next);
            customers_.erase(remove(customers_.begin(), customers_.end(), next), customers_.end());
        }
    }

    route.push_back(instance->depot_);
    all_routes.push_back(route);

    return all_routes;
}
