//
// Created by Yinghao Qin on 08/06/2025.
//

#include "preprocessor.hpp"

const int Preprocessor::MAX_EVALUATION_FACTOR = 25'000;

Preprocessor::Preprocessor(const Case &c, const Parameters &params) : c(c), params(params) {

    // Stop criteria setting up
    this->max_evals_ = c.problem_size_ * MAX_EVALUATION_FACTOR;
    if (c.num_customer_ <= 100) {
        max_exec_time_ = static_cast<int>(1 * (c.problem_size_ / 100.0) * 60 * 60);
    } else if (c.num_customer_ <= 915) {
        max_exec_time_ = static_cast<int>(2 * (c.problem_size_ / 100.0) * 60 * 60);
    } else {
        max_exec_time_ = static_cast<int>(3 * (c.problem_size_ / 100.0) * 60 * 60);
    }
    this->max_no_improvement_count_ = 800; // adjust further  default: 20,000

    this->max_demand_ = *std::max_element(c.demand_.begin(), c.demand_.end());
    this->total_demand_ = std::accumulate(c.demand_.begin(), c.demand_.end(), 0);
    this->route_cap_ =  10 * c.num_vehicle_;
    this->node_cap_ = c.num_customer_ + 2; // for the boarder condition, for example E-n23-k3, a route almost contains all the customers
    this->max_cruise_distance_ = c.max_battery_capa_ / c.energy_consumption_rate_;
    for (int i = 0; i < c.problem_size_; i++) {
        for (int j = 0; j < c.problem_size_; j++) {
            if (c.distances_[i][j] > max_distance_) max_distance_ = c.distances_[i][j];
        }
    }

    for (int i = c.num_depot_ + c.num_customer_; i < c.problem_size_; ++i) {
        station_ids_.push_back(i);
    }

    this->sorted_nearby_customers_ = vector<vector<int>>(c.num_customer_ + 1);
    for (int i = 1; i <= c.num_customer_; i++) {
        for (auto node : customer_ids_) {
            if (node == i) continue;
            sorted_nearby_customers_[i].push_back(node);
        }

        sort(sorted_nearby_customers_[i].begin(), sorted_nearby_customers_[i].end(), [&](const int a, const int b) {
            return c.distances_[i][a] < c.distances_[i][b];
        });
    }

    // Make charging decision, filling the vector with correlated vertices
    this->best_station_ = std::vector(c.num_depot_ + c.num_customer_,std::vector<int>(c.num_depot_ + c.num_customer_));
    for (int i = 0; i < c.num_depot_ + c.num_customer_ - 1; i++) {
        for (int j = i + 1; j < c.num_depot_ + c.num_customer_; j++) {
            this->best_station_[i][j] = this->best_station_[j][i] = get_best_station(i, j);
        }
    }
}

int Preprocessor::get_best_station(const int from, const int to) const {
    int target_station = -1;
    double min_dis = std::numeric_limits<double>::max();

    for (int i = c.num_customer_ + 1 ; i < c.problem_size_; ++i) {
        if (const double dis = c.distances_[from][i] + c.distances_[to][i]; min_dis > dis && from != i && to != i) {
            target_station = i;
            min_dis = dis;
        }
    }

    return target_station;
}

int Preprocessor::get_best_and_feasible_station(const int from, const int to, const double max_dis) const {
    int target_station = -1;
    double min_dis = std::numeric_limits<double>::max();

    for (int i = c.num_customer_ + 1; i < c.problem_size_; ++i) {
        if (c.distances_[from][i] < max_dis &&
            min_dis > c.distances_[from][i]  + c.distances_[to][i]  &&
            from != i && to != i &&
            c.distances_[i][to] < max_cruise_distance_) {

            target_station = i;
            min_dis = c.distances_[from][i] + c.distances_[to][i];
        }
    }

    return target_station;
}