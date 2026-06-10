//
// Created by Yinghao Qin on 08/06/2025.
//

#ifndef CASE_HPP
#define CASE_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <set>
#include <cstring>
#include <string>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <numeric>
#include <filesystem>


class Case {
public:
    explicit Case(const std::string& kDataPath, const std::string& file_name);
    ~Case();
    void read_problem(const std::filesystem::path& file_path);                          // reads .evrp file
    static double **generate_2D_matrix_double(int n, int m);                            // generate a 2D matrix of double
    [[nodiscard]] bool is_charging_station(int node) const;					            // returns true if node is a charging station
    [[nodiscard]] double euclidean_distance(int i, int j) const;                        // calculate the Euclidean distance between two nodes
    [[nodiscard]] double calculate_total_dist(const std::vector<std::vector<int>>& chromR) const; // return the total distance of the upper solution
    [[nodiscard]] double calculate_total_dist_follower(int** routes, int num_routes, const int* num_nodes_per_route) const;
    [[nodiscard]] int calculate_demand_sum(const std::vector<int>& route) const;             // return the demand sum of the given route.
    [[nodiscard]] double compute_total_distance(const std::vector<std::vector<int>>& routes);     // return the total distance of the given routes
    [[nodiscard]] double compute_total_distance(const std::vector<int>& route) const;        // return the total distance of the given route
    [[nodiscard]] std::vector<int> compute_demand_sum_per_route(const std::vector<std::vector<int>>& routes) const;
    [[nodiscard]] inline int get_customer_demand_(const int customer) const { return demand_[customer]; } // returns the customer demand
    [[nodiscard]] inline double get_evals() const { return evals_; }		// returns the number of evaluations
    [[nodiscard]] inline double get_distance(const int from, const int to) {			// returns the distance
        // adds partial evaluation to the overall fitness evaluation count
        // It can be used when local search is used and a whole evaluation is not necessary
        evals_ += eval_increment_;
        return distances_[from][to];
    }

    std::string file_name_;
    std::string instance_name_;

    int depot_{};                           // depot id (usually 0)
    int num_depot_{};                       // number of depots, 1
    int num_customer_{};                    // number of customers
    int num_station_{};                     // number of charging stations
    int num_vehicle_{};                     // number of available vehicles
    int dimension_{};                       // dimension of the problem, i.e., not accurate
    int problem_size_{};                    // Total number of customers, charging stations and depot
    int max_vehicle_capa_{};                // maximum capacity of the vehicle
    double max_battery_capa_{};             // maximum energy capacity of the vehicle
    double energy_consumption_rate_{};      // energy consumption rate
    double optimum_{};                      // known optimum value of the problem, if available
    double** distances_{};                  // distance matrix
    double evals_{};                        // number of evaluations used
    double eval_increment_{};               // cached increment for get_distance() eval accounting
    std::vector<int> demand_;                    // size = num_customer_ + 1
    std::vector<std::pair<double, double>> positions_;// coordinates of the nodes

    friend std::ostream& operator<<(std::ostream& os, const Case& c);
};

#endif //CASE_HPP
