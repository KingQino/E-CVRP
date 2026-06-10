//
// Created by Yinghao Qin on 08/06/2025.
//

#ifndef INITIALIZER_HPP
#define INITIALIZER_HPP

#include "case.hpp"
#include "preprocessor.hpp"

class Initializer {
private:
    int n; // the number of customers, i.e., the size of the chromosome
    std::vector<int> temp_x;
    std::vector<double> temp_vv;
    std::vector<int> temp_pp;

    std::vector<double> depot_dist;
public:
    Case* instance{};
    Preprocessor* preprocessor{};
    std::mt19937& random_engine;

    Initializer(std::mt19937& engine, Case* instance, Preprocessor* preprocessor);
    ~Initializer();

    [[nodiscard]] std::vector<std::vector<int>> prins_split(const std::vector<int>& chromosome) ;
    void prins_split(const std::vector<int>& chromosome, std::vector<std::vector<int>>& out_routes);
    [[nodiscard]] std::vector<std::vector<int>> hien_clustering() const;
    void hien_balancing(std::vector<std::vector<int>>& routes) const;
    [[nodiscard]] std::vector<std::vector<int>> routes_constructor_with_split();
    [[nodiscard]] std::vector<std::vector<int>> routes_constructor_with_greedy_seq(bool random_seed_customers = true) const;
    [[nodiscard]] std::vector<std::vector<int>> routes_constructor_with_hien_method() const;
    [[nodiscard]] std::vector<std::vector<int>> routes_constructor_with_direct_encoding() const;
};


#endif //INITIALIZER_HPP
