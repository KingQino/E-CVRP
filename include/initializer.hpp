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
    vector<int> temp_x;
    vector<double> temp_vv;
    vector<int> temp_pp;

    vector<double> depot_dist;
public:
    Case* instance{};
    Preprocessor* preprocessor{};
    std::mt19937& random_engine;

    Initializer(std::mt19937& engine, Case* instance, Preprocessor* preprocessor);
    ~Initializer();

    [[nodiscard]] vector<vector<int>> prins_split(const vector<int>& chromosome) ;
    void prins_split(const vector<int>& chromosome, vector<vector<int>>& out_routes);
    [[nodiscard]] vector<vector<int>> hien_clustering() const;
    void hien_balancing(vector<vector<int>>& routes) const;
    [[nodiscard]] vector<vector<int>> routes_constructor_with_split();
    [[nodiscard]] vector<vector<int>> routes_constructor_with_hien_method() const;
    [[nodiscard]] vector<vector<int>> routes_constructor_with_direct_encoding() const;
};


#endif //INITIALIZER_HPP
