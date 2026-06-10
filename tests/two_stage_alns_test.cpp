//
// Created by Codex on 26/04/2026.
//

#include "gtest/gtest.h"
#include "two_stage_alns.hpp"

#include <cmath>
#include <filesystem>
#include <memory>
#include <vector>

namespace fs = std::filesystem;

namespace {

Parameters make_two_stage_params() {
    Parameters params;
    params.algorithm = Algorithm::TWOSTAGEALNS;
    params.enable_logging = false;
    params.enable_final_refinement = false;
    params.stop_criteria = 0;
    params.seed = 11;
    return params;
}

void expect_valid_customer_assignment(const Individual& ind, const Case& instance) {
    std::vector<int> visit_count(instance.num_customer_ + 1, 0);

    for (int route_idx = 0; route_idx < ind.num_routes; ++route_idx) {
        ASSERT_GE(ind.num_nodes_per_route[route_idx], 2);
        EXPECT_EQ(ind.routes[route_idx][0], instance.depot_);
        EXPECT_EQ(ind.routes[route_idx][ind.num_nodes_per_route[route_idx] - 1], instance.depot_);

        for (int pos = 1; pos < ind.num_nodes_per_route[route_idx] - 1; ++pos) {
            const int node = ind.routes[route_idx][pos];
            if (node >= 1 && node <= instance.num_customer_) {
                ++visit_count[node];
            }
        }
    }

    for (int customer = 1; customer <= instance.num_customer_; ++customer) {
        EXPECT_EQ(visit_count[customer], 1) << "Customer " << customer << " should appear exactly once";
    }
}

} // namespace

class TwoStageAlnsTest : public ::testing::Test {
protected:
    void SetUp() override {
        const fs::path data_path = fs::path(__FILE__).parent_path().parent_path() / "data";
        instance = std::make_unique<Case>(data_path.string(), "E-n29-k4-s7.evrp");
    }

    std::unique_ptr<Preprocessor> make_preprocessor(const Parameters& params, const double max_evals = 4000.0) {
        auto prep = std::make_unique<Preprocessor>(*instance, params);
        prep->max_evals_ = max_evals;
        return prep;
    }

    std::unique_ptr<Case> instance;
};

TEST_F(TwoStageAlnsTest, RunBuildsFiniteUpperAndLowerIncumbents) {
    const Parameters params = make_two_stage_params();
    auto preprocessor = make_preprocessor(params, 3500.0);
    TwoStageAlns search(params.seed, instance.get(), preprocessor.get());

    const double final_cost = search.run();

    EXPECT_GT(search.alns_iter, 0);
    EXPECT_TRUE(std::isfinite(final_cost));
    EXPECT_TRUE(std::isfinite(search.global_best_upper->upper_cost));
    EXPECT_TRUE(std::isfinite(search.global_best_lower->lower_cost));
    EXPECT_LE(search.global_best_lower->upper_cost, search.global_best_upper->upper_cost + EPSILON);
    expect_valid_customer_assignment(*search.global_best_upper, *instance);
}

TEST_F(TwoStageAlnsTest, InitializeHeuristicBuildsValidGreedySequentialSeed) {
    const Parameters params = make_two_stage_params();
    auto preprocessor = make_preprocessor(params, 3500.0);
    TwoStageAlns search(params.seed, instance.get(), preprocessor.get());

    search.initialize_heuristic();

    EXPECT_GT(search.current->num_routes, 0);
    EXPECT_TRUE(std::isfinite(search.current->upper_cost));
    EXPECT_DOUBLE_EQ(search.global_best_upper->upper_cost, search.current->upper_cost);
    expect_valid_customer_assignment(*search.current, *instance);
}
