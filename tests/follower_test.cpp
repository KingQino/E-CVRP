//
// Created by Codex on 09/05/2026.
//

#include "gtest/gtest.h"
#include "follower.hpp"

#include <cmath>
#include <filesystem>
#include <memory>
#include <vector>

namespace fs = std::filesystem;

class FollowerTest : public ::testing::Test {
protected:
    void SetUp() override {
        const fs::path data_path = fs::path(__FILE__).parent_path().parent_path() / "data";
        instance = std::make_unique<Case>(data_path.string(), "RC108-10.evrp");
        preprocessor = std::make_unique<Preprocessor>(*instance, params);
        follower = std::make_unique<Follower>(instance.get(), preprocessor.get());
    }

    Parameters params;
    std::unique_ptr<Case> instance;
    std::unique_ptr<Preprocessor> preprocessor;
    std::unique_ptr<Follower> follower;
};

TEST_F(FollowerTest, RefineFallsBackToFeasibleChargingPlanWhenExactRangeHasNoSolution) {
    const std::vector<std::vector<int>> routes = {
        {0, 6, 8, 1, 10, 2, 3, 5, 7, 9, 4, 0}
    };
    const double upper_cost = instance->compute_total_distance(routes);
    const std::vector<int> demand_sums = instance->compute_demand_sum_per_route(routes);

    Individual standard(instance.get(), preprocessor.get(), routes, upper_cost, demand_sums);
    follower->run_with_variables(&standard);
    ASSERT_TRUE(std::isfinite(standard.lower_cost));
    ASSERT_GT(standard.lower_cost, 0.0);

    Individual refined(instance.get(), preprocessor.get(), routes, upper_cost, demand_sums);
    follower->refine(&refined);

    EXPECT_TRUE(std::isfinite(refined.lower_cost));
    EXPECT_GT(refined.lower_cost, 0.0);
    EXPECT_LE(refined.lower_cost, standard.lower_cost + EPSILON);
}
