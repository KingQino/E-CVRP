//
// Created by Codex on 09/05/2026.
//

#include "gtest/gtest.h"
#include "follower.hpp"

#include <chrono>
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

// R202-15 has one vehicle, so the whole solution is one long route. For this route no placement with
// floor/ceil(length / range) = 4 or 5 stations is feasible, and the former exhaustive enumeration had to
// go up to 9 stations (about 30 minutes). The dynamic program finds the optimum (7 stations) at once.
TEST(FollowerRefineTest, LongSingleRouteIsRefinedOptimallyAndQuickly) {
    const fs::path data_path = fs::path(__FILE__).parent_path().parent_path() / "data";
    Parameters params;
    Case instance(data_path.string(), "R202-15.evrp");
    Preprocessor preprocessor(instance, params);
    Follower follower(&instance, &preprocessor);

    const std::vector<std::vector<int>> routes = {
        {0, 12, 5, 2, 3, 13, 11, 1, 9, 8, 10, 14, 7, 4, 6, 15, 0}
    };
    Individual refined(&instance, &preprocessor, routes, instance.compute_total_distance(routes),
                       instance.compute_demand_sum_per_route(routes));

    const auto start = std::chrono::steady_clock::now();
    follower.refine(&refined);
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    EXPECT_NEAR(refined.lower_cost, 313.6035, 1e-3);
    EXPECT_LT(seconds, 1.0);

    // The refined route keeps the customer order and every piece between charging points is within range.
    const int length = follower.lower_num_nodes_per_route[0];
    const int* route = follower.lower_routes[0];
    std::vector<int> customers;
    int stations = 0;
    double piece = 0.0;
    double total = 0.0;
    for (int i = 0; i < length; ++i) {
        if (i > 0) {
            const double step = instance.get_distance(route[i - 1], route[i]);
            piece += step;
            total += step;
            EXPECT_LE(piece, preprocessor.max_cruise_distance_ + 1e-9);
        }
        if (instance.is_charging_station(route[i]) && i > 0 && i < length - 1) {
            ++stations;
            piece = 0.0;
        } else if (route[i] != instance.depot_) {
            customers.push_back(route[i]);
        }
    }
    EXPECT_EQ(stations, 7);
    EXPECT_NEAR(total, refined.lower_cost, 1e-6);
    EXPECT_EQ(customers, std::vector<int>(routes[0].begin() + 1, routes[0].end() - 1));
}
