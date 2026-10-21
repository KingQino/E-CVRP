//
// Created by Codex on 09/05/2026.
//

#include "gtest/gtest.h"
#include "follower.hpp"

#include <chrono>
#include <random>
#include <algorithm>
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

namespace {

// Checks that a charged route keeps the customer order, stays within range between charging points
// and costs what it claims. Returns the number of inserted stations.
int check_charged_route(Case& instance, const Preprocessor& preprocessor, const Follower& follower, const int route_idx,
                        const std::vector<int>& upper_route, const double claimed_cost) {
    const int length = follower.lower_num_nodes_per_route[route_idx];
    const int* route = follower.lower_routes[route_idx];
    std::vector<int> visited;
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
        if (i > 0 && i < length - 1 && instance.is_charging_station(route[i])) {
            ++stations;
            piece = 0.0;
        } else {
            visited.push_back(route[i]);
        }
    }
    EXPECT_EQ(visited, upper_route);
    EXPECT_NEAR(total, claimed_cost, 1e-6);
    return stations;
}

} // namespace

// R202-15 has one vehicle, so the whole solution is one long route. No placement with
// floor/ceil(length / range) = 4 or 5 stations is feasible; the former exhaustive enumeration had to
// go up to 9 stations (about 30 minutes). The shortest-path DP finds the optimum (7 stations) at once.
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

    EXPECT_EQ(check_charged_route(instance, preprocessor, follower, 0, routes[0], refined.lower_cost), 7);
}

// RC204-15: floor/ceil(length / range) = 3 or 4 stations are feasible but expensive (best 295.18).
// The former rule stopped there; the optimum uses 5 stations (285.13), which the standard follower
// also finds, so the final refinement used to make the reported result worse.
TEST(FollowerRefineTest, RefineConsidersMoreStationsThanTheLengthBound) {
    const fs::path data_path = fs::path(__FILE__).parent_path().parent_path() / "data";
    Parameters params;
    Case instance(data_path.string(), "RC204-15.evrp");
    Preprocessor preprocessor(instance, params);
    Follower follower(&instance, &preprocessor);

    const std::vector<std::vector<int>> routes = {{0, 9, 12, 6, 7, 4, 5, 11, 10, 14, 3, 15, 13, 2, 1, 8, 0}};
    Individual standard(&instance, &preprocessor, routes, instance.compute_total_distance(routes),
                        instance.compute_demand_sum_per_route(routes));
    follower.run(&standard);

    Individual refined(&instance, &preprocessor, routes, instance.compute_total_distance(routes),
                       instance.compute_demand_sum_per_route(routes));
    follower.refine(&refined);

    EXPECT_NEAR(refined.lower_cost, 285.1320, 1e-3);
    EXPECT_LE(refined.lower_cost, standard.lower_cost + 1e-9);
    EXPECT_EQ(check_charged_route(instance, preprocessor, follower, 0, routes[0], refined.lower_cost), 5);
}

// On random routes the refinement is feasible whenever the standard follower is, never more
// expensive, and always returns a valid charged route (several stations in a row are allowed).
TEST(FollowerRefineTest, RefineIsNeverWorseThanTheStandardFollower) {
    const fs::path data_path = fs::path(__FILE__).parent_path().parent_path() / "data";
    for (const char* name : {"RC204-15.evrp", "R202-15.evrp", "RC108-10.evrp", "E-n22-k4.evrp"}) {
        Parameters params;
        Case instance(data_path.string(), name);
        Preprocessor preprocessor(instance, params);
        Follower follower(&instance, &preprocessor);
        std::mt19937 engine(42);
        std::vector<int> customers = preprocessor.customer_ids_;

        for (int trial = 0; trial < 200; ++trial) {
            std::shuffle(customers.begin(), customers.end(), engine);
            const int size = 1 + static_cast<int>(engine() % customers.size());
            std::vector<int> route{instance.depot_};
            route.insert(route.end(), customers.begin(), customers.begin() + size);
            route.push_back(instance.depot_);
            const std::vector<std::vector<int>> routes{route};

            Individual standard(&instance, &preprocessor, routes, instance.compute_total_distance(routes),
                                instance.compute_demand_sum_per_route(routes));
            follower.run(&standard);
            Individual refined(&instance, &preprocessor, routes, instance.compute_total_distance(routes),
                               instance.compute_demand_sum_per_route(routes));
            follower.refine(&refined);

            if (standard.lower_cost < INFEASIBLE) {
                ASSERT_LT(refined.lower_cost, INFEASIBLE) << name << " trial " << trial;
                EXPECT_LE(refined.lower_cost, standard.lower_cost + 1e-9) << name << " trial " << trial;
            }
            if (refined.lower_cost < INFEASIBLE) {
                check_charged_route(instance, preprocessor, follower, 0, route, refined.lower_cost);
            }
        }
    }
}

// R103-10: the tour is 143.86 long with range 60.63, so l_min = 2 and l_min + 1 = 3 stations.
// Two stations are feasible (170.22) but three are much cheaper (160.41, the instance optimum).
// With se_compare_station_counts (MA), SE evaluates both counts.
TEST(FollowerRunTest, SimpleEnumerationComparesFloorAndFloorPlusOneStations) {
    const fs::path data_path = fs::path(__FILE__).parent_path().parent_path() / "data";
    Parameters params;
    Case instance(data_path.string(), "R103-10.evrp");
    Preprocessor preprocessor(instance, params);
    Follower follower(&instance, &preprocessor);
    follower.se_compare_station_counts = true;

    const std::vector<std::vector<int>> routes = {{0, 4, 7, 3, 6, 10, 1, 9, 5, 8, 2, 0}};
    const double upper_cost = instance.compute_total_distance(routes);
    const std::vector<int> demand_sums = instance.compute_demand_sum_per_route(routes);

    Individual evaluated(&instance, &preprocessor, routes, upper_cost, demand_sums);
    follower.run(&evaluated);                       // evaluate-only path used during the search
    EXPECT_NEAR(evaluated.lower_cost, 160.41224, 1e-4);

    Individual with_routes(&instance, &preprocessor, routes, upper_cost, demand_sums);
    follower.run_with_variables(&with_routes);      // path that also builds the charged route
    EXPECT_NEAR(with_routes.lower_cost, 160.41224, 1e-4);
    EXPECT_EQ(check_charged_route(instance, preprocessor, follower, 0, routes[0], with_routes.lower_cost), 3);
}

// Default (LAHC, TwoStageAlns): the original SE stops at the first feasible station count, so on the same
// R103-10 tour it returns the 2-station placement (170.22).
TEST(FollowerRunTest, SimpleEnumerationStopsAtFirstFeasibleCountByDefault) {
    const fs::path data_path = fs::path(__FILE__).parent_path().parent_path() / "data";
    Parameters params;
    Case instance(data_path.string(), "R103-10.evrp");
    Preprocessor preprocessor(instance, params);
    Follower follower(&instance, &preprocessor);
    ASSERT_FALSE(follower.se_compare_station_counts);

    const std::vector<std::vector<int>> routes = {{0, 4, 7, 3, 6, 10, 1, 9, 5, 8, 2, 0}};
    const double upper_cost = instance.compute_total_distance(routes);
    const std::vector<int> demand_sums = instance.compute_demand_sum_per_route(routes);

    Individual evaluated(&instance, &preprocessor, routes, upper_cost, demand_sums);
    follower.run(&evaluated);
    EXPECT_NEAR(evaluated.lower_cost, 170.21902, 1e-4);

    Individual with_routes(&instance, &preprocessor, routes, upper_cost, demand_sums);
    follower.run_with_variables(&with_routes);
    EXPECT_NEAR(with_routes.lower_cost, 170.21902, 1e-4);
    EXPECT_EQ(check_charged_route(instance, preprocessor, follower, 0, routes[0], with_routes.lower_cost), 2);
}
