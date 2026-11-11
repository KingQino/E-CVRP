//
// Tests for the charging-solver experiments (branch frvcp-experiments):
// exact charging during search (-lower_solver dp) and shadow diagnostics (-shadow_dp 1).
//

#include "gtest/gtest.h"
#include "follower.hpp"
#include "lahc.hpp"
#include "ma.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

fs::path data_path() { return fs::path(__FILE__).parent_path().parent_path() / "data"; }

struct RunResult {
    double cost;
    std::uint64_t distance_calls;
};

template <typename Algorithm>
RunResult run_once(const std::string& instance_name, const bool shadow, const bool exact_search) {
    Parameters params;
    params.k_stats_path = (fs::temp_directory_path() / "ecvrp_frvcp_test").string();
    params.shadow_dp = shadow;
    params.lower_solver_dp = exact_search;
    params.se_both_counts = true;
    params.ma_ls_policy = LocalSearchPolicy::Static;
    params.ma_static_unrestricted = true;
    Case instance(data_path().string(), instance_name);
    Preprocessor preprocessor(instance, params);
    Algorithm algorithm(2, &instance, &preprocessor);
    const double cost = algorithm.run();
    return {cost, instance.get_distance_calls()};
}

}  // namespace

// The exact evaluation used during search gives the same cost as the final refinement.
TEST(OptimalChargingTest, EvaluationMatchesRefinement) {
    for (const char* name : {"RC204-15.evrp", "R202-15.evrp"}) {
        Parameters params;
        Case instance(data_path().string(), name);
        Preprocessor preprocessor(instance, params);
        Follower follower(&instance, &preprocessor);
        std::vector<int> customers;
        for (int c = 1; c <= instance.num_customer_; ++c) customers.push_back(c);
        std::vector<std::vector<int>> routes = {{0}};
        routes[0].insert(routes[0].end(), customers.begin(), customers.end());
        routes[0].push_back(0);
        const double exact = follower.eval_optimal_cost(routes[0].data(), static_cast<int>(routes[0].size()));
        Individual refined(&instance, &preprocessor, routes, instance.compute_total_distance(routes),
                           instance.compute_demand_sum_per_route(routes));
        follower.refine(&refined);
        ASSERT_GT(exact, 0.0) << name;
        EXPECT_NEAR(exact, refined.lower_cost, 1e-6) << name;
    }
}

// Exact charging during search never reports a route as more expensive than simple enumeration.
TEST(OptimalChargingTest, ExactSearchEvaluationIsNeverWorseThanSimpleEnumeration) {
    Parameters params;
    Case instance(data_path().string(), "R102-15.evrp");
    Preprocessor preprocessor(instance, params);
    Follower heuristic(&instance, &preprocessor);
    heuristic.se_compare_station_counts = true;
    Follower exact(&instance, &preprocessor);
    exact.use_optimal_charging = true;
    std::vector<int> customers;
    for (int c = 1; c <= instance.num_customer_; ++c) customers.push_back(c);
    std::mt19937 rng(7);
    for (int trial = 0; trial < 50; ++trial) {
        std::shuffle(customers.begin(), customers.end(), rng);
        std::vector<std::vector<int>> routes = {{0}};
        routes[0].insert(routes[0].end(), customers.begin(), customers.end());
        routes[0].push_back(0);
        Individual a(&instance, &preprocessor, routes, instance.compute_total_distance(routes),
                     instance.compute_demand_sum_per_route(routes));
        Individual b(a);
        heuristic.run(&a);
        exact.run(&b);
        EXPECT_LE(b.lower_cost, a.lower_cost + 1e-6);
    }
}

// Shadow diagnostics must not change the search: same result and same budget use.
TEST(ShadowDiagnosticsTest, ShadowModeDoesNotChangeLahc) {
    const RunResult off = run_once<Lahc>("R102-15.evrp", false, false);
    const RunResult on = run_once<Lahc>("R102-15.evrp", true, false);
    EXPECT_EQ(off.cost, on.cost);
    EXPECT_EQ(off.distance_calls, on.distance_calls);
    EXPECT_TRUE(fs::exists(fs::temp_directory_path() / "ecvrp_frvcp_test" / "LAHC" / "R102-15" / "2" / "shadow.R102-15.csv"));
    fs::remove_all(fs::temp_directory_path() / "ecvrp_frvcp_test");
}

TEST(ShadowDiagnosticsTest, ShadowModeDoesNotChangeMa) {
    const RunResult off = run_once<Ma>("R102-15.evrp", false, false);
    const RunResult on = run_once<Ma>("R102-15.evrp", true, false);
    EXPECT_EQ(off.cost, on.cost);
    EXPECT_EQ(off.distance_calls, on.distance_calls);
    fs::remove_all(fs::temp_directory_path() / "ecvrp_frvcp_test");
}

// Exact charging during search runs to completion for both algorithms.
TEST(OptimalChargingTest, ExactSearchRuns) {
    EXPECT_LT(run_once<Lahc>("E-n22-k4.evrp", false, true).cost, 1e6);
    EXPECT_LT(run_once<Ma>("E-n22-k4.evrp", false, true).cost, 1e6);
    fs::remove_all(fs::temp_directory_path() / "ecvrp_frvcp_test");
}
