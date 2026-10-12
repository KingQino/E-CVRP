//
// Created by Yinghao Qin on 11/10/2026.
//

#include "gtest/gtest.h"
#include "ma.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <numeric>
#include <random>
#include <vector>

namespace fs = std::filesystem;

namespace {

Parameters make_ma_params() {
    Parameters params;
    params.algorithm = Algorithm::MA;
    params.enable_logging = false;
    params.enable_final_refinement = true;
    params.stop_criteria = 0;
    params.seed = 7;
    return params;
}

double route_distance(const Case& instance, const int* route, const int length) {
    double distance = 0.0;
    for (int i = 0; i + 1 < length; ++i) {
        distance += instance.distances_[route[i]][route[i + 1]];
    }
    return distance;
}

// Every customer exactly once, depot at both ends, capacity respected, cached demand/cost consistent.
void expect_valid_upper_solution(const Individual& ind, const Case& instance) {
    std::vector<int> visit_count(instance.num_customer_ + 1, 0);
    double total_distance = 0.0;
    for (int r = 0; r < ind.num_routes; ++r) {
        const int length = ind.num_nodes_per_route[r];
        ASSERT_GE(length, 3) << "MA never keeps empty routes";
        EXPECT_EQ(ind.routes[r][0], instance.depot_);
        EXPECT_EQ(ind.routes[r][length - 1], instance.depot_);
        int demand = 0;
        for (int pos = 1; pos < length - 1; ++pos) {
            const int node = ind.routes[r][pos];
            ASSERT_GE(node, 1);
            ASSERT_LE(node, instance.num_customer_);
            ++visit_count[node];
            demand += instance.get_customer_demand_(node);
        }
        EXPECT_EQ(demand, ind.demand_sum_per_route[r]);
        EXPECT_LE(demand, instance.max_vehicle_capa_);
        total_distance += route_distance(instance, ind.routes[r], length);
    }
    for (int customer = 1; customer <= instance.num_customer_; ++customer) {
        EXPECT_EQ(visit_count[customer], 1) << "Customer " << customer << " should appear exactly once";
    }
    EXPECT_NEAR(total_distance, ind.upper_cost, 1e-6);
}

std::vector<int> sorted_copy(std::vector<int> values) {
    std::sort(values.begin(), values.end());
    return values;
}

} // namespace

class MaTest : public ::testing::Test {
protected:
    void SetUp() override {
        const fs::path data_path = fs::path(__FILE__).parent_path().parent_path() / "data";
        instance = std::make_unique<Case>(data_path.string(), "E-n29-k4-s7.evrp");
    }

    std::unique_ptr<Preprocessor> make_preprocessor(const Parameters& params, const double max_evals) const {
        auto prep = std::make_unique<Preprocessor>(*instance, params);
        prep->max_evals_ = max_evals;
        return prep;
    }

    std::unique_ptr<Individual> make_random_individual(Preprocessor& preprocessor, std::mt19937& engine) const {
        Initializer initializer(engine, instance.get(), &preprocessor);
        const auto routes = initializer.routes_constructor_with_split();
        return std::make_unique<Individual>(instance.get(), &preprocessor, routes,
                                            instance->compute_total_distance(routes),
                                            instance->compute_demand_sum_per_route(routes));
    }

    std::unique_ptr<Case> instance;
};

TEST_F(MaTest, DistanceCallCounterMatchesEvaluations) {
    const double evals_before = instance->get_evals();
    const auto calls_before = instance->get_distance_calls();
    for (int i = 0; i < 50; ++i) {
        (void)instance->get_distance(0, 1);
    }
    const std::vector<std::vector<int>> routes = {{0, 1, 2, 0}};
    (void)instance->compute_total_distance(routes);

    const auto calls = instance->get_distance_calls() - calls_before;
    EXPECT_EQ(calls, 50u + static_cast<std::uint64_t>(instance->problem_size_));
    EXPECT_NEAR(instance->get_evals() - evals_before,
                static_cast<double>(calls) / static_cast<double>(instance->problem_size_), 1e-9);
}

TEST_F(MaTest, LeaderUnrestrictedReachesConsistentLocalOptimum) {
    const Parameters params = make_ma_params();
    auto preprocessor = make_preprocessor(params, 1e9);
    std::mt19937 engine(3);
    auto ind = make_random_individual(*preprocessor, engine);
    const double cost_before = ind->upper_cost;

    LocalSearchWorkspace workspace;
    const LocalSearchResult result = MaLeader::improve_unrestricted(*ind, *instance, engine, workspace);

    EXPECT_TRUE(result.reached_local_optimum);
    EXPECT_TRUE(ind->upper_locally_optimal);
    EXPECT_LE(ind->upper_cost, cost_before + 1e-9);
    EXPECT_GT(result.distance_calls_used, 0u);
    expect_valid_upper_solution(*ind, *instance);
}

TEST_F(MaTest, WeakProbeRespectsMoveLimitAndCanBeContinued) {
    const Parameters params = make_ma_params();
    auto preprocessor = make_preprocessor(params, 1e9);
    std::mt19937 engine(5);
    auto ind = make_random_individual(*preprocessor, engine);
    const LocalSearchDepthConfig depth = local_search_depth_config(LocalSearchDepthProfile::D5);

    LocalSearchWorkspace workspace;
    LocalSearchSession session;
    MaLeader::begin_eight_neighborhood_rvnd_one_move_session(*ind, session, workspace);
    const int weak_limit = MaLeader::move_limit_for_action(*ind, *instance, LocalSearchAction::Weak, depth);
    const LocalSearchResult weak = MaLeader::continue_eight_neighborhood_rvnd_one_move_session(
        *ind, *instance, engine, session, weak_limit, workspace, std::numeric_limits<double>::infinity());
    EXPECT_LE(weak.accepted_moves, weak_limit);

    const LocalSearchResult rest = MaLeader::continue_eight_neighborhood_rvnd_one_move_session(
        *ind, *instance, engine, session, -1, workspace, std::numeric_limits<double>::infinity());
    EXPECT_TRUE(rest.reached_local_optimum);
    EXPECT_EQ(session.total_accepted_moves, weak.accepted_moves + rest.accepted_moves);
    expect_valid_upper_solution(*ind, *instance);
}

TEST_F(MaTest, OffspringArePermutationsOfCustomers) {
    const Parameters params = make_ma_params();
    auto preprocessor = make_preprocessor(params, 1e9);
    std::mt19937 engine(11);
    std::uniform_real_distribution<double> probability(0.0, 1.0);

    std::vector<std::shared_ptr<Individual>> population;
    for (int i = 0; i < 20; ++i) {
        population.push_back(std::shared_ptr<Individual>(make_random_individual(*preprocessor, engine).release()));
    }
    std::sort(population.begin(), population.end(), [](const auto& a, const auto& b) {
        return a->upper_cost < b->upper_cost;
    });
    const auto pool = MaReproduction::build_parent_pool(population, 5);
    ASSERT_EQ(pool.size(), 5u);

    ReproductionWorkspace workspace;
    std::vector<int> parent_use_counts;
    const auto offspring = MaReproduction::create_offspring(
        pool, population.front().get(), true, preprocessor->customer_ids_, 19, 2, 0.5, 0.2, 0.05, 0.10,
        engine, probability, workspace, &parent_use_counts);

    ASSERT_EQ(offspring.size(), 19u);
    ASSERT_EQ(parent_use_counts.size(), pool.size());
    EXPECT_GT(std::accumulate(parent_use_counts.begin(), parent_use_counts.end(), 0), 0);
    const std::vector<int> customers = sorted_copy(preprocessor->customer_ids_);
    for (const auto& chromosome : offspring) {
        EXPECT_EQ(sorted_copy(chromosome), customers);
    }
}

TEST_F(MaTest, RunReturnsFeasibleRefinedSolution) {
    Parameters params = make_ma_params();
    params.ma_pop_size = 50;  // keeps the parent pool at its minimum size of 5
    auto preprocessor = make_preprocessor(params, 20'000.0);
    Ma search(params.seed, instance.get(), preprocessor.get());

    const double final_cost = search.run();

    EXPECT_GT(search.generation, 1);
    EXPECT_TRUE(std::isfinite(final_cost));
    EXPECT_LT(final_cost, INFEASIBLE);
    EXPECT_DOUBLE_EQ(final_cost, search.verified_best->lower_cost);
    EXPECT_GE(final_cost + 1e-9, search.verified_best->upper_cost);
    EXPECT_GE(instance->get_evals(), preprocessor->max_evals_);
    EXPECT_EQ(static_cast<int>(search.population.size()), params.ma_pop_size);
    expect_valid_upper_solution(*search.verified_best, *instance);
    expect_valid_upper_solution(*search.upper_best, *instance);
    for (const auto& individual : search.population) {
        expect_valid_upper_solution(*individual, *instance);
    }
}

TEST_F(MaTest, SameSeedIsReproducible) {
    Parameters params = make_ma_params();
    params.ma_pop_size = 50;
    double costs[2];
    for (double& cost : costs) {
        Case fresh_instance((fs::path(__FILE__).parent_path().parent_path() / "data").string(), "E-n29-k4-s7.evrp");
        auto preprocessor = std::make_unique<Preprocessor>(fresh_instance, params);
        preprocessor->max_evals_ = 10'000.0;
        Ma search(params.seed, &fresh_instance, preprocessor.get());
        cost = search.run();
    }
    EXPECT_DOUBLE_EQ(costs[0], costs[1]);
}

TEST_F(MaTest, AllAllocationPoliciesRun) {
    struct Configuration {
        LocalSearchPolicy policy;
        LocalSearchAction static_action;
        bool static_unrestricted;
    };
    const std::vector<Configuration> configurations = {
        {LocalSearchPolicy::Static, LocalSearchAction::Weak, false},
        {LocalSearchPolicy::Static, LocalSearchAction::Medium, false},
        {LocalSearchPolicy::Static, LocalSearchAction::Strong, false},
        {LocalSearchPolicy::Static, LocalSearchAction::Strong, true},
        {LocalSearchPolicy::RandomMixed, LocalSearchAction::Strong, false},
        {LocalSearchPolicy::MatchedRandom, LocalSearchAction::Strong, false},
        {LocalSearchPolicy::OnlineIndividual, LocalSearchAction::Strong, false},
    };
    for (const auto& [policy, static_action, static_unrestricted] : configurations) {
        Case fresh_instance((fs::path(__FILE__).parent_path().parent_path() / "data").string(), "E-n29-k4-s7.evrp");
        Parameters params = make_ma_params();
        params.ma_pop_size = 50;
        params.ma_ls_policy = policy;
        params.ma_static_action = static_action;
        params.ma_static_unrestricted = static_unrestricted;
        auto preprocessor = std::make_unique<Preprocessor>(fresh_instance, params);
        preprocessor->max_evals_ = 5'000.0;
        Ma search(params.seed, &fresh_instance, preprocessor.get());
        const double cost = search.run();
        EXPECT_LT(cost, INFEASIBLE) << local_search_policy_name(policy);
        expect_valid_upper_solution(*search.verified_best, fresh_instance);
    }
}
