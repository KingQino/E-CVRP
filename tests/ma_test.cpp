//
// Created by Yinghao Qin on 11/10/2026.
//

#include "gtest/gtest.h"
#include "ma.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
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
        {LocalSearchPolicy::Rank, LocalSearchAction::Strong, false},
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

// -ma_swap_star 0: SWAP* is never called, and the remaining seven operators still reach a local optimum.
TEST_F(MaTest, SwapStarCanBeLeftOutOfTheNeighbourhoods) {
    const Parameters params = make_ma_params();
    auto preprocessor = make_preprocessor(params, 1e9);
    const auto swap_star = static_cast<std::size_t>(LocalSearchOperator::SwapStar);
    for (const bool enabled : {true, false}) {
        std::mt19937 engine(3);
        auto ind = make_random_individual(*preprocessor, engine);
        LocalSearchWorkspace workspace;
        workspace.swap_star_enabled = enabled;
        const LocalSearchResult result = MaLeader::improve_unrestricted(*ind, *instance, engine, workspace);
        EXPECT_TRUE(result.reached_local_optimum);
        if (enabled) {
            EXPECT_GT(result.operator_stats[swap_star].calls, 0);
        } else {
            EXPECT_EQ(result.operator_stats[swap_star].calls, 0);
        }
        for (std::size_t i = 0; i < LOCAL_SEARCH_OPERATOR_COUNT; ++i) {
            if (i != swap_star) EXPECT_GT(result.operator_stats[i].calls, 0) << MaLeader::operator_name(static_cast<LocalSearchOperator>(i));
        }
        expect_valid_upper_solution(*ind, *instance);
    }
}

// The option reaches every local search of a full online run (allocation workspaces included).
TEST_F(MaTest, SwapStarOptionReachesTheOnlineRun) {
    const fs::path stats = fs::temp_directory_path() / "ecvrp_swap_star_test";
    for (const bool enabled : {true, false}) {
        fs::remove_all(stats);
        Case fresh_instance((fs::path(__FILE__).parent_path().parent_path() / "data").string(), "E-n29-k4-s7.evrp");
        Parameters params = make_ma_params();
        params.ma_pop_size = 50;
        params.ma_ls_policy = LocalSearchPolicy::OnlineIndividual;
        params.ma_unrestricted_credit_rho = 0.0;
        params.ma_swap_star = enabled;
        params.enable_logging = true;
        params.k_stats_path = stats.string();
        auto preprocessor = std::make_unique<Preprocessor>(fresh_instance, params);
        preprocessor->max_evals_ = 5'000.0;
        Ma search(params.seed, &fresh_instance, preprocessor.get());
        EXPECT_LT(search.run(), INFEASIBLE);
        std::uint64_t swap_star_calls = 0, other_calls = 0;
        for (const auto& entry : fs::recursive_directory_iterator(stats)) {
            if (entry.path().filename() != "local-search-operators.tsv") continue;
            std::ifstream in(entry.path());
            std::string line;
            std::getline(in, line);
            while (std::getline(in, line)) {
                std::istringstream fields(line);
                std::string iter, name;
                std::uint64_t calls = 0;
                fields >> iter >> name >> calls;
                (name == "swap_star" ? swap_star_calls : other_calls) += calls;
            }
        }
        EXPECT_GT(other_calls, 0u);
        if (enabled) {
            EXPECT_GT(swap_star_calls, 0u);
        } else {
            EXPECT_EQ(swap_star_calls, 0u);
        }
    }
    fs::remove_all(stats);
}

// ---------------------------------------------------------------------------------------------------------
// Ablation settings

TEST(MaAblationTest, ActionCountsUseLargestRemainders) {
    using Counts = std::array<std::size_t, 3>;
    EXPECT_EQ(LocalSearchAllocationRunner::action_counts(10, {0.5, 0.4, 0.1}), (Counts{5, 4, 1}));
    EXPECT_EQ(LocalSearchAllocationRunner::action_counts(7, {0.3, 0.4, 0.3}), (Counts{2, 3, 2}));
    EXPECT_EQ(LocalSearchAllocationRunner::action_counts(0, {0.3, 0.4, 0.3}), (Counts{0, 0, 0}));
    EXPECT_EQ(LocalSearchAllocationRunner::action_counts(3, {0.0, 0.0, 1.0}), (Counts{0, 0, 3}));
    for (std::size_t eligible = 0; eligible < 120; ++eligible) {
        const Counts counts = LocalSearchAllocationRunner::action_counts(eligible, {0.7563, 0.0348, 0.2089});
        EXPECT_EQ(counts[0] + counts[1] + counts[2], eligible);
    }
}

TEST(MaAblationTest, ContextModesMaskTheModelInput) {
    LocalSearchAllocationContext context;
    context.quality_gap = 0.3;
    context.budget_progress = 0.5;
    context.probe_success_rate = 0.4;
    context.probe_relative_gain = 0.01;
    context.probe_efficiency = 0.6;
    context.probe_cost = 0.2;
    OnlineActionLearner learner;
    learner.set_context_mode(LocalSearchContextMode::NoProbe);
    const auto no_probe = learner.model_context(context);
    EXPECT_DOUBLE_EQ(no_probe.quality_gap, 0.3);
    EXPECT_DOUBLE_EQ(no_probe.budget_progress, 0.5);
    EXPECT_DOUBLE_EQ(no_probe.probe_success_rate, 0.0);
    EXPECT_DOUBLE_EQ(no_probe.probe_relative_gain, 0.0);
    EXPECT_DOUBLE_EQ(no_probe.probe_efficiency, 0.0);
    EXPECT_DOUBLE_EQ(no_probe.probe_cost, 0.0);

    // Context-free: every context gives the same score, which still learns from the rewards.
    learner.set_context_mode(LocalSearchContextMode::None);
    LocalSearchAllocationContext other = context;
    other.quality_gap = 2.0;
    other.probe_efficiency = 0.0;
    learner.update(context, LocalSearchAction::Strong, 1.0, 5.0);
    learner.update(other, LocalSearchAction::Strong, 1.0, 5.0);
    EXPECT_DOUBLE_EQ(learner.score(context, LocalSearchAction::Strong), learner.score(other, LocalSearchAction::Strong));
    EXPECT_GT(learner.score(context, LocalSearchAction::Strong), learner.score(context, LocalSearchAction::Medium));
}

TEST(MaAblationTest, FrozenLearnerDoesNotUpdate) {
    LocalSearchAllocationContext context;
    context.quality_gap = 0.2;
    OnlineActionLearner learner;
    learner.update(context, LocalSearchAction::Medium, 0.5, 3.0);
    const double before = learner.score(context, LocalSearchAction::Medium);
    learner.set_frozen(true);
    learner.update(context, LocalSearchAction::Medium, 1.0, 50.0);
    EXPECT_DOUBLE_EQ(learner.score(context, LocalSearchAction::Medium), before);
    EXPECT_EQ(learner.observation_count(LocalSearchAction::Medium), 1);
    learner.set_frozen(false);
    learner.update(context, LocalSearchAction::Medium, 1.0, 50.0);
    EXPECT_EQ(learner.observation_count(LocalSearchAction::Medium), 2);
}

TEST(MaAblationTest, RewardUsesTheGivenWeights) {
    OnlineActionLearner learner;
    learner.set_reward_weights({0.0, 0.5, 0.4});
    LocalSearchAllocationRun run;
    AllocatedLocalSearchRecord record;
    record.lower_credit = 1.0;
    record.parent_credit = 1.0;
    record.post_probe_gamma_cross = true;
    run.records.push_back(record);
    LocalSearchAllocationRunner::finalize_feedback(run, LocalSearchPolicy::MatchedRandom, learner);
    EXPECT_DOUBLE_EQ(run.records[0].reward, 0.9);
    EXPECT_THROW(learner.set_reward_weights({0.6, 0.3, 0.2}), std::invalid_argument);
}

namespace {

// Sums the selections and forced local optima per action of local-search-allocation.tsv files under `root`.
struct AllocationTotals {
    std::array<long, 3> selections{};
    std::array<long, 3> forced{};
};

AllocationTotals read_allocation_totals(const fs::path& root) {
    AllocationTotals totals;
    for (const auto& entry : fs::recursive_directory_iterator(root)) {
        if (entry.path().filename() != "local-search-allocation.tsv") continue;
        std::ifstream in(entry.path());
        std::string line;
        std::getline(in, line);
        while (std::getline(in, line)) {
            std::istringstream fields(line);
            std::string iter, policy, action;
            long selections = 0, forced = 0;
            fields >> iter >> policy >> action >> selections >> forced;
            const std::size_t index = action == "weak" ? 0 : (action == "medium" ? 1 : 2);
            totals.selections[index] += selections;
            totals.forced[index] += forced;
        }
    }
    return totals;
}

}  // namespace

// Rank and matched random give exactly the requested shares to the individuals eligible after the probe.
TEST_F(MaTest, RankAndMatchedRandomFollowTheShares) {
    const fs::path stats = fs::temp_directory_path() / "ecvrp_allocation_shares_test";
    for (const LocalSearchPolicy policy : {LocalSearchPolicy::Rank, LocalSearchPolicy::MatchedRandom}) {
        for (const LocalSearchActionShares shares : {LocalSearchActionShares{0.0, 0.0, 1.0},
                                                     LocalSearchActionShares{1.0, 0.0, 0.0},
                                                     LocalSearchActionShares{0.5, 0.5, 0.0}}) {
            fs::remove_all(stats);
            Case fresh_instance((fs::path(__FILE__).parent_path().parent_path() / "data").string(), "E-n29-k4-s7.evrp");
            Parameters params = make_ma_params();
            params.ma_pop_size = 50;
            params.ma_ls_policy = policy;
            params.ma_action_shares = shares;
            params.enable_logging = true;
            params.k_stats_path = stats.string();
            auto preprocessor = std::make_unique<Preprocessor>(fresh_instance, params);
            preprocessor->max_evals_ = 5'000.0;
            Ma search(params.seed, &fresh_instance, preprocessor.get());
            EXPECT_LT(search.run(), INFEASIBLE);
            const AllocationTotals totals = read_allocation_totals(stats);
            const long eligible = totals.selections[0] - totals.forced[0] + totals.selections[1] + totals.selections[2];
            ASSERT_GT(eligible, 0);
            EXPECT_EQ(totals.forced[1] + totals.forced[2], 0);
            if (shares[2] == 1.0) EXPECT_EQ(totals.selections[2], eligible);
            if (shares[0] == 1.0) EXPECT_EQ(totals.selections[0] - totals.forced[0], eligible);
            if (shares[1] == 0.5) EXPECT_NEAR(static_cast<double>(totals.selections[1]) / eligible, 0.5, 0.05);
        }
    }
    fs::remove_all(stats);
}

// Per-instance shares are read from a file and normalised; a missing instance is an error.
TEST_F(MaTest, ActionSharesFileIsReadPerInstance) {
    const fs::path file = fs::temp_directory_path() / "ecvrp_shares_test.tsv";
    {
        std::ofstream out(file);
        out << "instance\tweak\tmedium\tstrong\n";
        out << "E-n22-k4\t0.1\t0.1\t0.8\n";
        out << "E-n29-k4-s7\t2\t1\t1\n";
    }
    Parameters params = make_ma_params();
    params.ma_ls_policy = LocalSearchPolicy::Rank;
    params.ma_action_shares_file = file.string();
    auto preprocessor = make_preprocessor(params, 1'000.0);
    const Ma search(params.seed, instance.get(), preprocessor.get());
    EXPECT_DOUBLE_EQ(search.allocation_options.action_shares[0], 0.5);
    EXPECT_DOUBLE_EQ(search.allocation_options.action_shares[1], 0.25);
    EXPECT_DOUBLE_EQ(search.allocation_options.action_shares[2], 0.25);
    {
        std::ofstream out(file);
        out << "E-n22-k4\t0.1\t0.1\t0.8\n";
    }
    EXPECT_THROW(Ma(params.seed, instance.get(), preprocessor.get()), std::invalid_argument);
    fs::remove(file);
}

// Default ablation settings reproduce the reference run; the reference arms change the result.
TEST_F(MaTest, AblationDefaultsAreTheReference) {
    const Parameters defaults;
    EXPECT_EQ(defaults.ma_context, LocalSearchContextMode::Full);
    EXPECT_DOUBLE_EQ(defaults.ma_freeze_learning_at, 1.0);
    EXPECT_DOUBLE_EQ(defaults.ma_reward_weights[0], 0.45);
    EXPECT_DOUBLE_EQ(defaults.ma_reward_weights[1], 0.25);
    EXPECT_DOUBLE_EQ(defaults.ma_reward_weights[2], 0.20);
}

// ---------------------------------------------------------------------------------------------------------
// Freeze study: freeze at 0 and summary logging

namespace {

std::map<std::string, std::string> read_single_row_tsv(const fs::path& path) {
    std::ifstream in(path);
    std::string header, row;
    std::getline(in, header);
    std::getline(in, row);
    std::map<std::string, std::string> values;
    std::istringstream names(header), fields(row);
    std::string name, field;
    while (std::getline(names, name, '\t') && std::getline(fields, field, '\t')) values[name] = field;
    return values;
}

struct LoggedRun {
    double cost;
    fs::path directory;
};

LoggedRun run_logged(const fs::path& stats, double freeze_at, bool summary) {
    Case fresh_instance((fs::path(__FILE__).parent_path().parent_path() / "data").string(), "E-n29-k4-s7.evrp");
    Parameters params = make_ma_params();
    params.ma_pop_size = 50;
    params.ma_ls_policy = LocalSearchPolicy::OnlineIndividual;
    params.ma_unrestricted_credit_rho = 0.0;
    params.ma_freeze_learning_at = freeze_at;
    params.ma_summary_log = summary;
    params.enable_logging = true;
    params.k_stats_path = stats.string();
    auto preprocessor = std::make_unique<Preprocessor>(fresh_instance, params);
    preprocessor->max_evals_ = 20'000.0;
    Ma search(params.seed, &fresh_instance, preprocessor.get());
    const double cost = search.run();
    return {cost, stats / "MA" / "E-n29-k4-s7" / std::to_string(params.seed)};
}

}  // namespace

TEST(MaFreezeStudyTest, FreezeAtZeroNeverUpdatesTheModels) {
    const fs::path stats = fs::temp_directory_path() / "ecvrp_freeze_zero_test";
    fs::remove_all(stats);
    const LoggedRun run = run_logged(stats, 0.0, true);
    EXPECT_LT(run.cost, INFEASIBLE);
    auto summary = read_single_row_tsv(run.directory / "run-summary.tsv");
    EXPECT_EQ(summary["freeze_generation"], "1");
    EXPECT_EQ(summary["updates_weak"], "0");
    EXPECT_EQ(summary["updates_medium"], "0");
    EXPECT_EQ(summary["updates_strong"], "0");
    EXPECT_EQ(summary["weak_selections_pre"], "0");
    EXPECT_GT(std::stol(summary["weak_selections_post"]) + std::stol(summary["strong_selections_post"]), 0);
    fs::remove_all(stats);
}

// Summary logging gives the same run as full logging, writes three files, and its totals match the full logs.
TEST(MaFreezeStudyTest, SummaryLogMatchesFullLog) {
    const fs::path full_stats = fs::temp_directory_path() / "ecvrp_freeze_full_test";
    const fs::path summary_stats = fs::temp_directory_path() / "ecvrp_freeze_summary_test";
    fs::remove_all(full_stats);
    fs::remove_all(summary_stats);
    const LoggedRun full = run_logged(full_stats, 0.3, false);
    const LoggedRun summary = run_logged(summary_stats, 0.3, true);
    EXPECT_DOUBLE_EQ(full.cost, summary.cost);

    std::vector<std::string> files;
    for (const auto& entry : fs::directory_iterator(summary.directory)) files.push_back(entry.path().filename().string());
    std::sort(files.begin(), files.end());
    EXPECT_EQ(files, (std::vector<std::string>{"run-configuration.tsv", "run-summary.tsv", "solution.E-n29-k4-s7.txt"}));

    auto a = read_single_row_tsv(full.directory / "run-summary.tsv");
    auto b = read_single_row_tsv(summary.directory / "run-summary.tsv");
    EXPECT_EQ(a, b);

    // Freeze took effect after the first generation, and the post-freeze phase has selections.
    EXPECT_GT(std::stoi(b["freeze_generation"]), 1);
    EXPECT_GE(std::stod(b["freeze_budget_fraction"]), 0.3);
    EXPECT_GT(std::stol(b["weak_selections_post"]) + std::stol(b["strong_selections_post"]), 0);

    // Selections in run-summary.tsv equal the sums of the full allocation log.
    std::array<long, 3> logged{};
    std::ifstream in(full.directory / "local-search-allocation.tsv");
    std::string line;
    std::getline(in, line);
    while (std::getline(in, line)) {
        std::istringstream fields(line);
        std::string iter, policy, action;
        long selections = 0;
        fields >> iter >> policy >> action >> selections;
        logged[action == "weak" ? 0 : (action == "medium" ? 1 : 2)] += selections;
    }
    const std::array<const char*, 3> names = {"weak", "medium", "strong"};
    for (std::size_t i = 0; i < 3; ++i) {
        const std::string name = names[i];
        EXPECT_EQ(std::stol(b[name + "_selections_pre"]) + std::stol(b[name + "_selections_post"]), logged[i]) << name;
    }
    fs::remove_all(full_stats);
    fs::remove_all(summary_stats);
}
