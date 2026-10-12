//
// Created by Yinghao Qin on 11/10/2026.
//
// Allocation of upper-level local-search actions across the MA population, including the
// online contextual learner (LinUCB reward and log-cost models per action).
//

#ifndef MA_LOCAL_SEARCH_ALLOCATION_HPP
#define MA_LOCAL_SEARCH_ALLOCATION_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "ma_leader.hpp"
#include "ma_reproduction.hpp"
#include "ma_types.hpp"

class Case;
class Individual;

struct LocalSearchAllocationContext {
    double quality_gap{};
    double adjacency_distance{};
    double budget_progress{};
    double gamma_margin{};
    double upper_stagnation{};
    double lower_stagnation{};
    double population_dispersion{};
    double recent_gamma_rate{};
    double probe_success_rate{};
    double probe_relative_gain{};
    double probe_efficiency{};
    double probe_cost{};
};

struct LinearUcbEstimate {
    double prediction{};
    double uncertainty{};
};

class LinearUcbModel {
public:
    static constexpr std::size_t FEATURE_COUNT = 14;
    using FeatureVector = std::array<double, FEATURE_COUNT>;

    LinearUcbModel();

    void reset();
    [[nodiscard]] LinearUcbEstimate estimate(
        const LocalSearchAllocationContext& context) const;
    void update(
        const LocalSearchAllocationContext& context,
        double target);

private:
    using Matrix =
        std::array<std::array<double, FEATURE_COUNT>, FEATURE_COUNT>;

    Matrix inverse_gram{};
    FeatureVector targets{};
    FeatureVector coefficients{};

    [[nodiscard]] static FeatureVector features(
        const LocalSearchAllocationContext& context);
    [[nodiscard]] static FeatureVector multiply(
        const Matrix& matrix,
        const FeatureVector& vector);
    [[nodiscard]] static double dot(
        const FeatureVector& first,
        const FeatureVector& second);
};

struct LocalSearchActionDecision {
    LocalSearchAction action{LocalSearchAction::Weak};
    double score{};
    bool exploratory{};
};

class OnlineActionLearner {
public:
    static constexpr std::size_t LOWER_ARCHIVE_CAPACITY = 10;
    static constexpr double DEFAULT_ACTION_COST_WEIGHT = 0.02;

    void reset();
    void set_action_cost_weight(double action_cost_weight);
    [[nodiscard]] double action_cost_weight() const;

    [[nodiscard]] LocalSearchActionDecision select(
        const LocalSearchAllocationContext& context,
        int generation,
        std::mt19937& random_engine) const;
    void update(
        const LocalSearchAllocationContext& context,
        LocalSearchAction action,
        double reward,
        double incremental_cost_units);
    [[nodiscard]] std::vector<std::pair<const Individual*, double>>
    update_lower_archive(
        const std::vector<std::shared_ptr<Individual>>& complete_solutions);

    [[nodiscard]] double score(
        const LocalSearchAllocationContext& context,
        LocalSearchAction action) const;
    [[nodiscard]] int observation_count(
        LocalSearchAction action) const;

private:
    struct LowerArchiveEntry {
        std::vector<int> chromosome;
        double lower_cost{};
    };

    std::array<LinearUcbModel, 3> reward_models;
    std::array<LinearUcbModel, 3> log_cost_models;
    std::array<int, 3> observation_counts{};
    std::vector<LowerArchiveEntry> lower_archive;
    double action_cost_weight_value{DEFAULT_ACTION_COST_WEIGHT};
};

struct LocalSearchAllocationStats {
    int selections{};
    int forced_local_optima{};
    int exploratory_selections{};
    int local_optimum_terminations{};
    int move_limit_terminations{};
    int distance_limit_terminations{};
    int accepted_moves{};
    int neighborhood_calls{};
    std::uint64_t distance_calls{};
    std::uint64_t continuation_distance_calls{};
    double upper_gain{};
    int gamma_crosses{};
    int parent_uses{};
    int lower_archive_entries{};
    double parent_reward{};
    double lower_reward{};
    double gamma_reward{};
    double continuation_gain_signal{};
    double reward{};
    double incremental_cost_units{};
    double selection_score{};
};

struct AllocatedLocalSearchRecord {
    std::shared_ptr<Individual> individual;
    LocalSearchSession session;
    LocalSearchAction terminal_action{LocalSearchAction::Weak};
    LocalSearchResult weak_result;
    LocalSearchResult continuation_result;
    LocalSearchResult total_result;
    int strong_move_limit{};
    std::uint64_t strong_distance_call_limit{};
    LocalSearchAllocationContext context;
    double cost_before_weak{};
    double cost_after_weak{};
    double cost_after_terminal{};
    double selection_score{};
    double parent_credit{};
    double lower_credit{};
    double normalized_continuation_gain{};
    double reward{};
    double incremental_cost_units{};
    int parent_use_count{};
    bool forced_local_optimum{};
    bool exploratory_selection{};
    bool exclude_from_learner_feedback{};
    bool crossed_gamma{};
    bool post_probe_gamma_cross{};
};

struct LocalSearchAllocationRun {
    std::vector<AllocatedLocalSearchRecord> records;
    std::array<LocalSearchAllocationStats, 3> stats{};
    std::array<
        LocalSearchOperatorStats,
        LOCAL_SEARCH_OPERATOR_COUNT> operator_stats{};
};

class LocalSearchAllocationRunner {
public:
    static LocalSearchAllocationRun run(
        const std::vector<std::shared_ptr<Individual>>& population,
        const std::shared_ptr<Individual>& unchanged_lower_elite,
        Case& instance,
        int generation,
        LocalSearchPolicy policy,
        const ParentCandidate& upper_reference,
        double trigger_upper_bound,
        double budget_progress,
        double upper_stagnation,
        double lower_stagnation,
        double population_dispersion,
        double recent_gamma_rate,
        std::uint64_t evaluation_limit_distance_calls,
        std::mt19937& local_search_engine,
        std::mt19937& allocation_engine,
        std::vector<LocalSearchWorkspace>& workspaces,
        const OnlineActionLearner& learner,
        const LocalSearchDepthConfig& depth_config = {});

    static void assign_parent_use_feedback(
        LocalSearchAllocationRun& run,
        const std::vector<ParentCandidate>& parent_pool,
        const std::vector<int>& parent_use_counts);
    static std::vector<std::pair<const Individual*, double>>
    assign_lower_archive_feedback(
        LocalSearchAllocationRun& run,
        const std::vector<std::shared_ptr<Individual>>& complete_solutions,
        OnlineActionLearner& learner);
    static void finalize_feedback(
        LocalSearchAllocationRun& run,
        LocalSearchPolicy policy,
        OnlineActionLearner& learner);

    static std::size_t action_index(
        LocalSearchAction action);
};


#endif //MA_LOCAL_SEARCH_ALLOCATION_HPP
