//
// Created by Yinghao Qin on 11/10/2026.
//

#include "ma_local_search_allocation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include "case.hpp"
#include "follower.hpp"
#include "individual.hpp"
#include "ma_leader.hpp"

namespace {

constexpr double kRidge = 1.0;
constexpr double kExplorationScale = 0.08;
constexpr double kRandomExplorationRate = 0.05;
constexpr double kLowerRewardWeight = 0.45;
constexpr double kGammaRewardWeight = 0.25;
constexpr double kParentRewardWeight = 0.20;
constexpr double kContinuationGainSignalWeight = 0.10;
constexpr double kContinuationGainScale = 100.0;
constexpr std::size_t kWorkspaceBatchSize = 10;
constexpr double kMediumSelectionRatio = 0.80;
constexpr double kStrongSelectionRatio = 0.50;
// Quality-only shared configuration's confirmed aggregate online allocation.
constexpr double kMatchedMediumRatio = 0.0348226126187;
constexpr double kMatchedStrongRatio = 0.2088046423357;

double bounded_nonnegative(double value) {
    const double nonnegative = std::max(0.0, value);
    return nonnegative / (1.0 + nonnegative);
}

double result_upper_gain(const LocalSearchResult& result) {
    double gain = 0.0;
    for (const auto& stats : result.operator_stats) {
        gain += stats.upper_gain;
    }
    return gain;
}

void add_operator_stats(
    std::array<
        LocalSearchOperatorStats,
        LOCAL_SEARCH_OPERATOR_COUNT>& destination,
    const LocalSearchResult& source) {
    for (std::size_t operator_index = 0;
         operator_index < LOCAL_SEARCH_OPERATOR_COUNT;
         ++operator_index) {
        auto& destination_stats = destination[operator_index];
        const auto& source_stats = source.operator_stats[operator_index];
        destination_stats.calls += source_stats.calls;
        destination_stats.accepts += source_stats.accepts;
        destination_stats.distance_calls += source_stats.distance_calls;
        destination_stats.upper_gain += source_stats.upper_gain;
        destination_stats.gamma_crosses += source_stats.gamma_crosses;
    }
}

LocalSearchResult combine_results(
    const LocalSearchResult& first,
    const LocalSearchResult& second,
    double cost_before,
    double cost_after) {
    LocalSearchResult combined;
    combined.move_limit = second.move_limit;
    combined.accepted_moves =
        first.accepted_moves + second.accepted_moves;
    combined.neighborhood_calls =
        first.neighborhood_calls + second.neighborhood_calls;
    combined.distance_calls_used =
        first.distance_calls_used + second.distance_calls_used;
    combined.relative_upper_improvement = cost_before > 0.0
        ? (cost_before - cost_after) / cost_before
        : 0.0;
    combined.reached_local_optimum = second.reached_local_optimum;
    combined.hit_move_limit = second.hit_move_limit;
    combined.hit_distance_call_limit = second.hit_distance_call_limit;
    combined.operator_stats = first.operator_stats;
    add_operator_stats(combined.operator_stats, second);
    return combined;
}

LocalSearchAllocationContext make_context(
    const Individual& individual,
    const ParentCandidate& reference,
    double trigger_upper_bound,
    double budget_progress,
    double upper_stagnation,
    double lower_stagnation,
    double population_dispersion,
    double recent_gamma_rate,
    const LocalSearchResult& weak_result,
    std::uint64_t evaluation_limit_distance_calls,
    std::size_t population_size) {
    const ParentCandidate candidate =
        MaReproduction::make_parent_candidate(individual);
    LocalSearchAllocationContext context;
    context.quality_gap = reference.upper_cost > 0.0
        ? (candidate.upper_cost - reference.upper_cost)
            / reference.upper_cost
        : 0.0;
    context.adjacency_distance = MaReproduction::adjacency_distance(
        candidate,
        reference);
    context.budget_progress = budget_progress;
    context.gamma_margin = trigger_upper_bound > 0.0
        ? (candidate.upper_cost - trigger_upper_bound)
            / trigger_upper_bound
        : 0.0;
    context.upper_stagnation = upper_stagnation;
    context.lower_stagnation = lower_stagnation;
    context.population_dispersion = population_dispersion;
    context.recent_gamma_rate = recent_gamma_rate;
    context.probe_success_rate = weak_result.neighborhood_calls > 0
        ? static_cast<double>(weak_result.accepted_moves)
            / static_cast<double>(weak_result.neighborhood_calls)
        : 0.0;
    context.probe_relative_gain = weak_result.relative_upper_improvement;
    if (evaluation_limit_distance_calls > 0) {
        const double budget_fraction =
            static_cast<double>(weak_result.distance_calls_used)
            / static_cast<double>(evaluation_limit_distance_calls);
        context.probe_efficiency = budget_fraction > 0.0
            ? bounded_nonnegative(
                weak_result.relative_upper_improvement / budget_fraction)
            : 0.0;
        context.probe_cost = std::clamp(
            budget_fraction
                * static_cast<double>(population_size),
            0.0,
            1.0);
    }
    return context;
}

void finish_record(
    AllocatedLocalSearchRecord& record,
    double trigger_upper_bound) {
    record.cost_after_terminal = record.individual->upper_cost;
    record.crossed_gamma =
        record.cost_before_weak > trigger_upper_bound
        && record.cost_after_terminal <= trigger_upper_bound;
    record.post_probe_gamma_cross =
        record.cost_after_weak > trigger_upper_bound
        && record.cost_after_terminal <= trigger_upper_bound;
}

void aggregate_run_operator_stats(LocalSearchAllocationRun& run) {
    run.operator_stats = {};
    for (const auto& record : run.records) {
        add_operator_stats(run.operator_stats, record.total_result);
    }
}

std::array<double, 3> default_incremental_cost_units() {
    return {0.0, 3.0, 15.0};
}

}  // namespace

LinearUcbModel::LinearUcbModel() {
    reset();
}

void LinearUcbModel::reset() {
    inverse_gram = {};
    targets = {};
    coefficients = {};
    for (std::size_t feature = 0; feature < FEATURE_COUNT; ++feature) {
        inverse_gram[feature][feature] = 1.0 / kRidge;
    }
}

LinearUcbEstimate LinearUcbModel::estimate(
    const LocalSearchAllocationContext& context) const {
    const FeatureVector context_features = features(context);
    const FeatureVector projected_context = multiply(
        inverse_gram,
        context_features);
    LinearUcbEstimate estimate;
    estimate.prediction = dot(coefficients, context_features);
    estimate.uncertainty = std::sqrt(std::max(
        0.0,
        dot(context_features, projected_context)));
    return estimate;
}

void LinearUcbModel::update(
    const LocalSearchAllocationContext& context,
    double target) {
    const FeatureVector context_features = features(context);
    const FeatureVector projected_context = multiply(
        inverse_gram,
        context_features);
    const double denominator =
        1.0 + dot(context_features, projected_context);

    for (std::size_t row = 0; row < FEATURE_COUNT; ++row) {
        for (std::size_t column = 0;
             column < FEATURE_COUNT;
             ++column) {
            inverse_gram[row][column] -=
                projected_context[row]
                * projected_context[column]
                / denominator;
        }
        targets[row] += context_features[row] * target;
    }
    coefficients = multiply(inverse_gram, targets);
}

LinearUcbModel::FeatureVector LinearUcbModel::features(
    const LocalSearchAllocationContext& context) {
    const double quality_gap = bounded_nonnegative(
        context.quality_gap);
    const double adjacency_distance = std::clamp(
        context.adjacency_distance,
        0.0,
        1.0);
    return {
        1.0,
        quality_gap,
        adjacency_distance,
        std::clamp(context.budget_progress, 0.0, 1.0),
        std::tanh(context.gamma_margin),
        std::clamp(context.upper_stagnation, 0.0, 1.0),
        std::clamp(context.lower_stagnation, 0.0, 1.0),
        std::clamp(context.population_dispersion, 0.0, 1.0),
        std::clamp(context.recent_gamma_rate, 0.0, 1.0),
        std::clamp(context.probe_success_rate, 0.0, 1.0),
        bounded_nonnegative(100.0 * context.probe_relative_gain),
        std::clamp(context.probe_efficiency, 0.0, 1.0),
        std::clamp(context.probe_cost, 0.0, 1.0),
        quality_gap * adjacency_distance,
    };
}

LinearUcbModel::FeatureVector LinearUcbModel::multiply(
    const Matrix& matrix,
    const FeatureVector& vector) {
    FeatureVector result{};
    for (std::size_t row = 0; row < FEATURE_COUNT; ++row) {
        for (std::size_t column = 0;
             column < FEATURE_COUNT;
             ++column) {
            result[row] += matrix[row][column] * vector[column];
        }
    }
    return result;
}

double LinearUcbModel::dot(
    const FeatureVector& first,
    const FeatureVector& second) {
    double result = 0.0;
    for (std::size_t feature = 0; feature < FEATURE_COUNT; ++feature) {
        result += first[feature] * second[feature];
    }
    return result;
}

void OnlineActionLearner::reset() {
    for (auto& model : reward_models) {
        model.reset();
    }
    for (auto& model : log_cost_models) {
        model.reset();
    }
    observation_counts = {};
    lower_archive.clear();
}

void OnlineActionLearner::set_action_cost_weight(double new_action_cost_weight) {
    if (!std::isfinite(new_action_cost_weight) || new_action_cost_weight < 0.0) {
        throw std::invalid_argument(
            "action cost weight must be finite and non-negative");
    }
    action_cost_weight_value = new_action_cost_weight;
}

double OnlineActionLearner::action_cost_weight() const {
    return action_cost_weight_value;
}

LocalSearchActionDecision OnlineActionLearner::select(
    const LocalSearchAllocationContext& context,
    int generation,
    std::mt19937& random_engine) const {
    const std::array<LocalSearchAction, 3> actions = {
        LocalSearchAction::Weak,
        LocalSearchAction::Medium,
        LocalSearchAction::Strong,
    };
    std::uniform_real_distribution<double> probability(0.0, 1.0);
    std::uniform_int_distribution<std::size_t> random_action(
        0,
        actions.size() - 1);

    LocalSearchActionDecision decision;
    if (generation <= 1
        || probability(random_engine) < kRandomExplorationRate) {
        decision.action = actions[random_action(random_engine)];
        decision.score = score(context, decision.action);
        decision.exploratory = true;
        return decision;
    }

    std::array<double, 3> scores{};
    for (std::size_t action_index = 0;
         action_index < actions.size();
         ++action_index) {
        scores[action_index] = score(
            context,
            actions[action_index]);
    }
    const double best_score = *std::max_element(
        scores.begin(),
        scores.end());
    std::vector<std::size_t> best_actions;
    for (std::size_t action_index = 0;
         action_index < scores.size();
         ++action_index) {
        if (std::fabs(scores[action_index] - best_score) <= 1e-12) {
            best_actions.push_back(action_index);
        }
    }
    std::uniform_int_distribution<std::size_t> tie_breaker(
        0,
        best_actions.size() - 1);
    const std::size_t selected_index =
        best_actions[tie_breaker(random_engine)];
    decision.action = actions[selected_index];
    decision.score = scores[selected_index];
    return decision;
}

void OnlineActionLearner::update(
    const LocalSearchAllocationContext& context,
    LocalSearchAction action,
    double reward,
    double incremental_cost_units) {
    const std::size_t action_index =
        LocalSearchAllocationRunner::action_index(action);
    reward_models[action_index].update(
        context,
        std::clamp(reward, 0.0, 1.0));
    log_cost_models[action_index].update(
        context,
        std::log1p(std::max(0.0, incremental_cost_units)));
    ++observation_counts[action_index];
}

std::vector<std::pair<const Individual*, double>>
OnlineActionLearner::update_lower_archive(
    const std::vector<std::shared_ptr<Individual>>& complete_solutions) {
    struct Candidate {
        const Individual* individual{};
        std::vector<int> chromosome;
        double lower_cost{};
        bool improves_previous_archive{};
    };

    std::vector<Candidate> candidates;
    for (const auto& individual : complete_solutions) {
        if (individual == nullptr
            || individual->lower_cost >= INFEASIBLE) {
            continue;
        }
        const std::vector<int> chromosome =
            individual->get_chromosome();
        const auto previous = std::find_if(
            lower_archive.begin(),
            lower_archive.end(),
            [&](const LowerArchiveEntry& entry) {
                return entry.chromosome == chromosome;
            });
        const bool improves_previous =
            previous == lower_archive.end()
            || individual->lower_cost
                < previous->lower_cost - 1e-12;

        const auto duplicate = std::find_if(
            candidates.begin(),
            candidates.end(),
            [&](const Candidate& candidate) {
                return candidate.chromosome == chromosome;
            });
        if (duplicate == candidates.end()) {
            candidates.push_back({
                individual.get(),
                chromosome,
                individual->lower_cost,
                improves_previous,
            });
        } else if (individual->lower_cost
                   < duplicate->lower_cost) {
            duplicate->individual = individual.get();
            duplicate->lower_cost = individual->lower_cost;
            duplicate->improves_previous_archive = improves_previous;
        }
    }

    std::vector<LowerArchiveEntry> merged = lower_archive;
    for (const auto& candidate : candidates) {
        const auto existing = std::find_if(
            merged.begin(),
            merged.end(),
            [&](const LowerArchiveEntry& entry) {
                return entry.chromosome == candidate.chromosome;
            });
        if (existing == merged.end()) {
            merged.push_back({
                candidate.chromosome,
                candidate.lower_cost,
            });
        } else if (candidate.lower_cost < existing->lower_cost) {
            existing->lower_cost = candidate.lower_cost;
        }
    }
    std::sort(
        merged.begin(),
        merged.end(),
        [](const LowerArchiveEntry& first,
           const LowerArchiveEntry& second) {
            if (std::fabs(first.lower_cost - second.lower_cost) > 1e-12) {
                return first.lower_cost < second.lower_cost;
            }
            return first.chromosome < second.chromosome;
        });
    if (merged.size() > LOWER_ARCHIVE_CAPACITY) {
        merged.resize(LOWER_ARCHIVE_CAPACITY);
    }
    lower_archive.swap(merged);

    std::vector<std::pair<const Individual*, double>> credits;
    if (lower_archive.empty()) {
        return credits;
    }
    for (const auto& candidate : candidates) {
        if (!candidate.improves_previous_archive) {
            continue;
        }
        const auto archived = std::find_if(
            lower_archive.begin(),
            lower_archive.end(),
            [&](const LowerArchiveEntry& entry) {
                return entry.chromosome == candidate.chromosome
                    && std::fabs(
                        entry.lower_cost - candidate.lower_cost)
                        <= 1e-12;
            });
        if (archived == lower_archive.end()) {
            continue;
        }
        const std::size_t rank = static_cast<std::size_t>(
            std::distance(lower_archive.begin(), archived));
        const double credit =
            static_cast<double>(lower_archive.size() - rank)
            / static_cast<double>(lower_archive.size());
        credits.emplace_back(candidate.individual, credit);
    }
    return credits;
}

double OnlineActionLearner::score(
    const LocalSearchAllocationContext& context,
    LocalSearchAction action) const {
    const std::size_t action_index =
        LocalSearchAllocationRunner::action_index(action);
    const LinearUcbEstimate reward_estimate =
        reward_models[action_index].estimate(context);
    const double optimistic_reward = std::max(
        0.0,
        reward_estimate.prediction
            + kExplorationScale * reward_estimate.uncertainty);

    double estimated_cost =
        default_incremental_cost_units()[action_index];
    if (observation_counts[action_index] > 0) {
        const LinearUcbEstimate cost_estimate =
            log_cost_models[action_index].estimate(context);
        estimated_cost = std::max(
            0.0,
            std::expm1(std::clamp(
                cost_estimate.prediction,
                0.0,
                std::log1p(1e6))));
    }
    return optimistic_reward
        - action_cost_weight_value * std::log1p(estimated_cost);
}

int OnlineActionLearner::observation_count(
    LocalSearchAction action) const {
    return observation_counts[
        LocalSearchAllocationRunner::action_index(action)];
}

LocalSearchAllocationRun LocalSearchAllocationRunner::run(
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
    const LocalSearchDepthConfig& depth_config) {
    if (policy == LocalSearchPolicy::Static) {
        throw std::logic_error(
            "static local search does not use the allocation runner");
    }

    LocalSearchAllocationRun run;
    run.records.reserve(population.size());
    const std::size_t required_workspaces = std::min(
        kWorkspaceBatchSize,
        population.size());
    if (workspaces.size() < required_workspaces) {
        workspaces.resize(required_workspaces);
    }

    std::vector<std::shared_ptr<Individual>> search_candidates;
    search_candidates.reserve(population.size());
    for (const auto& individual : population) {
        const bool can_reuse_lower_elite =
            individual == unchanged_lower_elite
            && individual->upper_locally_optimal
            && individual->lower_cost < INFEASIBLE;
        if (!can_reuse_lower_elite) {
            search_candidates.push_back(individual);
        }
    }

    std::vector<LocalSearchAction> matched_actions;
    if (policy == LocalSearchPolicy::MatchedRandom) {
        const std::size_t candidate_count = search_candidates.size();
        const std::size_t strong_count = std::min(
            candidate_count,
            static_cast<std::size_t>(std::lround(
                static_cast<double>(candidate_count)
                * kMatchedStrongRatio)));
        const std::size_t medium_count = std::min(
            candidate_count - strong_count,
            static_cast<std::size_t>(std::lround(
                static_cast<double>(candidate_count)
                * kMatchedMediumRatio)));
        matched_actions.assign(
            candidate_count - medium_count - strong_count,
            LocalSearchAction::Weak);
        matched_actions.insert(
            matched_actions.end(),
            medium_count,
            LocalSearchAction::Medium);
        matched_actions.insert(
            matched_actions.end(),
            strong_count,
            LocalSearchAction::Strong);
        std::shuffle(
            matched_actions.begin(),
            matched_actions.end(),
            allocation_engine);
    }

    for (std::size_t batch_start = 0;
         batch_start < search_candidates.size();
         batch_start += kWorkspaceBatchSize) {
        const std::size_t batch_size = std::min(
            kWorkspaceBatchSize,
            search_candidates.size() - batch_start);
        const std::size_t record_start = run.records.size();

        for (std::size_t local_index = 0;
             local_index < batch_size;
             ++local_index) {
            AllocatedLocalSearchRecord record;
            record.individual =
                search_candidates[batch_start + local_index];
            record.cost_before_weak =
                record.individual->upper_cost;
            record.strong_move_limit =
                MaLeader::move_limit_for_action(
                    *record.individual,
                    instance,
                    LocalSearchAction::Strong,
                    depth_config);
            MaLeader::begin_eight_neighborhood_rvnd_one_move_session(
                *record.individual,
                record.session,
                workspaces[local_index]);
            const int weak_limit =
                MaLeader::move_limit_for_action(
                    *record.individual,
                    instance,
                    LocalSearchAction::Weak,
                    depth_config);
            record.weak_result =
                MaLeader::continue_eight_neighborhood_rvnd_one_move_session(
                    *record.individual,
                    instance,
                    local_search_engine,
                    record.session,
                    weak_limit,
                    workspaces[local_index],
                    trigger_upper_bound);
            record.cost_after_weak =
                record.individual->upper_cost;
            record.strong_distance_call_limit =
                MaLeader::strong_distance_call_limit(
                    record.weak_result.distance_calls_used,
                    depth_config);
            record.cost_after_terminal = record.cost_after_weak;
            record.total_result = record.weak_result;
            if (policy == LocalSearchPolicy::OnlineIndividual) {
                record.context = make_context(
                    *record.individual,
                    upper_reference,
                    trigger_upper_bound,
                    budget_progress,
                    upper_stagnation,
                    lower_stagnation,
                    population_dispersion,
                    recent_gamma_rate,
                    record.weak_result,
                    evaluation_limit_distance_calls,
                    population.size());
            }
            run.records.push_back(std::move(record));
        }

        if (policy == LocalSearchPolicy::MatchedRandom
            || policy == LocalSearchPolicy::OnlineIndividual) {
            for (std::size_t local_index = 0;
                 local_index < batch_size;
                 ++local_index) {
                auto& record =
                    run.records[record_start + local_index];
                if (record.weak_result.reached_local_optimum) {
                    record.terminal_action =
                        LocalSearchAction::Weak;
                    record.forced_local_optimum = true;
                    finish_record(record, trigger_upper_bound);
                    continue;
                }

                LocalSearchActionDecision decision;
                if (policy == LocalSearchPolicy::MatchedRandom) {
                    decision.action =
                        matched_actions[batch_start + local_index];
                } else {
                    decision = learner.select(
                        record.context,
                        generation,
                        allocation_engine);
                }
                record.terminal_action = decision.action;
                record.selection_score = decision.score;
                record.exploratory_selection =
                    decision.exploratory;

                if (decision.action
                    != LocalSearchAction::Weak) {
                    const bool use_strong =
                        decision.action
                        == LocalSearchAction::Strong;
                    const int move_limit = use_strong
                        ? record.strong_move_limit
                        : MaLeader::move_limit_for_action(
                            *record.individual,
                            instance,
                            LocalSearchAction::Medium,
                            depth_config);
                    record.continuation_result =
                        MaLeader::continue_eight_neighborhood_rvnd_one_move_session(
                            *record.individual,
                            instance,
                            local_search_engine,
                            record.session,
                            move_limit,
                            workspaces[local_index],
                            trigger_upper_bound,
                            use_strong
                                ? record.strong_distance_call_limit
                                : std::numeric_limits<
                                    std::uint64_t>::max());
                    record.total_result = combine_results(
                        record.weak_result,
                        record.continuation_result,
                        record.cost_before_weak,
                        record.individual->upper_cost);
                }
                finish_record(record, trigger_upper_bound);
            }
            continue;
        }

        std::vector<std::size_t> medium_eligible;
        for (std::size_t local_index = 0;
             local_index < batch_size;
             ++local_index) {
            const std::size_t record_index =
                record_start + local_index;
            if (!run.records[record_index]
                     .weak_result.reached_local_optimum) {
                medium_eligible.push_back(record_index);
            }
        }
        std::shuffle(
            medium_eligible.begin(),
            medium_eligible.end(),
            allocation_engine);
        const std::size_t medium_target = std::min(
            medium_eligible.size(),
            static_cast<std::size_t>(std::lround(
                static_cast<double>(batch_size)
                * kMediumSelectionRatio)));
        medium_eligible.resize(medium_target);

        std::vector<std::size_t> strong_selected =
            medium_eligible;
        std::shuffle(
            strong_selected.begin(),
            strong_selected.end(),
            allocation_engine);
        const std::size_t strong_target = std::min(
            strong_selected.size(),
            static_cast<std::size_t>(std::lround(
                static_cast<double>(batch_size)
                * kStrongSelectionRatio)));
        strong_selected.resize(strong_target);

        for (const std::size_t record_index : medium_eligible) {
            const std::size_t local_index =
                record_index - record_start;
            auto& record = run.records[record_index];
            const bool use_strong =
                std::find(
                    strong_selected.begin(),
                    strong_selected.end(),
                    record_index)
                != strong_selected.end();
            record.terminal_action = use_strong
                ? LocalSearchAction::Strong
                : LocalSearchAction::Medium;
            const int move_limit = use_strong
                ? record.strong_move_limit
                : MaLeader::move_limit_for_action(
                    *record.individual,
                    instance,
                    LocalSearchAction::Medium,
                    depth_config);
            record.continuation_result =
                MaLeader::continue_eight_neighborhood_rvnd_one_move_session(
                    *record.individual,
                    instance,
                    local_search_engine,
                    record.session,
                    move_limit,
                    workspaces[local_index],
                    trigger_upper_bound,
                    use_strong
                        ? record.strong_distance_call_limit
                        : std::numeric_limits<
                            std::uint64_t>::max());
            record.total_result = combine_results(
                record.weak_result,
                record.continuation_result,
                record.cost_before_weak,
                record.individual->upper_cost);
        }
        for (std::size_t local_index = 0;
             local_index < batch_size;
             ++local_index) {
            finish_record(
                run.records[record_start + local_index],
                trigger_upper_bound);
        }
    }

    aggregate_run_operator_stats(run);
    return run;
}

void LocalSearchAllocationRunner::assign_parent_use_feedback(
    LocalSearchAllocationRun& run,
    const std::vector<ParentCandidate>& parent_pool,
    const std::vector<int>& parent_use_counts) {
    if (parent_pool.size() != parent_use_counts.size()) {
        throw std::invalid_argument(
            "parent use counts must match parent pool size");
    }
    int total_parent_uses = 0;
    std::size_t scored_parent_count = 0;
    for (std::size_t parent_index = 0;
         parent_index < parent_pool.size();
         ++parent_index) {
        const auto record = std::find_if(
            run.records.begin(),
            run.records.end(),
            [&](const AllocatedLocalSearchRecord& candidate) {
                return candidate.individual.get()
                    == parent_pool[parent_index].source;
            });
        if (record != run.records.end()
            && record->exclude_from_learner_feedback) {
            continue;
        }
        total_parent_uses += parent_use_counts[parent_index];
        ++scored_parent_count;
    }
    if (total_parent_uses <= 0 || scored_parent_count == 0) {
        return;
    }
    const double average_parent_uses =
        static_cast<double>(total_parent_uses)
        / static_cast<double>(scored_parent_count);
    for (std::size_t parent_index = 0;
         parent_index < parent_pool.size();
         ++parent_index) {
        if (parent_use_counts[parent_index] <= 0
            || parent_pool[parent_index].source == nullptr) {
            continue;
        }
        const auto record = std::find_if(
            run.records.begin(),
            run.records.end(),
            [&](const AllocatedLocalSearchRecord& candidate) {
                return candidate.individual.get()
                    == parent_pool[parent_index].source;
            });
        if (record == run.records.end()
            || record->exclude_from_learner_feedback) {
            continue;
        }
        record->parent_use_count += parent_use_counts[parent_index];
        record->parent_credit = std::min(
            1.0,
            record->parent_credit
                + static_cast<double>(parent_use_counts[parent_index])
                    / average_parent_uses);
    }
}

std::vector<std::pair<const Individual*, double>>
LocalSearchAllocationRunner::assign_lower_archive_feedback(
    LocalSearchAllocationRun& run,
    const std::vector<std::shared_ptr<Individual>>& complete_solutions,
    OnlineActionLearner& learner) {
    const auto credits = learner.update_lower_archive(
        complete_solutions);
    for (const auto& credit_entry : credits) {
        const Individual* individual = credit_entry.first;
        const double credit = credit_entry.second;
        const auto record = std::find_if(
            run.records.begin(),
            run.records.end(),
            [&](const AllocatedLocalSearchRecord& candidate) {
                return candidate.individual.get() == individual;
            });
        if (record != run.records.end()
            && !record->exclude_from_learner_feedback) {
            record->lower_credit += credit;
        }
    }
    return credits;
}

void LocalSearchAllocationRunner::finalize_feedback(
    LocalSearchAllocationRun& run,
    LocalSearchPolicy policy,
    OnlineActionLearner& learner) {
    for (auto& record : run.records) {
        // The allocation action starts after the common weak probe.
        const double continuation_relative_gain =
            record.cost_after_weak > 0.0
            ? std::max(
                0.0,
                (record.cost_after_weak
                    - record.cost_after_terminal)
                    / record.cost_after_weak)
            : 0.0;
        record.normalized_continuation_gain =
            bounded_nonnegative(
                kContinuationGainScale
                    * continuation_relative_gain);
        const double parent_reward =
            kParentRewardWeight
            * std::clamp(record.parent_credit, 0.0, 1.0);
        const double lower_reward =
            kLowerRewardWeight
            * std::clamp(record.lower_credit, 0.0, 1.0);
        const double gamma_reward =
            record.post_probe_gamma_cross
            ? kGammaRewardWeight
            : 0.0;
        const double continuation_gain_signal =
            kContinuationGainSignalWeight
            * record.normalized_continuation_gain;
        record.reward =
            parent_reward
            + lower_reward
            + gamma_reward;

        const std::uint64_t weak_distance_calls =
            std::max<std::uint64_t>(
                1,
                record.weak_result.distance_calls_used);
        record.incremental_cost_units =
            static_cast<double>(
                record.continuation_result.distance_calls_used)
            / static_cast<double>(weak_distance_calls);
        if (policy == LocalSearchPolicy::OnlineIndividual
            && !record.exclude_from_learner_feedback) {
            learner.update(
                record.context,
                record.terminal_action,
                record.reward,
                record.incremental_cost_units);
        }

        auto& stats = run.stats[
            action_index(record.terminal_action)];
        ++stats.selections;
        stats.forced_local_optima += record.forced_local_optimum;
        stats.exploratory_selections +=
            record.exploratory_selection;
        stats.local_optimum_terminations +=
            record.total_result.reached_local_optimum;
        stats.move_limit_terminations +=
            record.total_result.hit_move_limit;
        stats.distance_limit_terminations +=
            record.total_result.hit_distance_call_limit;
        stats.accepted_moves +=
            record.total_result.accepted_moves;
        stats.neighborhood_calls +=
            record.total_result.neighborhood_calls;
        stats.distance_calls +=
            record.total_result.distance_calls_used;
        stats.continuation_distance_calls +=
            record.continuation_result.distance_calls_used;
        stats.upper_gain +=
            result_upper_gain(record.total_result);
        stats.gamma_crosses += record.crossed_gamma;
        stats.parent_uses += record.parent_use_count;
        stats.lower_archive_entries +=
            record.lower_credit > 0.0;
        stats.parent_reward += parent_reward;
        stats.lower_reward += lower_reward;
        stats.gamma_reward += gamma_reward;
        stats.continuation_gain_signal +=
            continuation_gain_signal;
        stats.reward += record.reward;
        stats.incremental_cost_units +=
            record.incremental_cost_units;
        stats.selection_score += record.selection_score;
    }
}

std::size_t LocalSearchAllocationRunner::action_index(
    LocalSearchAction action) {
    switch (action) {
        case LocalSearchAction::Weak:
            return 0;
        case LocalSearchAction::Medium:
            return 1;
        case LocalSearchAction::Strong:
            return 2;
    }
    throw std::logic_error("unknown local-search action");
}

const char* local_search_policy_name(LocalSearchPolicy policy) {
    switch (policy) {
        case LocalSearchPolicy::Static:
            return "static";
        case LocalSearchPolicy::RandomMixed:
            return "random";
        case LocalSearchPolicy::MatchedRandom:
            return "matched_random";
        case LocalSearchPolicy::OnlineIndividual:
            return "online";
    }
    return "unknown";
}
