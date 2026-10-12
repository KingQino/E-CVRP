//
// Created by Yinghao Qin on 11/10/2026.
//

#include "ma_unrestricted.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "case.hpp"
#include "individual.hpp"
#include "ma_local_search_allocation.hpp"
#include "ma_reproduction.hpp"

namespace {

double result_upper_gain(const LocalSearchResult& result) {
    double gain = 0.0;
    for (const auto& stats : result.operator_stats) {
        gain += stats.upper_gain;
    }
    return gain;
}

int result_gamma_crosses(const LocalSearchResult& result) {
    int crosses = 0;
    for (const auto& stats : result.operator_stats) {
        crosses += stats.gamma_crosses;
    }
    return crosses;
}

}  // namespace

void EliteUnrestrictedController::reset() {
    credit_balance_distance_calls = 0.0L;
}

void EliteUnrestrictedController::set_credit_rho(double new_credit_rho) {
    if (!std::isfinite(new_credit_rho)
        || new_credit_rho < 0.0
        || new_credit_rho > 1.0) {
        throw std::invalid_argument(
            "unrestricted credit rho must be in [0, 1]");
    }
    credit_rho_value = new_credit_rho;
}

double EliteUnrestrictedController::credit_rho() const {
    return credit_rho_value;
}

EliteUnrestrictedRun EliteUnrestrictedController::run(
    LocalSearchAllocationRun& allocation_run,
    Case& instance,
    int generation,
    std::uint64_t normal_distance_calls,
    double trigger_upper_bound,
    std::mt19937& random_engine,
    LocalSearchWorkspace& workspace) {
    credit_balance_distance_calls +=
        static_cast<long double>(credit_rho_value)
        * static_cast<long double>(normal_distance_calls);

    EliteUnrestrictedRun run;
    auto best_candidate = allocation_run.records.end();
    for (auto record = allocation_run.records.begin();
         record != allocation_run.records.end();
         ++record) {
        const bool eligible =
            record->terminal_action
                == LocalSearchAction::Strong
            && !record->total_result.reached_local_optimum;
        if (!eligible) {
            continue;
        }
        ++run.eligible_candidates;
        if (best_candidate == allocation_run.records.end()
            || record->individual->upper_cost
                < best_candidate->individual->upper_cost) {
            best_candidate = record;
        }
    }

    if (generation <= WARMUP_GENERATIONS
        || credit_balance_distance_calls <= 0.0L
        || best_candidate == allocation_run.records.end()) {
        return run;
    }

    best_candidate->exclude_from_learner_feedback = true;
    run.individual = best_candidate->individual;
    run.upper_cost_before = run.individual->upper_cost;
    run.result =
        MaLeader::improve_unrestricted(
            *run.individual,
            instance,
            random_engine,
            workspace,
            trigger_upper_bound);
    run.upper_cost_after = run.individual->upper_cost;
    run.triggered = true;
    run.crossed_gamma =
        run.upper_cost_before > trigger_upper_bound
        && run.upper_cost_after <= trigger_upper_bound;
    credit_balance_distance_calls -=
        static_cast<long double>(run.result.distance_calls_used);
    return run;
}

void EliteUnrestrictedController::assign_parent_use_feedback(
    EliteUnrestrictedRun& run,
    const std::vector<ParentCandidate>& parent_pool,
    const std::vector<int>& parent_use_counts) {
    if (!run.triggered) {
        return;
    }
    if (parent_pool.size() != parent_use_counts.size()) {
        throw std::invalid_argument(
            "parent use counts must match parent pool size");
    }
    for (std::size_t index = 0; index < parent_pool.size(); ++index) {
        if (parent_pool[index].source == run.individual.get()) {
            run.parent_uses += parent_use_counts[index];
        }
    }
}

void EliteUnrestrictedController::assign_lower_archive_feedback(
    EliteUnrestrictedRun& run,
    const std::vector<std::pair<const Individual*, double>>& credits) {
    if (!run.triggered) {
        return;
    }
    run.lower_archive_entries += static_cast<int>(std::count_if(
        credits.begin(),
        credits.end(),
        [&](const auto& entry) {
            return entry.first == run.individual.get()
                && entry.second > 0.0;
        }));
}

EliteUnrestrictedStats EliteUnrestrictedController::make_stats(
    const EliteUnrestrictedRun& run,
    std::uint64_t normal_distance_calls,
    long double ending_credit_distance_calls) {
    EliteUnrestrictedStats stats;
    stats.eligible_candidates = run.eligible_candidates;
    stats.triggers = run.triggered;
    stats.normal_distance_calls = normal_distance_calls;
    stats.distance_calls = run.result.distance_calls_used;
    stats.accepted_moves = run.result.accepted_moves;
    stats.neighborhood_calls = run.result.neighborhood_calls;
    stats.upper_gain = result_upper_gain(run.result);
    stats.gamma_crosses = result_gamma_crosses(run.result);
    stats.parent_uses = run.parent_uses;
    stats.lower_archive_entries = run.lower_archive_entries;
    stats.verified_improvements = run.verified_improvements;
    stats.ending_credit_distance_calls = ending_credit_distance_calls;
    return stats;
}

long double EliteUnrestrictedController::credit_distance_calls() const {
    return credit_balance_distance_calls;
}
