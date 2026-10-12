//
// Created by Yinghao Qin on 11/10/2026.
//
// Elite Unrestricted: a post-paid credit account (unrestricted_credit_rho x ordinary local-search distance
// calls) that lets at most one unfinished strong-action individual per generation continue
// with unrestricted RVND (MaLeader::improve_unrestricted).
//

#ifndef MA_UNRESTRICTED_HPP
#define MA_UNRESTRICTED_HPP

#include <cstdint>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "ma_leader.hpp"

class Case;
class Individual;
struct LocalSearchAllocationRun;
struct ParentCandidate;

struct EliteUnrestrictedStats {
    int eligible_candidates{};
    int triggers{};
    std::uint64_t normal_distance_calls{};
    std::uint64_t distance_calls{};
    int accepted_moves{};
    int neighborhood_calls{};
    double upper_gain{};
    int gamma_crosses{};
    int parent_uses{};
    int lower_archive_entries{};
    int verified_improvements{};
    long double ending_credit_distance_calls{};
};

struct EliteUnrestrictedRun {
    std::shared_ptr<Individual> individual;
    LocalSearchResult result;
    int eligible_candidates{};
    double upper_cost_before{};
    double upper_cost_after{};
    int parent_uses{};
    int lower_archive_entries{};
    int verified_improvements{};
    bool triggered{};
    bool crossed_gamma{};
};

class EliteUnrestrictedController {
public:
    static constexpr int WARMUP_GENERATIONS = 10;
    static constexpr double DEFAULT_CREDIT_RHO = 0.10;

    void reset();
    void set_credit_rho(double credit_rho);
    [[nodiscard]] double credit_rho() const;

    EliteUnrestrictedRun run(
        LocalSearchAllocationRun& allocation_run,
        Case& instance,
        int generation,
        std::uint64_t normal_distance_calls,
        double trigger_upper_bound,
        std::mt19937& random_engine,
        LocalSearchWorkspace& workspace);

    static void assign_parent_use_feedback(
        EliteUnrestrictedRun& run,
        const std::vector<ParentCandidate>& parent_pool,
        const std::vector<int>& parent_use_counts);
    static void assign_lower_archive_feedback(
        EliteUnrestrictedRun& run,
        const std::vector<std::pair<const Individual*, double>>& credits);
    static EliteUnrestrictedStats make_stats(
        const EliteUnrestrictedRun& run,
        std::uint64_t normal_distance_calls,
        long double ending_credit_distance_calls);

    [[nodiscard]] long double credit_distance_calls() const;

private:
    long double credit_balance_distance_calls{};
    double credit_rho_value{DEFAULT_CREDIT_RHO};
};

#endif //MA_UNRESTRICTED_HPP
