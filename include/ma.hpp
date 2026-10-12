//
// Created by Yinghao Qin on 11/10/2026.
//
// Budget-aware bilevel memetic algorithm (BA-BMA, "MA" in the code) with online local-search
// action allocation and Elite Unrestricted.
//
// Notation (as in the BA-BMA paper):
//   x*     best complete solution found so far (verified_best)
//   x_g^E  best complete solution obtained in generation g (lower_elite in run_generation)
//   P_U    parent pool;  f_U  upper-level cost;  d_J  multiset Jaccard distance of adjacency signatures
//
// One generation:
//   1. upper-level LS on the population: every individual gets the weak probe and an allocated
//      action (weak / medium / strong) decides where it stops; the online policy adds Elite
//      Unrestricted (unrestricted local search for one candidate). The static policy applies one action to
//      all individuals, or unrestricted local search as the original BMA baseline;
//   2. parent pool P_U: up to ceil(alpha * N) structurally distinct solutions with the lowest f_U;
//   3. follower on every individual with f_U <= gamma * best f_U so far; updates x_g^E and x*;
//   4. learner feedback (lower archive, gamma crossing, parent usage);
//   5. next population (default N = 100): 1 x_g^E + 84 P_U x P_U + 5 x* x P_U + 10 immigrants.
//      P_U x P_U: first parent by binary tournament on f_U, second parent with the largest d_J to it
//      among up to 8 randomly sampled members of P_U (ties by f_U); x* x P_U: x* mated with members
//      of P_U drawn at random. PMX, shuffle-index mutation (p_m, p_g), Prins split.
//      If x_g^E does not exist, x* fills the elite slot; with no complete solution yet, the slot
//      becomes one more P_U x P_U offspring.
//
// Shared E-CVRP components are used for the initial population (Hien clustering + balancing),
// the giant-tour split (Prins) and the follower (SE/RE, final refine).
//

#ifndef MA_HPP
#define MA_HPP

#include "case.hpp"
#include "follower.hpp"
#include "heuristic_interface.hpp"
#include "individual.hpp"
#include "initializer.hpp"
#include "ma_unrestricted.hpp"
#include "ma_leader.hpp"
#include "ma_local_search_allocation.hpp"
#include "ma_reproduction.hpp"
#include "stats_interface.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <random>
#include <vector>

// Partition of the distance calls of a generation window (search-budget.tsv).
struct SearchBudgetStats {
    std::uint64_t normal_local_search_distance_calls{};
    std::uint64_t elite_local_search_distance_calls{};
    std::uint64_t follower_distance_calls{};
    std::uint64_t other_distance_calls{};
    std::uint64_t total_distance_calls{};
    int follower_candidates{};
    int follower_runs{};
};

class Ma final : public HeuristicInterface, public StatsInterface {
public:
    static constexpr int kLogInterval = 10;    // generations aggregated per TSV snapshot row

    bool enable_logging;                        // Whether to enable logging
    int stop_criteria;                          // 0: max evals, 1: max time

    // Parameters
    int pop_size;
    double mutation_prob;
    double mutation_ind_prob;
    int tournament_size;
    LocalSearchPolicy ls_policy;
    LocalSearchAction static_action;            // action of the static policy
    bool static_unrestricted;                   // static policy uses unrestricted RVND instead
    LocalSearchDepthProfile ls_depth_profile;
    LocalSearchDepthConfig ls_depth_config;
    double action_cost_weight;
    double unrestricted_credit_rho;
    double parent_pool_ratio;
    double verified_upper_ratio;                // share of x* x P_U offspring
    double pure_immigrant_ratio;
    double gamma;                               // follower trigger ratio (params.low_opt_trigger_margin)
    int route_cap;                              // route capacity of MA individuals (3 x vehicles, as in the original MA)
    int node_cap;                               // node capacity of MA individuals (customers + 2 depots)

    // Search state
    int generation;
    double global_best_upper_cost;
    std::uint64_t evaluation_limit_distance_calls;
    std::uint64_t last_upper_improvement_distance_calls;
    std::uint64_t last_lower_improvement_distance_calls;
    double recent_gamma_entry_rate;

    std::mt19937 local_search_engine;           // RVND operator/route choices
    std::mt19937 allocation_engine;             // action allocation choices
    std::vector<std::shared_ptr<Individual>> population;
    std::vector<std::shared_ptr<Individual>> population_buffer;
    std::unique_ptr<Individual> verified_best;          // x*: best complete solution (routes + stations) found so far
    std::unique_ptr<Individual> upper_best;             // best upper-level solution ever seen
    std::shared_ptr<Individual> retained_lower_elite;   // elite slot of the current population: x_g^E of the previous
                                                        // generation (or a copy of x*), carried over unchanged

    Initializer* initializer;
    Follower* follower;

    ReproductionWorkspace reproduction_workspace;
    LocalSearchWorkspace local_search_workspace;
    LocalSearchWorkspace elite_unrestricted_workspace;
    std::vector<LocalSearchWorkspace> allocation_workspaces;
    std::vector<std::vector<int>> split_routes;
    OnlineActionLearner action_learner;
    EliteUnrestrictedController elite_unrestricted_controller;

    Ma(int seed, Case* instance, Preprocessor* preprocessor);
    ~Ma() override;

    double run() override;
    void initialize_heuristic() override;
    void run_heuristic() override;
    void run_generation();

    void open_log_for_evolution() override;
    void close_log_for_evolution() override;
    void flush_row_into_evol_log() override;
    void save_log_for_solution() override;

private:
    // Logs
    std::ostringstream oss_operator_rows;
    std::ostringstream oss_allocation_rows;
    std::ostringstream oss_elite_rows;
    std::ostringstream oss_budget_rows;
    std::ofstream log_operators;
    std::ofstream log_allocation;
    std::ofstream log_elite;
    std::ofstream log_budget;
    std::array<LocalSearchAllocationStats, 3> pending_allocation_stats{};
    int pending_allocation_generations{};
    EliteUnrestrictedStats pending_elite_stats{};
    int pending_elite_generations{};
    SearchBudgetStats pending_budget_stats{};
    int pending_budget_generations{};

    [[nodiscard]] bool within_budget() const;
    [[nodiscard]] bool elite_unrestricted_enabled() const;
    [[nodiscard]] double calls_to_evals(std::uint64_t distance_calls) const;
    [[nodiscard]] std::filesystem::path log_directory() const;
    void initialize_population();
    [[nodiscard]] std::shared_ptr<Individual> make_individual(const std::vector<std::vector<int>>& routes);
    void check_capacity(const std::vector<std::vector<int>>& routes) const;
    void load_offspring(Individual& offspring, const std::vector<int>& chromosome);
    void run_final_follower();

    void open_log_for_local_search();
    void close_log_for_local_search();
    void flush_local_search_log();
    void write_allocation_snapshot();
    void write_elite_snapshot();
    void write_budget_snapshot();
};

#endif //MA_HPP
