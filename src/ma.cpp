//
// Created by Yinghao Qin on 11/10/2026.
//

#include "ma.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace fs = std::filesystem;

namespace {

void add_operator_stats(std::array<LocalSearchOperatorStats, LOCAL_SEARCH_OPERATOR_COUNT>& destination,
                        const LocalSearchResult& result) {
    for (std::size_t i = 0; i < LOCAL_SEARCH_OPERATOR_COUNT; ++i) {
        destination[i].calls += result.operator_stats[i].calls;
        destination[i].accepts += result.operator_stats[i].accepts;
        destination[i].distance_calls += result.operator_stats[i].distance_calls;
        destination[i].upper_gain += result.operator_stats[i].upper_gain;
        destination[i].gamma_crosses += result.operator_stats[i].gamma_crosses;
    }
}

void add_allocation_stats(LocalSearchAllocationStats& destination, const LocalSearchAllocationStats& source) {
    destination.selections += source.selections;
    destination.forced_local_optima += source.forced_local_optima;
    destination.exploratory_selections += source.exploratory_selections;
    destination.local_optimum_terminations += source.local_optimum_terminations;
    destination.move_limit_terminations += source.move_limit_terminations;
    destination.distance_limit_terminations += source.distance_limit_terminations;
    destination.accepted_moves += source.accepted_moves;
    destination.neighborhood_calls += source.neighborhood_calls;
    destination.distance_calls += source.distance_calls;
    destination.continuation_distance_calls += source.continuation_distance_calls;
    destination.upper_gain += source.upper_gain;
    destination.gamma_crosses += source.gamma_crosses;
    destination.parent_uses += source.parent_uses;
    destination.lower_archive_entries += source.lower_archive_entries;
    destination.parent_reward += source.parent_reward;
    destination.lower_reward += source.lower_reward;
    destination.gamma_reward += source.gamma_reward;
    destination.continuation_gain_signal += source.continuation_gain_signal;
    destination.reward += source.reward;
    destination.incremental_cost_units += source.incremental_cost_units;
    destination.selection_score += source.selection_score;
}

void add_elite_stats(EliteUnrestrictedStats& destination, const EliteUnrestrictedStats& source) {
    destination.eligible_candidates += source.eligible_candidates;
    destination.triggers += source.triggers;
    destination.normal_distance_calls += source.normal_distance_calls;
    destination.distance_calls += source.distance_calls;
    destination.accepted_moves += source.accepted_moves;
    destination.neighborhood_calls += source.neighborhood_calls;
    destination.upper_gain += source.upper_gain;
    destination.gamma_crosses += source.gamma_crosses;
    destination.parent_uses += source.parent_uses;
    destination.lower_archive_entries += source.lower_archive_entries;
    destination.verified_improvements += source.verified_improvements;
    destination.ending_credit_distance_calls = source.ending_credit_distance_calls;
}

void add_budget_stats(SearchBudgetStats& destination, const SearchBudgetStats& source) {
    destination.normal_local_search_distance_calls += source.normal_local_search_distance_calls;
    destination.elite_local_search_distance_calls += source.elite_local_search_distance_calls;
    destination.follower_distance_calls += source.follower_distance_calls;
    destination.other_distance_calls += source.other_distance_calls;
    destination.total_distance_calls += source.total_distance_calls;
    destination.follower_candidates += source.follower_candidates;
    destination.follower_runs += source.follower_runs;
}

std::uint64_t allocation_distance_calls(const LocalSearchAllocationRun& run) {
    std::uint64_t distance_calls = 0;
    for (const auto& record : run.records) {
        distance_calls += record.total_result.distance_calls_used;
    }
    return distance_calls;
}

} // namespace

Ma::Ma(const int seed, Case* instance, Preprocessor* preprocessor)
    : HeuristicInterface("MA", seed, instance, preprocessor) {
    const Parameters& params = preprocessor->params;
    enable_logging = params.enable_logging;
    stop_criteria = params.stop_criteria;

    pop_size = params.ma_pop_size;
    mutation_prob = params.ma_mutation_prob;
    mutation_ind_prob = params.ma_mutation_ind_prob;
    tournament_size = params.ma_tournament_size;
    static_action = params.ma_static_action;
    static_unrestricted = params.ma_static_unrestricted;
    ls_policy = params.ma_ls_policy;
    ls_depth_profile = params.ma_ls_depth;
    ls_depth_config = local_search_depth_config(params.ma_ls_depth);
    action_cost_weight = params.ma_action_cost_weight;
    unrestricted_credit_rho = params.ma_unrestricted_credit_rho;
    parent_pool_ratio = params.ma_parent_pool_ratio;
    verified_upper_ratio = params.ma_verified_upper_ratio;
    pure_immigrant_ratio = params.ma_pure_immigrant_ratio;
    gamma = params.low_opt_trigger_margin;
    route_cap = std::min(preprocessor->route_cap_, 3 * instance->num_vehicle_);
    node_cap = std::min(preprocessor->node_cap_, instance->num_customer_ + 2);

    // Independent streams for local search and allocation, as in the original MA.
    std::seed_seq local_search_seed{seed, 0x4C53, 0x52564E44};
    local_search_engine.seed(local_search_seed);
    std::seed_seq allocation_seed{seed, 0x4C53, 0x414C4C4F};
    allocation_engine.seed(allocation_seed);

    action_learner.set_action_cost_weight(action_cost_weight);
    elite_unrestricted_controller.set_credit_rho(unrestricted_credit_rho);

    generation = 0;
    global_best_upper_cost = std::numeric_limits<double>::max();
    evaluation_limit_distance_calls =
        static_cast<std::uint64_t>(preprocessor->max_evals_) * static_cast<std::uint64_t>(instance->problem_size_);
    last_upper_improvement_distance_calls = 0;
    last_lower_improvement_distance_calls = 0;
    recent_gamma_entry_rate = 0.0;

    initializer = new Initializer(random_engine, instance, preprocessor);
    follower = new Follower(instance, preprocessor);
    follower->se_compare_station_counts = true;  // MA's SE compares l_min and l_min + 1 stations
}

Ma::~Ma() {
    population.clear();
    population_buffer.clear();
    retained_lower_elite.reset();
    delete initializer;
    delete follower;
}

double Ma::run() {
    start = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration<double>::zero();

    if (enable_logging) {
        open_log_for_evolution();
        open_log_for_local_search();
    }

    if (preprocessor->params.enable_stn_logging) {
        stn_logger = std::make_unique<StnLogger>(*instance, log_directory(), evaluation_limit_distance_calls,
                                                 preprocessor->params.stn_grid, preprocessor->params.stn_solutions);
    }

    initialize_heuristic();
    run_heuristic();
    if (stn_logger) {
        stn_logger->finish();  // last generation, before the final follower (which does not change f_U)
        stn_logger.reset();
    }
    run_final_follower();

    if (enable_logging) {
        close_log_for_evolution();
        close_log_for_local_search();
        save_log_for_solution();
    }

    return verified_best->lower_cost;
}

bool Ma::within_budget() const {
    return stop_criteria == 0 ? !stop_criteria_max_evals() : !stop_criteria_max_exec_time(duration);
}

bool Ma::elite_unrestricted_enabled() const {
    return ls_policy == LocalSearchPolicy::OnlineIndividual;
}

double Ma::calls_to_evals(const std::uint64_t distance_calls) const {
    return instance->problem_size_ > 0
        ? static_cast<double>(distance_calls) / static_cast<double>(instance->problem_size_)
        : 0.0;
}

void Ma::check_capacity(const std::vector<std::vector<int>>& routes) const {
    if (routes.size() > static_cast<std::size_t>(route_cap)) {
        throw std::length_error("MA solution exceeds the route capacity (3 x vehicles)");
    }
}

std::shared_ptr<Individual> Ma::make_individual(const std::vector<std::vector<int>>& routes) {
    check_capacity(routes);
    auto individual = std::make_shared<Individual>(instance, preprocessor, route_cap, node_cap);
    individual->load_routes(routes, instance->compute_total_distance(routes), instance->compute_demand_sum_per_route(routes));
    return individual;
}

void Ma::initialize_population() {
    for (int i = 0; i < pop_size; ++i) {
        population.push_back(make_individual(initializer->routes_constructor_with_hien_method()));
    }
}

void Ma::initialize_heuristic() {
    action_learner.reset();
    elite_unrestricted_controller.reset();
    pending_allocation_stats = {};
    pending_allocation_generations = 0;
    pending_elite_stats = {};
    pending_elite_generations = 0;
    pending_budget_stats = {};
    pending_budget_generations = 0;
    generation = 0;
    retained_lower_elite.reset();
    population.clear();
    population_buffer.clear();
    population.reserve(static_cast<std::size_t>(pop_size));
    population_buffer.reserve(static_cast<std::size_t>(pop_size));
    allocation_workspaces.clear();

    initialize_population();

    verified_best = std::make_unique<Individual>(instance, preprocessor, route_cap, node_cap);
    verified_best->upper_cost = INFEASIBLE;
    verified_best->lower_cost = INFEASIBLE;

    const std::shared_ptr<Individual> initial_best = MaReproduction::best_by_upper_cost(population);
    upper_best = std::make_unique<Individual>(*initial_best);
    global_best_upper_cost = upper_best->upper_cost;

    last_upper_improvement_distance_calls = instance->get_distance_calls();
    last_lower_improvement_distance_calls = instance->get_distance_calls();
    recent_gamma_entry_rate = 0.0;
}

void Ma::run_heuristic() {
    duration = std::chrono::high_resolution_clock::now() - start;
    while (within_budget()) {
        run_generation();
        duration = std::chrono::high_resolution_clock::now() - start;
        if (enable_logging) {
            flush_row_into_evol_log();
        }
    }
}

void Ma::run_generation() {
    ++generation;

    std::shared_ptr<Individual> unchanged_lower_elite = retained_lower_elite;
    LocalSearchAllocationRun allocation_run;
    EliteUnrestrictedRun elite_run;
    std::uint64_t normal_ls_distance_calls = 0;
    std::array<LocalSearchOperatorStats, LOCAL_SEARCH_OPERATOR_COUNT> generation_operator_stats{};

    // The upper reference is frozen for the whole generation.
    const ParentCandidate frozen_upper_reference = MaReproduction::make_parent_candidate(*upper_best);
    const double frozen_trigger_upper_bound = frozen_upper_reference.upper_cost * gamma;
    const std::uint64_t generation_start_distance_calls = instance->get_distance_calls();
    const auto budget_fraction = [&](const std::uint64_t calls) {
        return evaluation_limit_distance_calls > 0
            ? static_cast<double>(calls) / static_cast<double>(evaluation_limit_distance_calls)
            : 0.0;
    };
    const double budget_progress = budget_fraction(generation_start_distance_calls);
    const double upper_stagnation = budget_fraction(generation_start_distance_calls - last_upper_improvement_distance_calls);
    const double lower_stagnation = budget_fraction(generation_start_distance_calls - last_lower_improvement_distance_calls);
    double population_dispersion = 0.0;
    if (ls_policy == LocalSearchPolicy::OnlineIndividual && !population.empty()) {
        for (const auto& individual : population) {
            population_dispersion += MaReproduction::adjacency_distance(
                MaReproduction::make_parent_candidate(*individual), frozen_upper_reference);
        }
        population_dispersion /= static_cast<double>(population.size());
    }

    // === 1. Upper-level local search ===
    if (ls_policy == LocalSearchPolicy::Static) {
        // STN representative: best individual of this generation among those whose local search is complete,
        // observed after every individual (the random initial population and unsearched offspring are never logged).
        const Individual* stn_best = nullptr;
        const auto stn_observe = [&](const Individual& searched) {
            if (!stn_logger) return;
            const bool improved = stn_best == nullptr || searched.upper_cost < stn_best->upper_cost;
            if (improved) stn_best = &searched;
            if (improved || stn_logger->due(instance->get_distance_calls())) {
                stn_logger->offer(*stn_best, generation, instance->get_distance_calls());
            }
        };
        for (const auto& individual : population) {
            const bool can_reuse_lower_elite = individual == unchanged_lower_elite
                && individual->upper_locally_optimal
                && individual->lower_cost < INFEASIBLE;
            if (can_reuse_lower_elite) {
                stn_observe(*individual);
                continue;
            }

            const LocalSearchResult result = static_unrestricted
                ? MaLeader::improve_unrestricted(*individual, *instance, local_search_engine,
                                                 local_search_workspace, frozen_trigger_upper_bound)
                : MaLeader::improve_with_eight_neighborhood_rvnd_one_move(
                      *individual, *instance, local_search_engine, static_action, local_search_workspace,
                      frozen_trigger_upper_bound, ls_depth_config);
            add_operator_stats(generation_operator_stats, result);
            stn_observe(*individual);
        }
    } else {
        allocation_run = LocalSearchAllocationRunner::run(
            population, unchanged_lower_elite, *instance, generation, ls_policy,
            frozen_upper_reference, frozen_trigger_upper_bound, budget_progress, upper_stagnation,
            lower_stagnation, population_dispersion, recent_gamma_entry_rate, evaluation_limit_distance_calls,
            local_search_engine, allocation_engine, allocation_workspaces, action_learner, ls_depth_config);
        generation_operator_stats = allocation_run.operator_stats;
        normal_ls_distance_calls = allocation_distance_calls(allocation_run);
        if (elite_unrestricted_enabled()) {
            elite_run = elite_unrestricted_controller.run(
                allocation_run, *instance, generation, normal_ls_distance_calls, frozen_trigger_upper_bound,
                local_search_engine, elite_unrestricted_workspace);
            add_operator_stats(generation_operator_stats, elite_run.result);
        }
    }
    const std::uint64_t elite_ls_distance_calls = elite_run.result.distance_calls_used;
    if (ls_policy == LocalSearchPolicy::Static) {
        normal_ls_distance_calls = instance->get_distance_calls() - generation_start_distance_calls;
    }

    // === 2. Parent pool (built before the follower so reproduction is independent of triggering) ===
    std::vector<std::shared_ptr<Individual>> ranked_upper_solutions = population;
    std::sort(ranked_upper_solutions.begin(), ranked_upper_solutions.end(),
              [](const std::shared_ptr<Individual>& lhs, const std::shared_ptr<Individual>& rhs) {
                  return lhs->upper_cost < rhs->upper_cost;
              });
    const auto target_parent_pool_size =
        static_cast<std::size_t>(std::ceil(static_cast<double>(pop_size) * parent_pool_ratio));
    const std::vector<ParentCandidate> parent_pool =
        MaReproduction::build_parent_pool(ranked_upper_solutions, target_parent_pool_size);

    const std::shared_ptr<Individual> generation_best_upper = MaReproduction::best_by_upper_cost(population);
    if (stn_logger && ls_policy != LocalSearchPolicy::Static) {
        // STN representative for the allocation policies: the best upper individual of the generation after local search.
        stn_logger->offer(*generation_best_upper, generation, instance->get_distance_calls());
    }
    if (generation_best_upper->upper_cost < global_best_upper_cost) {
        *upper_best = *generation_best_upper;
        global_best_upper_cost = upper_best->upper_cost;
        last_upper_improvement_distance_calls = instance->get_distance_calls();
    }

    // === 3. Follower on the gamma-filtered candidates ===
    const double trigger_upper_bound = global_best_upper_cost * gamma;
    std::vector<std::shared_ptr<Individual>> evaluated_complete_solutions;
    int follower_candidate_count = 0;
    int follower_runs = 0;
    const std::uint64_t follower_start_distance_calls = instance->get_distance_calls();
    const double verified_lower_cost_before_follower = verified_best->lower_cost;
    for (const auto& individual : population) {
        if (individual->upper_cost > trigger_upper_bound) continue;
        ++follower_candidate_count;
        const bool can_reuse_lower_elite = individual == unchanged_lower_elite && individual->lower_cost < INFEASIBLE;
        if (!can_reuse_lower_elite) {
            ++follower_runs;
            follower->run(individual.get());
        }
        evaluated_complete_solutions.push_back(individual);
    }
    const std::uint64_t follower_distance_calls = instance->get_distance_calls() - follower_start_distance_calls;

    std::shared_ptr<Individual> lower_elite;
    if (!evaluated_complete_solutions.empty()) {
        const std::shared_ptr<Individual> best_complete = MaReproduction::best_by_lower_cost(evaluated_complete_solutions);
        if (best_complete->lower_cost < INFEASIBLE) {
            lower_elite = best_complete;
        }
        if (verified_best->lower_cost > best_complete->lower_cost) {
            *verified_best = *best_complete;
            last_lower_improvement_distance_calls = instance->get_distance_calls();
        }
    }
    if (elite_run.triggered && elite_run.individual->lower_cost < verified_lower_cost_before_follower - 1e-12) {
        elite_run.verified_improvements = 1;
    }
    const bool retain_verified_fallback = lower_elite == nullptr && verified_best->lower_cost < INFEASIBLE;

    // === 4. Lower-archive feedback for the online learner ===
    if (ls_policy == LocalSearchPolicy::OnlineIndividual) {
        std::vector<std::shared_ptr<Individual>> ordinary_complete_solutions;
        const std::vector<std::shared_ptr<Individual>>* ordinary_source = &evaluated_complete_solutions;
        if (elite_run.triggered) {
            ordinary_complete_solutions.reserve(evaluated_complete_solutions.size());
            std::copy_if(evaluated_complete_solutions.begin(), evaluated_complete_solutions.end(),
                         std::back_inserter(ordinary_complete_solutions),
                         [&](const std::shared_ptr<Individual>& individual) {
                             return individual != elite_run.individual;
                         });
            ordinary_source = &ordinary_complete_solutions;
        }
        LocalSearchAllocationRunner::assign_lower_archive_feedback(allocation_run, *ordinary_source, action_learner);
        std::vector<std::pair<const Individual*, double>> elite_lower_archive_credits;
        if (elite_run.triggered && elite_run.individual->lower_cost < INFEASIBLE) {
            elite_lower_archive_credits = action_learner.update_lower_archive({elite_run.individual});
        }
        EliteUnrestrictedController::assign_lower_archive_feedback(elite_run, elite_lower_archive_credits);
    }

    // === 5. Reproduction ===
    const bool has_lower_elite = lower_elite != nullptr || retain_verified_fallback;
    const int offspring_target = pop_size - (has_lower_elite ? 1 : 0);
    const bool has_verified_best = verified_best->lower_cost < INFEASIBLE;
    std::vector<int> parent_use_counts;
    const std::vector<std::vector<int>> chromosomes = MaReproduction::create_offspring(
        parent_pool, verified_best.get(), has_verified_best, preprocessor->customer_ids_, offspring_target,
        tournament_size, mutation_prob, mutation_ind_prob, verified_upper_ratio, pure_immigrant_ratio,
        random_engine, uniform_real_dist, reproduction_workspace, &parent_use_counts);

    if (ls_policy != LocalSearchPolicy::Static) {
        LocalSearchAllocationRunner::assign_parent_use_feedback(allocation_run, parent_pool, parent_use_counts);
        EliteUnrestrictedController::assign_parent_use_feedback(elite_run, parent_pool, parent_use_counts);
        LocalSearchAllocationRunner::finalize_feedback(allocation_run, ls_policy, action_learner);
        const auto gamma_entries = std::count_if(allocation_run.records.begin(), allocation_run.records.end(),
                                                 [](const AllocatedLocalSearchRecord& record) {
                                                     return record.crossed_gamma;
                                                 });
        recent_gamma_entry_rate = allocation_run.records.empty()
            ? 0.0
            : static_cast<double>(gamma_entries) / static_cast<double>(allocation_run.records.size());
    }

    if (enable_logging) {
        for (std::size_t i = 0; i < LOCAL_SEARCH_OPERATOR_COUNT; ++i) {
            const auto& stats = generation_operator_stats[i];
            oss_operator_rows << std::setprecision(12)
                              << generation << "\t"
                              << MaLeader::operator_name(static_cast<LocalSearchOperator>(i)) << "\t"
                              << stats.calls << "\t"
                              << stats.accepts << "\t"
                              << calls_to_evals(stats.distance_calls) << "\t"
                              << stats.upper_gain << "\t"
                              << stats.gamma_crosses << "\n";
        }
        if (ls_policy != LocalSearchPolicy::Static) {
            for (std::size_t i = 0; i < pending_allocation_stats.size(); ++i) {
                add_allocation_stats(pending_allocation_stats[i], allocation_run.stats[i]);
            }
            if (++pending_allocation_generations == kLogInterval) write_allocation_snapshot();
        }
        if (elite_unrestricted_enabled()) {
            add_elite_stats(pending_elite_stats,
                            EliteUnrestrictedController::make_stats(elite_run, normal_ls_distance_calls,
                                                                 elite_unrestricted_controller.credit_distance_calls()));
            if (++pending_elite_generations == kLogInterval) write_elite_snapshot();
        }
        flush_local_search_log();
    }

    // === 6. Next population: elite slot (x_g^E, or x* as fallback) + offspring (Individual objects are reused) ===
    evaluated_complete_solutions.clear();
    ranked_upper_solutions.clear();
    allocation_run.records.clear();
    elite_run.individual.reset();
    unchanged_lower_elite.reset();
    retained_lower_elite.reset();

    population_buffer.clear();
    std::size_t lower_elite_index = population.size();
    if (lower_elite != nullptr) {
        const auto position = std::find(population.begin(), population.end(), lower_elite);
        if (position == population.end()) {
            throw std::logic_error("lower elite is not part of the current population");
        }
        lower_elite_index = static_cast<std::size_t>(std::distance(population.begin(), position));
        population_buffer.push_back(lower_elite);
    }

    std::size_t reusable_index = 0;
    auto take_reusable_individual = [&]() -> std::shared_ptr<Individual> {
        while (reusable_index < population.size() && reusable_index == lower_elite_index) {
            ++reusable_index;
        }
        if (reusable_index >= population.size()) {
            throw std::logic_error("population reuse pool was exhausted");
        }
        return population[reusable_index++];
    };

    if (retain_verified_fallback) {
        lower_elite = take_reusable_individual();
        *lower_elite = *verified_best;
        population_buffer.push_back(lower_elite);
    }

    for (const auto& chromosome : chromosomes) {
        std::shared_ptr<Individual> offspring = take_reusable_individual();
        load_offspring(*offspring, chromosome);
        population_buffer.push_back(std::move(offspring));
    }

    if (population_buffer.size() != static_cast<std::size_t>(pop_size)) {
        throw std::logic_error("rebuilt population has an unexpected size");
    }
    retained_lower_elite = lower_elite;
    population.clear();
    population.swap(population_buffer);

    if (enable_logging) {
        const std::uint64_t generation_distance_calls = instance->get_distance_calls() - generation_start_distance_calls;
        const std::uint64_t categorized = normal_ls_distance_calls + elite_ls_distance_calls + follower_distance_calls;
        SearchBudgetStats budget_stats;
        budget_stats.normal_local_search_distance_calls = normal_ls_distance_calls;
        budget_stats.elite_local_search_distance_calls = elite_ls_distance_calls;
        budget_stats.follower_distance_calls = follower_distance_calls;
        budget_stats.other_distance_calls = generation_distance_calls >= categorized ? generation_distance_calls - categorized : 0;
        budget_stats.total_distance_calls = generation_distance_calls;
        budget_stats.follower_candidates = follower_candidate_count;
        budget_stats.follower_runs = follower_runs;
        add_budget_stats(pending_budget_stats, budget_stats);
        if (++pending_budget_generations == kLogInterval) write_budget_snapshot();
        flush_local_search_log();
    }
}

void Ma::load_offspring(Individual& offspring, const std::vector<int>& chromosome) {
    initializer->prins_split(chromosome, split_routes);  // routes include the depot at both ends
    check_capacity(split_routes);
    offspring.load_routes(split_routes, instance->compute_total_distance(split_routes),
                          instance->compute_demand_sum_per_route(split_routes));
}

void Ma::run_final_follower() {
    // Fall back to the best upper-level solution if no feasible complete solution (x*) was found.
    if (verified_best->lower_cost >= INFEASIBLE && upper_best != nullptr) {
        *verified_best = *upper_best;
    }
    if (preprocessor->params.enable_final_refinement) {
        follower->refine(verified_best.get());
    } else {
        follower->run_with_variables(verified_best.get());
    }
}

// ============================== Logging ==============================

fs::path Ma::log_directory() const {
    return fs::path(preprocessor->params.k_stats_path) / this->name / instance->instance_name_ / std::to_string(seed);
}

void Ma::open_log_for_evolution() {
    const fs::path directory = log_directory();
    create_directories_if_not_exists(directory);
    log_evolution.open(directory / ("evols." + instance->instance_name_ + ".csv"));
    log_evolution << "iter,evals,best_upper_cost,best_lower_cost,progress,duration\n";
}

void Ma::flush_row_into_evol_log() {
    const double evals_used = instance->get_evals();
    oss_row_evol << generation << ","
                 << evals_used << ","
                 << global_best_upper_cost << ","
                 << verified_best->lower_cost << ","
                 << evals_used / preprocessor->max_evals_ << ","
                 << duration.count() << "\n";
}

void Ma::close_log_for_evolution() {
    log_evolution << oss_row_evol.str();
    oss_row_evol.str("");
    oss_row_evol.clear();
    log_evolution.close();
}

void Ma::save_log_for_solution() {
    const fs::path directory = log_directory();
    create_directories_if_not_exists(directory);
    log_solution.open(directory / ("solution." + instance->instance_name_ + ".txt"));
    log_solution << std::fixed << std::setprecision(5) << verified_best->lower_cost << std::endl;
    for (int i = 0; i < follower->num_routes; ++i) {
        for (int j = 0; j < follower->lower_num_nodes_per_route[i]; ++j) {
            log_solution << follower->lower_routes[i][j] << ",";
        }
        log_solution << std::endl;
    }
    log_solution.close();
}

void Ma::open_log_for_local_search() {
    const fs::path directory = log_directory();
    create_directories_if_not_exists(directory);

    log_operators.open(directory / "local-search-operators.tsv");
    log_operators << "iter\toperator\tcalls\taccepts\tevals\tupper_gain\tgamma_crosses\n";
    if (ls_policy != LocalSearchPolicy::Static) {
        log_allocation.open(directory / "local-search-allocation.tsv");
        log_allocation << "iter\tpolicy\taction\tselections\tforced_local_optima\t"
                          "exploratory_selections\treached_local_optimum\t"
                          "hit_move_limit\thit_distance_limit\taccepted_moves\t"
                          "neighborhood_calls\tevals\tcontinuation_evals\tupper_gain\t"
                          "gamma_crosses\tparent_uses\t"
                          "lower_archive_entries\tparent_reward\tlower_reward\t"
                          "gamma_reward\tcontinuation_gain_signal\treward\t"
                          "avg_incremental_cost_units\tavg_score\n";
    }
    if (elite_unrestricted_enabled()) {
        log_elite.open(directory / "elite-local-search.tsv");
        log_elite << "iter\teligible_candidates\ttriggers\tnormal_ls_evals\t"
                     "elite_evals\taccepted_moves\tneighborhood_calls\tupper_gain\t"
                     "gamma_crosses\tparent_uses\tlower_archive_entries\t"
                     "verified_improvements\tbudget_credit_evals\n";
    }
    log_budget.open(directory / "search-budget.tsv");
    log_budget << "iter\tnormal_ls_evals\telite_ls_evals\tfollower_evals\t"
                  "other_evals\ttotal_evals\tnormal_ls_share\telite_ls_share\t"
                  "follower_share\tother_share\tfollower_candidates\tfollower_runs\n";

    std::ofstream log_configuration(directory / "run-configuration.tsv");
    log_configuration << "seed\tgamma\tls_policy\tstatic_action\tstatic_unrestricted\tunrestricted_credit_rho\tls_depth\t"
                         "weak_move_fraction\tmedium_move_fraction\tstrong_move_fraction\t"
                         "strong_weak_call_multiplier\taction_cost_weight\n";
    log_configuration << std::setprecision(12)
                      << seed << "\t"
                      << gamma << "\t"
                      << local_search_policy_name(ls_policy) << "\t"
                      << local_search_action_name(static_action) << "\t"
                      << static_unrestricted << "\t"
                      << unrestricted_credit_rho << "\t"
                      << local_search_depth_profile_name(ls_depth_profile) << "\t"
                      << ls_depth_config.weak_move_fraction << "\t"
                      << ls_depth_config.medium_move_fraction << "\t"
                      << ls_depth_config.strong_move_fraction << "\t"
                      << ls_depth_config.strong_weak_call_multiplier << "\t"
                      << action_cost_weight << "\n";
}

void Ma::flush_local_search_log() {
    const auto flush = [](std::ofstream& file, std::ostringstream& rows) {
        if (file.is_open()) {
            file << rows.str();
        }
        rows.str("");
        rows.clear();
    };
    flush(log_operators, oss_operator_rows);
    flush(log_allocation, oss_allocation_rows);
    flush(log_elite, oss_elite_rows);
    flush(log_budget, oss_budget_rows);
}

void Ma::close_log_for_local_search() {
    write_allocation_snapshot();
    write_elite_snapshot();
    write_budget_snapshot();
    flush_local_search_log();
    log_operators.close();
    log_allocation.close();
    log_elite.close();
    log_budget.close();
}

void Ma::write_allocation_snapshot() {
    if (pending_allocation_generations == 0 || ls_policy == LocalSearchPolicy::Static) {
        return;
    }
    const std::array<LocalSearchAction, LOCAL_SEARCH_ACTION_COUNT> actions = {
        LocalSearchAction::Weak, LocalSearchAction::Medium, LocalSearchAction::Strong};
    for (std::size_t i = 0; i < actions.size(); ++i) {
        const auto& stats = pending_allocation_stats[i];
        const double selections = static_cast<double>(stats.selections);
        oss_allocation_rows << std::setprecision(12)
                            << generation << "\t"
                            << local_search_policy_name(ls_policy) << "\t"
                            << local_search_action_name(actions[i]) << "\t"
                            << stats.selections << "\t"
                            << stats.forced_local_optima << "\t"
                            << stats.exploratory_selections << "\t"
                            << stats.local_optimum_terminations << "\t"
                            << stats.move_limit_terminations << "\t"
                            << stats.distance_limit_terminations << "\t"
                            << stats.accepted_moves << "\t"
                            << stats.neighborhood_calls << "\t"
                            << calls_to_evals(stats.distance_calls) << "\t"
                            << calls_to_evals(stats.continuation_distance_calls) << "\t"
                            << stats.upper_gain << "\t"
                            << stats.gamma_crosses << "\t"
                            << stats.parent_uses << "\t"
                            << stats.lower_archive_entries << "\t"
                            << stats.parent_reward << "\t"
                            << stats.lower_reward << "\t"
                            << stats.gamma_reward << "\t"
                            << stats.continuation_gain_signal << "\t"
                            << stats.reward << "\t"
                            << (stats.selections > 0 ? stats.incremental_cost_units / selections : 0.0) << "\t"
                            << (stats.selections > 0 ? stats.selection_score / selections : 0.0) << "\n";
    }
    pending_allocation_stats = {};
    pending_allocation_generations = 0;
}

void Ma::write_elite_snapshot() {
    if (pending_elite_generations == 0 || !elite_unrestricted_enabled()) {
        return;
    }
    const auto& stats = pending_elite_stats;
    const double credit_evals = instance->problem_size_ > 0
        ? static_cast<double>(stats.ending_credit_distance_calls) / static_cast<double>(instance->problem_size_)
        : 0.0;
    oss_elite_rows << std::setprecision(12)
                   << generation << "\t"
                   << stats.eligible_candidates << "\t"
                   << stats.triggers << "\t"
                   << calls_to_evals(stats.normal_distance_calls) << "\t"
                   << calls_to_evals(stats.distance_calls) << "\t"
                   << stats.accepted_moves << "\t"
                   << stats.neighborhood_calls << "\t"
                   << stats.upper_gain << "\t"
                   << stats.gamma_crosses << "\t"
                   << stats.parent_uses << "\t"
                   << stats.lower_archive_entries << "\t"
                   << stats.verified_improvements << "\t"
                   << credit_evals << "\n";
    pending_elite_stats = {};
    pending_elite_generations = 0;
}

void Ma::write_budget_snapshot() {
    if (pending_budget_generations == 0) {
        return;
    }
    const auto& stats = pending_budget_stats;
    const double total = static_cast<double>(stats.total_distance_calls);
    const auto share = [&](const std::uint64_t calls) {
        return total > 0.0 ? static_cast<double>(calls) / total : 0.0;
    };
    oss_budget_rows << std::setprecision(12)
                    << generation << "\t"
                    << calls_to_evals(stats.normal_local_search_distance_calls) << "\t"
                    << calls_to_evals(stats.elite_local_search_distance_calls) << "\t"
                    << calls_to_evals(stats.follower_distance_calls) << "\t"
                    << calls_to_evals(stats.other_distance_calls) << "\t"
                    << calls_to_evals(stats.total_distance_calls) << "\t"
                    << share(stats.normal_local_search_distance_calls) << "\t"
                    << share(stats.elite_local_search_distance_calls) << "\t"
                    << share(stats.follower_distance_calls) << "\t"
                    << share(stats.other_distance_calls) << "\t"
                    << stats.follower_candidates << "\t"
                    << stats.follower_runs << "\n";
    pending_budget_stats = {};
    pending_budget_generations = 0;
}
