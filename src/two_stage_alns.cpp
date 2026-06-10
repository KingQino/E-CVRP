//
// Created by Yinghao Qin on 26/04/2026.
//

#include "two_stage_alns.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>

namespace fs = std::filesystem;

TwoStageAlns::TwoStageAlns(const int seed, Case* instance, Preprocessor* preprocessor)
    : HeuristicInterface("TWOSTAGEALNS", seed, instance, preprocessor) {
    enable_logging = preprocessor->params.enable_logging;
    stop_criteria = preprocessor->params.stop_criteria;

    start_deviation = preprocessor->params.alns_start_deviation;
    adaptive_decay = preprocessor->params.alns_decay;
    score_accepted = preprocessor->params.alns_score_accepted;
    score_improved = preprocessor->params.alns_score_improved;
    score_global_best = preprocessor->params.alns_score_global_best;
    min_destroy_ratio = preprocessor->params.alns_min_destroy_ratio;
    max_destroy_ratio = preprocessor->params.alns_max_destroy_ratio;
    min_destroy_abs = preprocessor->params.alns_min_destroy_abs;
    max_destroy_abs = preprocessor->params.alns_max_destroy_abs;
    geo_destroy_rand_factor = preprocessor->params.alns_geo_destroy_rand_factor;
    repair_rand_factor = preprocessor->params.alns_repair_rand_factor;

    iter_global = 0L;
    alns_iter = 0L;
    alns_idle_iter = 0L;

    current = std::make_unique<Individual>(instance, preprocessor);
    alns_candidate = std::make_unique<Individual>(instance, preprocessor);
    global_best_upper = std::make_unique<Individual>(instance, preprocessor);
    global_best_lower = std::make_unique<Individual>(instance, preprocessor);
    global_best_upper->upper_cost = std::numeric_limits<double>::max();
    global_best_lower->upper_cost = std::numeric_limits<double>::max();
    global_best_lower->lower_cost = std::numeric_limits<double>::max();

    initializer = new Initializer(random_engine, instance, preprocessor);
    follower = new Follower(instance, preprocessor);

    best_arc_cost = std::vector<std::vector<double>>(instance->problem_size_,
        std::vector<double>(instance->problem_size_, std::numeric_limits<double>::max() / 1024.0));

    stage_best_upper_cost = std::numeric_limits<double>::max();
    last_destroy_idx = -1;
    last_repair_idx = -1;
    last_candidate_built = false;
    last_candidate_accepted = false;
    last_candidate_improved = false;
    last_candidate_global_best = false;
    last_candidate_upper = std::numeric_limits<double>::quiet_NaN();

    prof_follower_run_sec = 0.0;
    prof_follower_refine_sec = 0.0;
    prof_follower_run_calls = 0L;

    scratch_buffers.customer_pool.reserve(instance->num_customer_);
    scratch_buffers.selected_customers.reserve(instance->num_customer_);
    scratch_buffers.rebuilt_demands.reserve(preprocessor->route_cap_);
    scratch_buffers.rebuilt_routes.reserve(preprocessor->route_cap_);
    scratch_buffers.node_marks.reserve(instance->problem_size_);
    scratch_buffers.history_candidates.reserve(instance->num_customer_);

    reset_adaptive_state();
}

TwoStageAlns::~TwoStageAlns() {
    delete initializer;
    delete follower;
}

double TwoStageAlns::run() {
    start = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration<double>::zero();
    iter_global = 0L;
    alns_iter = 0L;
    alns_idle_iter = 0L;
    prof_follower_run_sec = 0.0;
    prof_follower_refine_sec = 0.0;
    prof_follower_run_calls = 0L;

    reset_adaptive_state();

    if (enable_logging) {
        open_log_for_evolution();
    }

    initialize_heuristic();
    run_heuristic();
    run_final_follower();

    if (enable_logging) {
        close_log_for_evolution();
        save_log_for_solution();
        save_log_for_profile();
    }

    return global_best_lower->lower_cost;
}

void TwoStageAlns::initialize_heuristic() {
    construct_initial_seed(current.get());
    *global_best_upper = *current;
    stage_best_upper_cost = current->upper_cost;
}

void TwoStageAlns::run_heuristic() {
    *current = *global_best_upper;
    stage_best_upper_cost = current->upper_cost;
    update_arc_history(*current);

    WorkingSolution work;

    while (within_global_budget()) {
        export_working_solution(current.get(), work);

        last_destroy_idx = roulette_wheel_selection(destroy_weights.data(), kNumDestroyMethods);
        last_repair_idx = roulette_wheel_selection(repair_weights.data(), kNumRepairMethods);
        ++destroy_calls[last_destroy_idx];
        ++repair_calls[last_repair_idx];

        if (enable_logging) {
            const auto t0 = std::chrono::high_resolution_clock::now();
            apply_destroy(work, static_cast<DestroyMethod>(last_destroy_idx));
            destroy_time_sec[last_destroy_idx] += std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();
        } else {
            apply_destroy(work, static_cast<DestroyMethod>(last_destroy_idx));
        }

        bool repaired = false;
        if (!work.removed_customers.empty()) {
            if (enable_logging) {
                const auto t0 = std::chrono::high_resolution_clock::now();
                repaired = apply_repair(work, static_cast<RepairMethod>(last_repair_idx));
                repair_time_sec[last_repair_idx] += std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();
            } else {
                repaired = apply_repair(work, static_cast<RepairMethod>(last_repair_idx));
            }
        }

        last_candidate_built = false;
        last_candidate_accepted = false;
        last_candidate_improved = false;
        last_candidate_global_best = false;
        last_candidate_upper = std::numeric_limits<double>::quiet_NaN();

        bool accepted = false;
        bool improved = false;
        bool new_best_upper = false;

        if (repaired || work.removed_customers.empty()) {
            ++destroy_successes[last_destroy_idx];
            ++repair_successes[last_repair_idx];

            load_working_solution(alns_candidate.get(), work);
            last_candidate_built = true;
            last_candidate_upper = alns_candidate->upper_cost;

            const double prev_stage_best = stage_best_upper_cost;
            const double prev_global_best = global_best_upper->upper_cost;
            accepted = should_accept(alns_candidate->upper_cost);

            if (accepted) {
                improved = alns_candidate->upper_cost < current->upper_cost - EPSILON;
                *current = *alns_candidate;
                update_arc_history(*current);

                if (current->upper_cost < stage_best_upper_cost - EPSILON) {
                    stage_best_upper_cost = current->upper_cost;
                    new_best_upper = true;
                    alns_idle_iter = 0L;
                } else {
                    ++alns_idle_iter;
                }
            } else {
                ++alns_idle_iter;
            }

            update_global_best_upper(*alns_candidate);
            if (global_best_upper->upper_cost < prev_global_best - EPSILON) {
                new_best_upper = true;
            }

            last_candidate_accepted = accepted;
            last_candidate_improved = improved;
            last_candidate_global_best = new_best_upper;

            if (!accepted && alns_candidate->upper_cost < prev_stage_best - EPSILON) {
                stage_best_upper_cost = alns_candidate->upper_cost;
            }
        } else {
            ++alns_idle_iter;
        }

        const double score = calc_score(accepted, improved, new_best_upper);
        destroy_weights[last_destroy_idx] = destroy_weights[last_destroy_idx] * adaptive_decay +
                                            score * (1.0 - adaptive_decay);
        repair_weights[last_repair_idx] = repair_weights[last_repair_idx] * adaptive_decay +
                                          score * (1.0 - adaptive_decay);

        ++alns_iter;
        ++iter_global;

        if (enable_logging) {
            flush_row_into_evol_log();
        }
    }
}

void TwoStageAlns::reset_adaptive_state() {
    destroy_weights.fill(1.0);
    repair_weights.fill(1.0);
    destroy_calls.fill(0L);
    repair_calls.fill(0L);
    destroy_successes.fill(0L);
    repair_successes.fill(0L);
    destroy_time_sec.fill(0.0);
    repair_time_sec.fill(0.0);

    for (auto& row : best_arc_cost) {
        std::fill(row.begin(), row.end(), std::numeric_limits<double>::max() / 1024.0);
    }

    last_destroy_idx = -1;
    last_repair_idx = -1;
    last_candidate_built = false;
    last_candidate_accepted = false;
    last_candidate_improved = false;
    last_candidate_global_best = false;
    last_candidate_upper = std::numeric_limits<double>::quiet_NaN();
}

void TwoStageAlns::refresh_duration() {
    duration = std::chrono::high_resolution_clock::now() - start;
}

bool TwoStageAlns::within_global_budget() {
    if (stop_criteria == 1 || enable_logging) {
        refresh_duration();
    }
    return stop_criteria == 0 ? !stop_criteria_max_evals() : !stop_criteria_max_exec_time(duration);
}

void TwoStageAlns::construct_initial_seed(Individual* seed_solution) const {
    // Match PALNS-CVRP's intent: randomized seed customers + greedy sequential best insertion.
    const std::vector<std::vector<int>> routes = initializer->routes_constructor_with_greedy_seq(true);
    seed_solution->load_routes(routes,
                               instance->compute_total_distance(routes),
                               instance->compute_demand_sum_per_route(routes));
}

void TwoStageAlns::update_global_best_upper(const Individual& ind) const {
    if (ind.upper_cost < global_best_upper->upper_cost - EPSILON) {
        *global_best_upper = ind;
    }
}

void TwoStageAlns::run_final_follower() {
    *global_best_lower = *global_best_upper;

    if (preprocessor->params.enable_final_refinement) {
        if (enable_logging) {
            const auto t0 = std::chrono::high_resolution_clock::now();
            follower->refine(global_best_lower.get());
            prof_follower_refine_sec += std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();
        } else {
            follower->refine(global_best_lower.get());
        }
    } else {
        if (enable_logging) {
            const auto t0 = std::chrono::high_resolution_clock::now();
            follower->run_with_variables(global_best_lower.get());
            prof_follower_run_sec += std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();
            ++prof_follower_run_calls;
        } else {
            follower->run_with_variables(global_best_lower.get());
        }
    }
}

void TwoStageAlns::export_working_solution(const Individual* ind, WorkingSolution& work) {
    work.routes.clear();
    work.route_demands.clear();
    work.removed_customers.clear();
    work.total_cost = ind->upper_cost;

    work.routes.reserve(ind->num_routes);
    work.route_demands.reserve(ind->num_routes);

    for (int i = 0; i < ind->num_routes; ++i) {
        work.routes.emplace_back(ind->routes[i], ind->routes[i] + ind->num_nodes_per_route[i]);
        work.route_demands.push_back(ind->demand_sum_per_route[i]);
    }
}

void TwoStageAlns::load_working_solution(Individual* ind, const WorkingSolution& work) {
    ind->clean();
    ind->load_routes(work.routes, work.total_cost, work.route_demands);
}

int TwoStageAlns::calc_destroy_count() {
    int min_remove = std::max(min_destroy_abs,
        static_cast<int>(min_destroy_ratio * static_cast<double>(instance->num_customer_)));
    int max_remove = std::min(max_destroy_abs,
        static_cast<int>(max_destroy_ratio * static_cast<double>(instance->num_customer_)));

    min_remove = std::min(min_remove, instance->num_customer_);
    max_remove = std::min(max_remove, instance->num_customer_);
    if (min_remove > max_remove) min_remove = max_remove;
    if (max_remove <= 0) return 0;

    std::uniform_int_distribution<int> destroy_dist(min_remove, max_remove);
    return destroy_dist(random_engine);
}

void TwoStageAlns::remove_customers_from_routes(WorkingSolution& work, const std::vector<int>& selected_customers) {
    if (selected_customers.empty()) return;

    auto& is_removed = scratch_buffers.node_marks;
    if (static_cast<int>(is_removed.size()) != instance->problem_size_) {
        is_removed.assign(instance->problem_size_, 0);
    } else {
        std::fill(is_removed.begin(), is_removed.end(), 0);
    }

    work.removed_customers.clear();
    work.removed_customers.reserve(selected_customers.size());
    for (const int customer : selected_customers) {
        if (!is_removed[customer]) {
            is_removed[customer] = 1;
            work.removed_customers.push_back(customer);
        }
    }

    auto& new_routes = scratch_buffers.rebuilt_routes;
    auto& new_demands = scratch_buffers.rebuilt_demands;
    new_routes.clear();
    new_demands.clear();
    double new_total_cost = 0.0;
    new_routes.reserve(work.routes.size());
    new_demands.reserve(work.route_demands.size());

    for (const auto& route : work.routes) {
        std::vector<int> new_route;
        new_route.reserve(route.size());
        new_route.push_back(instance->depot_);

        int demand_sum = 0;
        for (int pos = 1; pos < static_cast<int>(route.size()) - 1; ++pos) {
            const int node = route[pos];
            if (is_removed[node]) continue;
            new_route.push_back(node);
            demand_sum += instance->get_customer_demand_(node);
        }

        new_route.push_back(instance->depot_);
        if (new_route.size() > 2) {
            new_total_cost += instance->compute_total_distance(new_route);
            new_routes.push_back(std::move(new_route));
            new_demands.push_back(demand_sum);
        }
    }

    work.routes.swap(new_routes);
    work.route_demands.swap(new_demands);
    work.total_cost = new_total_cost;
}

void TwoStageAlns::destroy_random(WorkingSolution& work) {
    auto& assigned_customers = scratch_buffers.customer_pool;
    auto& selected = scratch_buffers.selected_customers;
    assigned_customers.clear();
    selected.clear();

    for (const auto& route : work.routes) {
        for (int pos = 1; pos < static_cast<int>(route.size()) - 1; ++pos) {
            assigned_customers.push_back(route[pos]);
        }
    }

    const int n_remove = std::min(calc_destroy_count(), static_cast<int>(assigned_customers.size()));
    selected.reserve(n_remove);

    for (int removed = 0; removed < n_remove && !assigned_customers.empty(); ++removed) {
        std::uniform_int_distribution<int> dist(0, static_cast<int>(assigned_customers.size()) - 1);
        const int idx = dist(random_engine);
        selected.push_back(assigned_customers[idx]);
        assigned_customers[idx] = assigned_customers.back();
        assigned_customers.pop_back();
    }

    remove_customers_from_routes(work, selected);
}

void TwoStageAlns::destroy_geo(WorkingSolution& work) {
    auto& remaining_customers = scratch_buffers.customer_pool;
    auto& selected = scratch_buffers.selected_customers;
    auto& is_customer_remaining = scratch_buffers.node_marks;
    remaining_customers.clear();
    selected.clear();

    for (const auto& route : work.routes) {
        for (int pos = 1; pos < static_cast<int>(route.size()) - 1; ++pos) {
            remaining_customers.push_back(route[pos]);
        }
    }

    const int n_remove = std::min(calc_destroy_count(), static_cast<int>(remaining_customers.size()));
    if (n_remove <= 0) {
        work.removed_customers.clear();
        return;
    }

    selected.reserve(n_remove);
    if (static_cast<int>(is_customer_remaining.size()) != instance->problem_size_) {
        is_customer_remaining.assign(instance->problem_size_, 0);
    } else {
        std::fill(is_customer_remaining.begin(), is_customer_remaining.end(), 0);
    }
    for (const int customer : remaining_customers) {
        is_customer_remaining[customer] = 1;
    }

    int remaining_count = static_cast<int>(remaining_customers.size());
    {
        std::uniform_int_distribution<int> dist(0, static_cast<int>(remaining_customers.size()) - 1);
        const int idx = dist(random_engine);
        const int seed_customer = remaining_customers[idx];
        selected.push_back(seed_customer);
        is_customer_remaining[seed_customer] = 0;
        --remaining_count;
    }

    while (static_cast<int>(selected.size()) < n_remove && remaining_count > 0) {
        std::uniform_int_distribution<int> removed_dist(0, static_cast<int>(selected.size()) - 1);
        const int anchor = selected[removed_dist(random_engine)];

        const int offset = static_cast<int>(pow(uniform_real_dist(random_engine), geo_destroy_rand_factor) *
            static_cast<double>(remaining_count - 1));

        int chosen_customer = -1;
        int valid_rank = 0;
        const auto& nearby_customers = preprocessor->sorted_nearby_customers_[anchor];
        for (const int customer : nearby_customers) {
            if (!is_customer_remaining[customer]) continue;
            if (valid_rank == offset) {
                chosen_customer = customer;
                break;
            }
            ++valid_rank;
        }

        if (chosen_customer < 0) {
            for (const int customer : remaining_customers) {
                if (is_customer_remaining[customer]) {
                    chosen_customer = customer;
                    break;
                }
            }
            if (chosen_customer < 0) break;
        }

        selected.push_back(chosen_customer);
        is_customer_remaining[chosen_customer] = 0;
        --remaining_count;
    }

    remove_customers_from_routes(work, selected);
}

void TwoStageAlns::destroy_history(WorkingSolution& work) {
    auto& candidates = scratch_buffers.history_candidates;
    auto& selected = scratch_buffers.selected_customers;
    candidates.clear();
    selected.clear();

    for (const auto& route : work.routes) {
        for (int pos = 1; pos < static_cast<int>(route.size()) - 1; ++pos) {
            const int prev = route[pos - 1];
            const int node = route[pos];
            const int next = route[pos + 1];
            const double score = 0.5 * (best_arc_cost[prev][node] + best_arc_cost[node][next]);
            candidates.push_back({node, score});
        }
    }

    std::sort(candidates.begin(), candidates.end(),
        [](const HistoryDestroyCandidate& lhs, const HistoryDestroyCandidate& rhs) { return lhs.score > rhs.score; });

    const int n_remove = std::min(calc_destroy_count(), static_cast<int>(candidates.size()));
    selected.reserve(n_remove);

    while (static_cast<int>(selected.size()) < n_remove && !candidates.empty()) {
        const int offset = static_cast<int>(pow(uniform_real_dist(random_engine), kHistoryDestroyRandFactor) *
            static_cast<double>(candidates.size() - 1));
        selected.push_back(candidates[offset].customer);
        candidates.erase(candidates.begin() + offset);
    }

    remove_customers_from_routes(work, selected);
}

void TwoStageAlns::apply_destroy(WorkingSolution& work, const DestroyMethod method) {
    switch (method) {
        case DestroyMethod::Random:
            destroy_random(work);
            break;
        case DestroyMethod::Geo:
            destroy_geo(work);
            break;
        case DestroyMethod::History:
            destroy_history(work);
            break;
    }
}

bool TwoStageAlns::find_best_insertion(const std::vector<int>& route, const int customer, int& best_pos, double& best_delta) const {
    if (route.size() < 2) return false;

    best_delta = std::numeric_limits<double>::max();
    best_pos = -1;

    for (int pos = 1; pos < static_cast<int>(route.size()); ++pos) {
        const int prev = route[pos - 1];
        const int next = route[pos];
        const double delta = instance->get_distance(prev, customer) +
                             instance->get_distance(customer, next) -
                             instance->get_distance(prev, next);
        if (delta < best_delta) {
            best_delta = delta;
            best_pos = pos;
        }
    }

    return best_pos != -1;
}

double TwoStageAlns::route_cost_for_new_customer(const int customer) const {
    return instance->get_distance(instance->depot_, customer) +
           instance->get_distance(customer, instance->depot_);
}

void TwoStageAlns::insert_customer(WorkingSolution& work, int route_idx, const int customer, const int insert_pos,
                                   const double route_delta) const {
    if (route_idx == static_cast<int>(work.routes.size())) {
        work.routes.push_back({instance->depot_, instance->depot_});
        work.route_demands.push_back(0);
    }

    work.routes[route_idx].insert(work.routes[route_idx].begin() + insert_pos, customer);
    work.route_demands[route_idx] += instance->get_customer_demand_(customer);
    work.total_cost += route_delta;
}

bool TwoStageAlns::repair_regret2(WorkingSolution& work, const double randomized_factor) {
    while (!work.removed_customers.empty()) {
        bool has_single_option = false;
        double best_single_metric = std::numeric_limits<double>::max();
        double best_regret_metric = -std::numeric_limits<double>::max();

        int chosen_removed_idx = -1;
        int chosen_route_idx = -1;
        int chosen_insert_pos = -1;
        double chosen_route_delta = 0.0;

        for (int idx = 0; idx < static_cast<int>(work.removed_customers.size()); ++idx) {
            const int customer = work.removed_customers[idx];
            const int demand = instance->get_customer_demand_(customer);

            double best_delta = std::numeric_limits<double>::max();
            double second_best_delta = std::numeric_limits<double>::max();
            int best_route_idx = -1;
            int best_insert_pos = -1;

            for (int route_idx = 0; route_idx < static_cast<int>(work.routes.size()); ++route_idx) {
                if (work.route_demands[route_idx] + demand > instance->max_vehicle_capa_) continue;

                int insert_pos = -1;
                double route_delta = std::numeric_limits<double>::max();
                if (!find_best_insertion(work.routes[route_idx], customer, insert_pos, route_delta)) continue;

                if (route_delta < best_delta) {
                    second_best_delta = best_delta;
                    best_delta = route_delta;
                    best_route_idx = route_idx;
                    best_insert_pos = insert_pos;
                } else if (route_delta < second_best_delta) {
                    second_best_delta = route_delta;
                }
            }

            if (static_cast<int>(work.routes.size()) < preprocessor->route_cap_) {
                const double new_route_delta = route_cost_for_new_customer(customer);
                if (new_route_delta < best_delta) {
                    second_best_delta = best_delta;
                    best_delta = new_route_delta;
                    best_route_idx = static_cast<int>(work.routes.size());
                    best_insert_pos = 1;
                } else if (new_route_delta < second_best_delta) {
                    second_best_delta = new_route_delta;
                }
            }

            if (best_route_idx == -1) continue;

            const double factor = randomized_factor > 1.0
                ? 1.0 + uniform_real_dist(random_engine) * (randomized_factor - 1.0)
                : 1.0;

            if (second_best_delta == std::numeric_limits<double>::max()) {
                const double metric = best_delta * factor;
                if (!has_single_option || metric < best_single_metric) {
                    has_single_option = true;
                    best_single_metric = metric;
                    chosen_removed_idx = idx;
                    chosen_route_idx = best_route_idx;
                    chosen_insert_pos = best_insert_pos;
                    chosen_route_delta = best_delta;
                }
            } else if (!has_single_option) {
                const double metric = (second_best_delta - best_delta) * factor;
                if (metric > best_regret_metric) {
                    best_regret_metric = metric;
                    chosen_removed_idx = idx;
                    chosen_route_idx = best_route_idx;
                    chosen_insert_pos = best_insert_pos;
                    chosen_route_delta = best_delta;
                }
            }
        }

        if (chosen_removed_idx == -1) {
            return false;
        }

        const int customer = work.removed_customers[chosen_removed_idx];
        insert_customer(work, chosen_route_idx, customer, chosen_insert_pos, chosen_route_delta);
        work.removed_customers[chosen_removed_idx] = work.removed_customers.back();
        work.removed_customers.pop_back();
    }

    return true;
}

bool TwoStageAlns::apply_repair(WorkingSolution& work, const RepairMethod method) {
    switch (method) {
        case RepairMethod::Regret2:
            return repair_regret2(work, 1.0);
        case RepairMethod::Regret2Randomized:
            return repair_regret2(work, repair_rand_factor);
    }
    return false;
}

int TwoStageAlns::roulette_wheel_selection(const double* weights, const int count) {
    if (count <= 1) return 0;

    const double sum_weights = std::accumulate(weights, weights + count, 0.0);
    if (sum_weights <= EPSILON) {
        std::uniform_int_distribution<int> fallback_dist(0, count - 1);
        return fallback_dist(random_engine);
    }

    const double target = uniform_real_dist(random_engine) * sum_weights;
    double running_sum = 0.0;
    for (int idx = 0; idx < count - 1; ++idx) {
        running_sum += weights[idx];
        if (target <= running_sum) return idx;
    }

    return count - 1;
}

double TwoStageAlns::calc_score(const bool accepted, const bool improved, const bool new_best_upper) const {
    double score = 1.0;
    if (accepted) score = std::max(score, score_accepted);
    if (improved) score = std::max(score, score_improved);
    if (new_best_upper) score = std::max(score, score_global_best);
    return score;
}

double TwoStageAlns::acceptance_deviation() const {
    double progress = 0.0;

    if (stop_criteria == 0) {
        progress = instance->get_evals() / preprocessor->max_evals_;
    } else if (preprocessor->max_exec_time_ > 0) {
        progress = duration.count() / static_cast<double>(preprocessor->max_exec_time_);
    }

    progress = std::clamp(progress, 0.0, 1.0);
    return std::max(0.0, start_deviation * (1.0 - progress));
}

bool TwoStageAlns::should_accept(const double candidate_upper) const {
    if (candidate_upper < stage_best_upper_cost - EPSILON) return true;

    const double deviation = acceptance_deviation();
    const double denom = std::max(candidate_upper, EPSILON);
    const double gap = (candidate_upper - stage_best_upper_cost) / denom;
    return gap < deviation - EPSILON;
}

void TwoStageAlns::update_arc_history(const Individual& ind) {
    for (int route_idx = 0; route_idx < ind.num_routes; ++route_idx) {
        const int length = ind.num_nodes_per_route[route_idx];
        for (int pos = 0; pos < length - 1; ++pos) {
            const int from = ind.routes[route_idx][pos];
            const int to = ind.routes[route_idx][pos + 1];
            best_arc_cost[from][to] = std::min(best_arc_cost[from][to], ind.upper_cost);
        }
    }
}

void TwoStageAlns::open_log_for_evolution() {
    const fs::path directory = fs::path(preprocessor->params.k_stats_path)
        / this->name
        / instance->instance_name_
        / std::to_string(seed);

    create_directories_if_not_exists(directory);

    const std::string file_name = "evols." + instance->instance_name_ + ".csv";
    log_evolution.open(directory / file_name);
    log_evolution << "iter_global,global_best_upper,candidate_upper,deviation,destroy,repair,evals,time\n";
}

void TwoStageAlns::close_log_for_evolution() {
    log_evolution << oss_row_evol.str();
    oss_row_evol.clear();
    log_evolution.close();
}

void TwoStageAlns::flush_row_into_evol_log() {
    refresh_duration();

    oss_row_evol << iter_global << ",";
    write_csv_cost(oss_row_evol, global_best_upper->upper_cost);
    oss_row_evol << ",";
    write_csv_cost(oss_row_evol, last_candidate_upper);
    oss_row_evol << "," << std::fixed << std::setprecision(6) << acceptance_deviation() << ",";
    oss_row_evol << (last_destroy_idx >= 0 ? kDestroyLabels[last_destroy_idx] : "none") << ",";
    oss_row_evol << (last_repair_idx >= 0 ? kRepairLabels[last_repair_idx] : "none") << ",";
    oss_row_evol << std::fixed << std::setprecision(2) << instance->get_evals() << ","
                 << duration.count()
                 << "\n";
}

void TwoStageAlns::save_log_for_solution() {
    const fs::path directory = fs::path(preprocessor->params.k_stats_path)
        / this->name
        / instance->instance_name_
        / std::to_string(seed);

    create_directories_if_not_exists(directory);

    const std::string file_name = "solution." + instance->instance_name_ + ".txt";
    log_solution.open(directory / file_name);
    log_solution << std::fixed << std::setprecision(5) << global_best_lower->lower_cost << std::endl;
    for (int i = 0; i < follower->num_routes; ++i) {
        for (int j = 0; j < follower->lower_num_nodes_per_route[i]; ++j) {
            log_solution << follower->lower_routes[i][j] << ",";
        }
        log_solution << std::endl;
    }
    log_solution.close();
}

void TwoStageAlns::save_log_for_profile() const {
    const fs::path directory = fs::path(preprocessor->params.k_stats_path)
        / this->name
        / instance->instance_name_
        / std::to_string(seed);

    create_directories_if_not_exists(directory);

    const std::string file_name = "profile." + instance->instance_name_ + ".txt";
    std::ofstream log_profile(directory / file_name);

    const double destroy_total = std::accumulate(destroy_time_sec.begin(), destroy_time_sec.end(), 0.0);
    const double repair_total = std::accumulate(repair_time_sec.begin(), repair_time_sec.end(), 0.0);
    const double follower_total = prof_follower_run_sec + prof_follower_refine_sec;
    const double measured_total = destroy_total + repair_total + follower_total;

    auto write_profile_row = [&](const char* label, const double time_sec, const long calls) {
        const double avg_ms_per_call = calls > 0 ? (1e3 * time_sec / static_cast<double>(calls)) : 0.0;
        const double share_pct = measured_total > 0.0 ? (100.0 * time_sec / measured_total) : 0.0;
        log_profile << std::left << std::setw(24) << label
                    << std::right
                    << std::setw(14) << std::fixed << std::setprecision(6) << time_sec
                    << std::setw(12) << calls
                    << std::setw(16) << std::fixed << std::setprecision(3) << avg_ms_per_call
                    << std::setw(14) << std::fixed << std::setprecision(3) << share_pct
                    << "\n";
    };

    log_profile << "two_stage_alns_profile\n";
    log_profile << std::left
                << std::setw(24) << "component"
                << std::right
                << std::setw(14) << "time_sec"
                << std::setw(12) << "calls"
                << std::setw(16) << "avg_ms/call"
                << std::setw(14) << "share(%)"
                << "\n";
    log_profile << std::string(80, '-') << "\n";

    for (int idx = 0; idx < kNumDestroyMethods; ++idx) {
        write_profile_row(kDestroyLabels[idx], destroy_time_sec[idx], destroy_calls[idx]);
    }
    for (int idx = 0; idx < kNumRepairMethods; ++idx) {
        write_profile_row(kRepairLabels[idx], repair_time_sec[idx], repair_calls[idx]);
    }
    write_profile_row("follower_run", prof_follower_run_sec, prof_follower_run_calls);
    write_profile_row("follower_refine", prof_follower_refine_sec, preprocessor->params.enable_final_refinement ? 1L : 0L);

    log_profile << "\n";
    log_profile << "alns_iters=" << alns_iter << "\n";
    log_profile << "global_iters=" << iter_global << "\n";
    log_profile << "final_upper=" << std::fixed << std::setprecision(6) << global_best_upper->upper_cost << "\n";
    log_profile << "final_lower=" << std::fixed << std::setprecision(6) << global_best_lower->lower_cost << "\n";
}

void TwoStageAlns::write_csv_cost(std::ostream& os, const double value) {
    if (std::isnan(value)) {
        os << "nan";
    } else if (value > 1e6) {
        os << std::scientific << std::setprecision(3) << value << std::defaultfloat;
    } else {
        os << std::fixed << std::setprecision(3) << value << std::defaultfloat;
    }
}
