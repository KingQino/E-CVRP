//
// Created by Yinghao Qin on 09/06/2025.
//

#include "lahc.hpp"

using namespace std;

Lahc::Lahc(const int seed, Case* instance, Preprocessor* preprocessor)
: HeuristicInterface("LAHC", seed, instance, preprocessor) {
    enable_logging = preprocessor->params.enable_logging;
    stop_criteria = preprocessor->params.stop_criteria;

    iter = 0L;
    iter_global = 0L;
    idle_iter = 0L;
    history_length = static_cast<long>(preprocessor->params.history_length);
    num_successful_moves_per_history = 0.;
    ratio_successful_moves = 1.0;
    low_opt_trigger_margin = preprocessor->params.low_opt_trigger_margin;
    history_list = vector<double>(history_length);
    current = nullptr;
    global_best_upper_cost = std::numeric_limits<double>::max();
    global_best = make_unique<Individual>(instance, preprocessor);
    history_noise = uniform_real_distribution(preprocessor->params.noise_lb, preprocessor->params.noise_ub); // Noise for history list
    restart_idx = 0;
    prof_leader_greedy_sec = 0.0;
    prof_leader_neigh_sec = 0.0;
    prof_follower_run_sec = 0.0;
    prof_follower_refine_sec = 0.0;
    prof_leader_neigh_calls = 0L;
    prof_follower_run_calls = 0L;

    initializer = new Initializer(random_engine, instance, preprocessor);
    leader = new LeaderSingle(random_engine, instance, preprocessor);
    follower = new Follower(instance, preprocessor);
}

Lahc::~Lahc() {
    delete initializer;
    delete leader;
    delete follower;
    delete current;
}

void Lahc::initialize_heuristic() {
    delete current;
    ++restart_idx;
    // vector<vector<int>> routes = initializer->routes_constructor_with_hien_method();
    vector<vector<int>> routes = initializer->routes_constructor_with_split();
    current = new Individual(instance, preprocessor, routes,instance->compute_total_distance(routes),
        instance->compute_demand_sum_per_route(routes));
    routes.clear();
    routes.shrink_to_fit();

    if (enable_logging) {
        const auto t0 = std::chrono::high_resolution_clock::now();
        leader->greedy_descent(current);
        prof_leader_greedy_sec += std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();
    } else {
        leader->greedy_descent(current);
    }

    for (int i =0; i < history_length; i++) {
        history_list[i] = current->upper_cost * history_noise(random_engine);
    }

    if (enable_logging) {
        const auto t0 = std::chrono::high_resolution_clock::now();
        follower->run(current);
        prof_follower_run_sec += std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();
        ++prof_follower_run_calls;
    } else {
        follower->run(current);
    }

    if (current->lower_cost < global_best->lower_cost) {
        *global_best = *current;
    }

    this->global_best_upper_cost = std::min(global_best_upper_cost,current->upper_cost);

    this->iter = 0L;
    this->idle_iter = 0L;
    this->ratio_successful_moves = 1.0; // the largest decimal value
    this->num_successful_moves_per_history = static_cast<double>(history_length);
}

void Lahc::run_heuristic() {
    bool within_budget;
    const bool need_iteration_duration = (stop_criteria == 1) || enable_logging;

    do {

        const double current_cost = current->upper_cost;

        const auto v = iter % history_length;
        const double history_cost = history_list[v];

        if (v == 0L) {
            if (enable_logging) {
                history_list_metrics = calculate_statistical_indicators(history_list);
            }

            ratio_successful_moves = num_successful_moves_per_history / static_cast<double>(history_length);

            if (enable_logging) {
                flush_row_into_evol_log();
            }
            num_successful_moves_per_history = 0.;
        }

        bool has_moved;
        if (enable_logging) {
            const auto t0 = std::chrono::high_resolution_clock::now();
            has_moved = leader->neighbourhood_explore(current, history_cost);
            prof_leader_neigh_sec += std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();
            ++prof_leader_neigh_calls;
        } else {
            has_moved = leader->neighbourhood_explore(current, history_cost);
        }
        if (has_moved) num_successful_moves_per_history++;
        double candidate_cost = current->upper_cost;
        global_best_upper_cost = std::min(global_best_upper_cost, candidate_cost);

        // idle judgement and counting
        idle_iter = candidate_cost >= current_cost ? idle_iter + 1 : 0;
        // update the history list
        history_list[v] = std::min(candidate_cost, history_cost);

        iter++;
        iter_global++;
        if (need_iteration_duration) {
            duration = std::chrono::high_resolution_clock::now() - start;
        }

        if (has_moved && candidate_cost < global_best_upper_cost * low_opt_trigger_margin) {
            if (enable_logging) {
                const auto t0 = std::chrono::high_resolution_clock::now();
                follower->run(current);
                prof_follower_run_sec += std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();
                ++prof_follower_run_calls;
            } else {
                follower->run(current);
            }
            if (current->lower_cost < global_best->lower_cost) {
                *global_best = *current;
            }
        }

        within_budget = stop_criteria == 0 ? !stop_criteria_max_evals() : !stop_criteria_max_exec_time(duration);
    } while ((iter < 100'000L || idle_iter < iter / 50) && ratio_successful_moves > 0.001 && within_budget);

}

double Lahc::run() {
    // Initialize time variables
    start = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration<double>::zero();
    iter_global = 0L;
    restart_idx = 0;
    prof_leader_greedy_sec = 0.0;
    prof_leader_neigh_sec = 0.0;
    prof_follower_run_sec = 0.0;
    prof_follower_refine_sec = 0.0;
    prof_leader_neigh_calls = 0L;
    prof_follower_run_calls = 0L;

    if (enable_logging) {
        open_log_for_evolution();  // Open log if logging is enabled
    }


    switch (stop_criteria) {
        case 0:
            while (!stop_criteria_max_evals()) {
                initialize_heuristic();
                run_heuristic();
            }
            break;
        case 1:
            while (!stop_criteria_max_exec_time(duration)) {
                initialize_heuristic();
                run_heuristic();
                duration = std::chrono::high_resolution_clock::now() - start;
            }
            break;

        default:
            std::cerr << "Invalid stop criteria option!" << std::endl;
            break;
    }

    // Optional final exact refinement for the best upper-level solution
    if (preprocessor->params.enable_final_refinement) {
        if (enable_logging) {
            const auto t0 = std::chrono::high_resolution_clock::now();
            follower->refine(global_best.get());
            prof_follower_refine_sec += std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();
        } else {
            follower->refine(global_best.get());
        }
    } else {
            follower->run_with_variables(global_best.get());
    }

    if (enable_logging) {
        flush_row_into_evol_log();
        close_log_for_evolution();  // Close log if logging is enabled
        save_log_for_solution();    // Save the log if logging is enabled
        save_log_for_profile();     // Save lightweight timing profile
    }

    return global_best->lower_cost;
}

void Lahc::open_log_for_evolution() {
    const fs::path directory = fs::path(preprocessor->params.k_stats_path)
    / this->name
    / instance->instance_name_
    / to_string(seed);

    create_directories_if_not_exists(directory);

    const string file_name = "evols." + instance->instance_name_ + ".csv";
    log_evolution.open(directory / file_name);
    log_evolution << "restart,iter_global,global_best,min,max,mean,std,ratio_moves,evals,time\n";
}

void Lahc::close_log_for_evolution() {
    log_evolution << oss_row_evol.str();
    oss_row_evol.clear();
    log_evolution.close();
}

void Lahc::flush_row_into_evol_log() {
    oss_row_evol << restart_idx << "," << iter_global << ",";

    if (global_best->lower_cost > 1e6) {
        oss_row_evol << std::scientific << std::setprecision(3) << global_best->lower_cost;
    } else {
        oss_row_evol << std::fixed << std::setprecision(3) << global_best->lower_cost;
    }

    // Reset to default float formatting
    oss_row_evol << std::defaultfloat << ",";

    // Ensure fixed precision for history metrics
    oss_row_evol << std::fixed << std::setprecision(3)
                 << history_list_metrics.min << ","
                 << history_list_metrics.max << ","
                 << history_list_metrics.avg << ","
                 << history_list_metrics.std << ","
                 << ratio_successful_moves << ","
                 << std::fixed << std::setprecision(2)
                 << instance->get_evals() << ","
                 << duration.count()
                 << "\n";
}

void Lahc::save_log_for_solution() {
    const fs::path directory = fs::path(preprocessor->params.k_stats_path)
    / this->name
    / instance->instance_name_
    / to_string(seed);

    create_directories_if_not_exists(directory);

    const string file_name = "solution." + instance->instance_name_ + ".txt";

    log_solution.open(directory / file_name);
    log_solution << fixed << setprecision(5) << global_best->lower_cost << endl;
    for (int i = 0; i < follower->num_routes; ++i) {
        for (int j = 0; j < follower->lower_num_nodes_per_route[i]; ++j) {
            log_solution << follower->lower_routes[i][j] << ",";
        }
        log_solution << endl;
    }
    log_solution.close();
}

void Lahc::save_log_for_profile() const {
    const fs::path directory = fs::path(preprocessor->params.k_stats_path)
    / this->name
    / instance->instance_name_
    / to_string(seed);

    create_directories_if_not_exists(directory);

    const string file_name = "profile." + instance->instance_name_ + ".txt";
    std::ofstream log_profile(directory / file_name);

    const double leader_total = prof_leader_greedy_sec + prof_leader_neigh_sec;
    const double follower_total = prof_follower_run_sec + prof_follower_refine_sec;
    const double measured_total = leader_total + follower_total;
    const double leader_share = measured_total > 0.0 ? (100.0 * leader_total / measured_total) : 0.0;
    const double follower_share = measured_total > 0.0 ? (100.0 * follower_total / measured_total) : 0.0;

    auto write_profile_row = [&](const char* label, const double time_sec, const long calls, const double share_pct) {
        const double avg_ms_per_call = calls > 0 ? (1e3 * time_sec / static_cast<double>(calls)) : 0.0;
        log_profile << std::left << std::setw(20) << label
                    << std::right
                    << std::setw(14) << std::fixed << std::setprecision(6) << time_sec
                    << std::setw(12) << calls
                    << std::setw(16) << std::fixed << std::setprecision(3) << avg_ms_per_call
                    << std::setw(14) << std::fixed << std::setprecision(3) << share_pct
                    << "\n";
    };

    log_profile << "lahc_component_profile\n";
    log_profile << std::left
                << std::setw(20) << "component"
                << std::right
                << std::setw(14) << "time_sec"
                << std::setw(12) << "calls"
                << std::setw(16) << "avg_ms/call"
                << std::setw(14) << "share(%)"
                << "\n";
    log_profile << std::string(76, '-') << "\n";
    write_profile_row("leader_greedy", prof_leader_greedy_sec, std::max(1, restart_idx), measured_total > 0.0 ? (100.0 * prof_leader_greedy_sec / measured_total) : 0.0);
    write_profile_row("leader_neigh", prof_leader_neigh_sec, prof_leader_neigh_calls, measured_total > 0.0 ? (100.0 * prof_leader_neigh_sec / measured_total) : 0.0);
    write_profile_row("leader_total", leader_total, prof_leader_neigh_calls, leader_share);
    write_profile_row("follower_run", prof_follower_run_sec, prof_follower_run_calls, measured_total > 0.0 ? (100.0 * prof_follower_run_sec / measured_total) : 0.0);
    write_profile_row("follower_final", prof_follower_refine_sec, 1L, measured_total > 0.0 ? (100.0 * prof_follower_refine_sec / measured_total) : 0.0);
    write_profile_row("follower_total", follower_total, prof_follower_run_calls, follower_share);
    log_profile << "\n";

    if (const auto* single = dynamic_cast<const LeaderSingle*>(leader)) {
        single->write_operator_profile(log_profile);
    }

    log_profile.close();
}
