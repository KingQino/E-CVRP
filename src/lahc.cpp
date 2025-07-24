//
// Created by Yinghao Qin on 09/06/2025.
//

#include "lahc.hpp"

Lahc::Lahc(const int seed, Case* instance, Preprocessor* preprocessor)
: HeuristicInterface("LAHC", seed, instance, preprocessor) {
    enable_logging = preprocessor->params.enable_logging;
    stop_criteria = preprocessor->params.stop_criteria;

    iter = 0L;
    idle_iter = 0L;
    history_length = static_cast<long>(preprocessor->params.history_length);
    num_successful_moves_per_history = 0.;
    ratio_successful_moves = 1.0;
    low_opt_trigger_threshold = preprocessor->params.low_opt_trigger_threshold;
    low_opt_trigger_margin = preprocessor->params.low_opt_trigger_margin;
    history_list = vector<double>(history_length);
    current = nullptr;
    best_upper_cost = std::numeric_limits<double>::max();
    global_best = make_unique<Individual>(instance, preprocessor);
    history_noise = uniform_real_distribution(preprocessor->params.noise_lb, preprocessor->params.noise_ub); // Noise for history list
    his_refill_idx = 0;
    restart_idx = 0;

    initializer = new Initializer(random_engine, instance, preprocessor);
    leader = new LeaderLahc(random_engine, instance, preprocessor);
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
    vector<vector<int>> routes = initializer->routes_constructor_with_hien_method();
    current = new Individual(instance, preprocessor, routes,instance->compute_total_distance(routes),
        instance->compute_demand_sum_per_route(routes));
    routes.clear();
    routes.shrink_to_fit();

    this->iter = 0L;
    this->idle_iter = 0L;
    this->ratio_successful_moves = 1.0; // the largest decimal value
    this->num_successful_moves_per_history = static_cast<double>(history_length);

    leader->fully_greedy_local_optimum(current);
    best_upper_cost = current->upper_cost;
    for (int i =0; i < history_length; i++) {
        history_list[i] = current->upper_cost * history_noise(random_engine);
    }
}

void Lahc::restart_heuristic() {
    *current = *global_best; // Reset current solution to the global best
    leader->load_individual(current);

    iter = 0L;
    idle_iter = 0L;
    ratio_successful_moves = 1.0; // the largest decimal value
    num_successful_moves_per_history = static_cast<double>(history_length);

    leader->perturbation(100);
    leader->fully_greedy_local_optimum(current);
}

void Lahc::run_heuristic() {
    bool within_budget;

    do {

        const double current_cost = leader->upper_cost;

        const auto v = iter % history_length;
        double history_cost = history_list[v];

        if (v == 0L) {
            history_list_metrics = calculate_statistical_indicators(history_list);

            ratio_successful_moves = num_successful_moves_per_history / static_cast<double>(history_length);

            flush_row_into_evol_log();
            num_successful_moves_per_history = 0.;
            his_refill_idx++;
        }

        const bool has_moved = leader->random_walk(history_cost);
        if (has_moved) {
            leader->export_individual(current);
            num_successful_moves_per_history++;
        }
        double candidate_cost = leader->upper_cost;
        best_upper_cost = std::min(best_upper_cost, candidate_cost);

        // idle judgement and counting
        idle_iter = candidate_cost >= current_cost ? idle_iter + 1 : 0;
        // update the history list
        history_list[v] = std::min(candidate_cost, history_cost);

        iter++;
        duration = std::chrono::high_resolution_clock::now() - start;

        if (ratio_successful_moves < low_opt_trigger_threshold &&
            has_moved &&
            candidate_cost < best_upper_cost * low_opt_trigger_margin) {

            follower->run(current);
            if (current->lower_cost < global_best->lower_cost) {
                *global_best = *current;
            }
        }

        within_budget = stop_criteria == 0 ? !stop_criteria_max_evals() : !stop_criteria_max_exec_time(duration);
    } while ((iter < 100'000L || idle_iter < iter / 5) && ratio_successful_moves > 0.001 && within_budget);

    ++restart_idx;
}

void Lahc::run() {
    // Initialize time variables
    start = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration<double>::zero();

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

    follower->run(global_best.get());

    if (enable_logging) {
        flush_row_into_evol_log();
        close_log_for_evolution();  // Close log if logging is enabled
        save_log_for_solution();    // Save the log if logging is enabled
    }
}

void Lahc::open_log_for_evolution() {
    const fs::path directory = fs::path(preprocessor->params.k_stats_path)
    / this->name
    / instance->instance_name_
    / to_string(seed);

    create_directories_if_not_exists(directory);

    const string file_name = "evols." + instance->instance_name_ + ".csv";
    log_evolution.open(directory / file_name);
    log_evolution << "iters,global_best,min,max,mean,std,ratio_moves,evals,time\n";
}

void Lahc::close_log_for_evolution() {
    log_evolution << oss_row_evol.str();
    oss_row_evol.clear();
    log_evolution.close();
}

void Lahc::flush_row_into_evol_log() {
    oss_row_evol << iter << ",";

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
    follower->run(global_best.get());
    for (int i = 0; i < follower->num_routes; ++i) {
        for (int j = 0; j < follower->lower_num_nodes_per_route[i]; ++j) {
            log_solution << follower->lower_routes[i][j] << ",";
        }
        log_solution << endl;
    }
    log_solution.close();
}

