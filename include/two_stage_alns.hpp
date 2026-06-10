//
// Created by Yinghao Qin on 26/04/2026.
//

#ifndef TWO_STAGE_ALNS_HPP
#define TWO_STAGE_ALNS_HPP

#include "case.hpp"
#include "follower.hpp"
#include "heuristic_interface.hpp"
#include "individual.hpp"
#include "initializer.hpp"
#include "stats_interface.hpp"

#include <array>
#include <memory>

class TwoStageAlns final : public HeuristicInterface, public StatsInterface {
public:
    bool enable_logging;
    int stop_criteria;

    long iter_global;         // Global ALNS iteration counter.
    long alns_iter;           // ALNS iterations completed in the current run.
    long alns_idle_iter;      // Consecutive ALNS iterations without a new stage best.

    std::unique_ptr<Individual> current;           // Current ALNS incumbent.
    std::unique_ptr<Individual> alns_candidate;    // Scratch solution rebuilt after destroy/repair.
    std::unique_ptr<Individual> global_best_upper; // Best upper-level solution seen during ALNS.
    std::unique_ptr<Individual> global_best_lower; // Final follower-evaluated solution.

    Initializer* initializer;
    Follower* follower;

    TwoStageAlns(int seed, Case* instance, Preprocessor* preprocessor);
    ~TwoStageAlns() override;

    double run() override;
    void initialize_heuristic() override;
    void run_heuristic() override;

    void open_log_for_evolution() override;
    void close_log_for_evolution() override;
    void flush_row_into_evol_log() override;
    void save_log_for_solution() override;
    void save_log_for_profile() const;

private:
    struct HistoryDestroyCandidate {
        int customer;
        double score;
    };

    struct ScratchBuffers {
        std::vector<int> customer_pool;
        std::vector<int> selected_customers;
        std::vector<int> rebuilt_demands;
        std::vector<std::vector<int>> rebuilt_routes;
        std::vector<char> node_marks;
        std::vector<HistoryDestroyCandidate> history_candidates;
    };

    struct WorkingSolution {
        std::vector<std::vector<int>> routes;
        std::vector<int> route_demands;
        std::vector<int> removed_customers;
        double total_cost{};
    };

    enum class DestroyMethod : int { Random = 0, Geo = 1, History = 2 };
    enum class RepairMethod : int { Regret2 = 0, Regret2Randomized = 1 };

    static constexpr int kNumDestroyMethods = 3;
    static constexpr int kNumRepairMethods = 2;

    double start_deviation{};
    double adaptive_decay{};
    double score_accepted{};
    double score_improved{};
    double score_global_best{};
    double min_destroy_ratio{};
    double max_destroy_ratio{};
    int min_destroy_abs{};
    int max_destroy_abs{};
    double geo_destroy_rand_factor{};
    double repair_rand_factor{};

    static constexpr double kHistoryDestroyRandFactor = 5.0;

    static inline constexpr std::array<const char*, kNumDestroyMethods> kDestroyLabels = {
        "random_destroy",
        "geo_destroy",
        "history_destroy"
    };
    static inline constexpr std::array<const char*, kNumRepairMethods> kRepairLabels = {
        "regret2",
        "regret2_randomized"
    };

    std::array<double, kNumDestroyMethods> destroy_weights{};
    std::array<double, kNumRepairMethods> repair_weights{};
    std::array<long, kNumDestroyMethods> destroy_calls{};
    std::array<long, kNumRepairMethods> repair_calls{};
    std::array<long, kNumDestroyMethods> destroy_successes{};
    std::array<long, kNumRepairMethods> repair_successes{};
    std::array<double, kNumDestroyMethods> destroy_time_sec{};
    std::array<double, kNumRepairMethods> repair_time_sec{};

    std::vector<std::vector<double>> best_arc_cost;
    ScratchBuffers scratch_buffers;

    double stage_best_upper_cost;
    int last_destroy_idx;
    int last_repair_idx;
    bool last_candidate_built;
    bool last_candidate_accepted;
    bool last_candidate_improved;
    bool last_candidate_global_best;
    double last_candidate_upper;

    double prof_follower_run_sec;
    double prof_follower_refine_sec;
    long prof_follower_run_calls;

    void reset_adaptive_state();
    void refresh_duration();
    [[nodiscard]] bool within_global_budget();
    void construct_initial_seed(Individual* seed_solution) const;
    void update_global_best_upper(const Individual& ind) const;
    void run_final_follower();

    static void export_working_solution(const Individual* ind, WorkingSolution& work);
    static void load_working_solution(Individual* ind, const WorkingSolution& work) ;
    int calc_destroy_count();
    void remove_customers_from_routes(WorkingSolution& work, const std::vector<int>& selected_customers);
    void destroy_random(WorkingSolution& work);
    void destroy_geo(WorkingSolution& work);
    void destroy_history(WorkingSolution& work);
    void apply_destroy(WorkingSolution& work, DestroyMethod method);
    [[nodiscard]] bool find_best_insertion(const std::vector<int>& route, int customer, int& best_pos, double& best_delta) const;
    [[nodiscard]] double route_cost_for_new_customer(int customer) const;
    void insert_customer(WorkingSolution& work, int route_idx, int customer, int insert_pos, double route_delta) const;
    bool repair_regret2(WorkingSolution& work, double randomized_factor);
    bool apply_repair(WorkingSolution& work, RepairMethod method);
    [[nodiscard]] int roulette_wheel_selection(const double* weights, int count);
    [[nodiscard]] double calc_score(bool accepted, bool improved, bool new_best_upper) const;
    [[nodiscard]] double acceptance_deviation() const;
    [[nodiscard]] bool should_accept(double candidate_upper) const;
    void update_arc_history(const Individual& ind);

    static void write_csv_cost(std::ostream& os, double value);
};

#endif //TWO_STAGE_ALNS_HPP
