//
// Created by Yinghao Qin on 08/06/2025.
//

#include <omp.h>  // OpenMP header
#include <filesystem>
#include "parameters.hpp"
#include "command_line.hpp"
#include "case.hpp"
#include "preprocessor.hpp"
#include "magic_enum.hpp"
#include "stats_interface.hpp"
#include "lahc.hpp"

namespace fs = std::filesystem;

using namespace std;
using namespace magic_enum;

#define MAX_TRIALS 10

void run_algorithm(const int run, const Parameters* params, vector<double>& perf_of_trials) {
    auto* instance = new Case(params->k_data_path, params->instance);
    auto* preprocessor = new Preprocessor(*instance, *params);

    switch (params->algorithm) {
        case Algorithm::LAHC: {
            const auto lahc = new Lahc(run, instance, preprocessor);
            lahc->run();

            // Prevent race condition on shared vector
            #pragma omp critical
            {
                perf_of_trials[run - 1] = lahc->global_best->lower_cost;
            }

            delete lahc;
            break;
        }
        default: {;
            cerr << "Unsupported algorithm! " << endl;
            exit(EXIT_FAILURE);
        }
    }

    delete preprocessor;
    delete instance;
}

int main(int argc, char* argv[]) {
    // === 1. Parameter parsing ===
    Parameters params;
    CommandLine cmd(argc, argv);
    cmd.parse_parameters(params);

    // === 2. Performance indicator for multiple trials ===
    vector<double> perf_of_trials(MAX_TRIALS, 0.0);

    // === 3. Algorithm running ===
    if (!params.enable_multithreading) {
        run_algorithm(1, &params, std::ref(perf_of_trials));
    } else {
        #pragma omp parallel for default(none) shared(params, perf_of_trials)
        for (int run = 1; run <= MAX_TRIALS; ++run) {
            run_algorithm(run, &params, perf_of_trials);
        }
    }

    // === 4. Determining file path ===
    fs::path stats_dir = fs::path(params.k_stats_path) /
                         enum_name(params.algorithm) /
                         params.instance.substr(0, params.instance.find('.'));
    fs::path stats_file = stats_dir / ("stats." + params.instance);

    // === 5. Saving statistical info ===
    StatsInterface::create_directories_if_not_exists(stats_dir);
    StatsInterface::stats_for_multiple_trials(stats_file, perf_of_trials);

    // === 6. Console output ===
    if (!params.enable_multithreading) {
        std::cout << std::fixed << std::setprecision(2) << perf_of_trials[0] << std::endl;
    } else {
        Indicators indicators = StatsInterface::calculate_statistical_indicators(perf_of_trials);
        if (indicators.min < 1e-6) {
            std::cout << std::fixed << std::setprecision(2) << 99999999.99 << std::endl;  // If the minimum value is less than 1e6, print a large number
        } else {
            std::cout << std::fixed << std::setprecision(2) << indicators.avg << std::endl;
        }
    }

    return 0;
}
