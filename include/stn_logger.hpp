//
// Search Trajectory Network (STN) logging, for the STNs-v2 tool (https://github.com/gabro8a/STNs-v2).
//
// The logger is passive: it reads the representative solution (B-LAHC: the current solution; BA-BMA: the best
// upper-level individual of the generation) and writes it to a raw CSV. It never calls Case::get_distance and never
// draws random numbers, so it does not change the evaluation budget or the search.
//
// Raw log (stats/<ALG>/<instance>/<seed>/stn_raw.<instance>.csv), one row per observation:
//   step,distance_calls,evals,fitness,signature
// step: LAHC global iteration or MA generation; distance_calls: exact budget counter (evals = distance_calls /
// problem_size); fitness: upper-level objective f_U with full precision; signature: canonical routing hash.
//
// An observation is written at the first observation point at or after each grid point k * grid * budget
// (k = 0, 1, ...), so later sampling intervals that are multiples of the grid can be extracted without rerunning.
// Optionally, stn_solutions.<instance>.csv maps each signature to its canonical routes (written once per signature).
//

#ifndef STN_LOGGER_HPP
#define STN_LOGGER_HPP

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "case.hpp"
#include "individual.hpp"

class StnLogger {
public:
    // budget_calls: the max-evals budget in distance calls; grid: observation grid as a fraction of the budget.
    StnLogger(const Case& instance, const std::filesystem::path& directory, std::uint64_t budget_calls, double grid,
              bool write_solutions);
    ~StnLogger();

    // Cheap check for the hot path: is an observation due at this budget counter?
    [[nodiscard]] bool due(const std::uint64_t distance_calls) const { return distance_calls >= next_calls_; }

    // Records the representative if an observation is due (or always when force is true).
    void observe(const Individual& representative, long step, std::uint64_t distance_calls, bool force = false);
    // Like observe, but when no observation is due the representative is kept as pending, and finish() writes the
    // last pending one. Used when the representative object may be overwritten before the run ends (MA).
    void offer(const Individual& representative, long step, std::uint64_t distance_calls);
    void finish();

    // Canonical routing: customers only (depot and charging stations removed), each route oriented so that its
    // smaller endpoint comes first, routes sorted; serialized as "c-c-c|c-c|...".
    static std::string canonical_routes(const Individual& individual, const Case& instance);
    static std::string signature(const std::string& canonical); // 64-bit FNV-1a, 16 lowercase hex characters

private:
    struct Observation {
        long step = 0;
        std::uint64_t distance_calls = 0;
        double fitness = 0.0;
        std::string canonical;
    };
    void write(const Observation& observation);

    const Case& instance_;
    std::uint64_t grid_calls_;
    std::uint64_t next_calls_ = 0;
    std::uint64_t last_calls_ = UINT64_MAX;
    std::ofstream raw_;
    std::ofstream solutions_;
    bool write_solutions_;
    std::unordered_set<std::string> seen_;
    Observation pending_;
    bool has_pending_ = false;
};

#endif //STN_LOGGER_HPP
