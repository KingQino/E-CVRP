#pragma once

// Runtime profile of the MA (-ma_profile 1). Wall-clock time and distance calls are attributed exclusively to the
// innermost open part: entering a part pauses the enclosing one. Time outside every part goes to Other.
// The profile only measures; it never changes the search.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

class Case;

namespace ma_profile {

enum class Part : std::size_t {
    Other = 0,       // generation bookkeeping outside every named part
    Init,            // initial population (construction heuristic and split)
    LsProbe,         // Weak probe (RVND session start and first moves)
    LsContinuation,  // Medium/Strong continuation
    LsStatic,        // static policies: the whole local search of an individual
    Follower,        // charging of the gamma-filtered individuals (SE/RE)
    Split,           // Prins split and route loading of the offspring
    FinalRefine,     // final exact charging refinement
    Context,         // context features: edge signatures, structural distance, dispersion
    LearnerSelect,   // learner scoring and action choice
    LearnerUpdate,   // reward computation and Sherman-Morrison model updates
    Feedback,        // lower-archive and parent-use credit
    ParentPool,      // sorting by route cost, duplicate removal, best solution
    Reproduction,    // tournament selection, crossover, mutation
    Population,      // assembly of the next population (without the split)
    Logging,         // log and result files
    Count
};

constexpr std::size_t kPartCount = static_cast<std::size_t>(Part::Count);

const char* part_name(Part part);

// True for the parts whose distance reads are counted in the evaluation budget.
bool budget_part(Part part);

struct Totals {
    std::array<double, kPartCount> seconds{};
    std::array<std::uint64_t, kPartCount> distance_calls{};
    std::array<std::uint64_t, kPartCount> entries{};
};

class Profiler {
public:
    // Starts a profile for one run; the distance counter is read from instance.
    void start(const Case* instance);
    void stop();
    [[nodiscard]] bool enabled() const { return active; }
    [[nodiscard]] const Totals& totals() const { return data; }

    void enter(Part part);
    void leave();

private:
    using Clock = std::chrono::steady_clock;
    void charge_current(Clock::time_point now, std::uint64_t calls);

    bool active{false};
    const Case* instance{nullptr};
    Totals data{};
    std::vector<Part> stack;
    Clock::time_point last_time{};
    std::uint64_t last_calls{0};
};

// One profiler per thread (each run is single-threaded).
Profiler& profiler();

// Unscoped section, for code whose variables must stay visible after the section; begin and end must pair up.
inline void begin(Part part) {
    if (profiler().enabled()) profiler().enter(part);
}
inline void end() {
    if (profiler().enabled()) profiler().leave();
}

class Scope {
public:
    explicit Scope(Part part) : enabled(profiler().enabled()) {
        if (enabled) profiler().enter(part);
    }
    ~Scope() {
        if (enabled) profiler().leave();
    }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;

private:
    bool enabled;
};

}  // namespace ma_profile
