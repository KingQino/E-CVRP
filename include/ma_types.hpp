//
// Created by Yinghao Qin on 11/10/2026.
//
// Shared enums and configuration for the budget-aware bilevel memetic algorithm (BA-BMA, "MA").
// Kept separate so that parameters.hpp does not depend on the MA modules.
//

#ifndef MA_TYPES_HPP
#define MA_TYPES_HPP

#include <array>
#include <cstdint>

// Upper-level local-search action chosen for one individual. Every individual first receives the
// weak probe; the action decides where its RVND session terminates.
enum class LocalSearchAction {
    Weak,   // stop after the weak probe: accepted-move cap = weak fraction of (customers + routes)
    Medium, // continue up to the medium accepted-move cap
    Strong, // continue up to the strong accepted-move cap and the strong distance-call cap
};
constexpr int LOCAL_SEARCH_ACTION_COUNT = 3;

// Unrestricted RVND (run until all eight neighbourhoods fail, i.e. a local optimum) is deliberately
// not an action: it is only used by Elite Unrestricted and by the static baseline (-ma_static_unrestricted 1).

// How local-search actions are allocated across the population.
enum class LocalSearchPolicy {
    Static,              // every individual receives the same action (or unrestricted RVND)
    RandomMixed,         // progressive random mix of weak/medium/strong
    MatchedRandom,       // given weak/medium/strong shares among the individuals still eligible after the probe,
                         // assigned at random over the whole generation
    OnlineIndividual,    // online contextual learner (LinUCB) per individual
    Rank,                // rule-based adaptive allocation (RAA): the given shares, assigned over the whole generation
                         // by probe efficiency (stronger actions first; ties: lower route cost after the probe)
};

// Context seen by the online learner's reward and cost models.
enum class LocalSearchContextMode {
    Full,     // all 13 features
    NoProbe,  // probe success rate, probe improvement, probe efficiency and probe cost set to 0 (the probe still runs)
    None,     // context-free: only the constant term, i.e. one reward and one cost estimate per action
};

// Weak/medium/strong shares used by the matched-random and rank policies.
using LocalSearchActionShares = std::array<double, 3>;
// Weights of the lower-archive (L), gamma-crossing (G) and parent-use (P) reward terms.
using LocalSearchRewardWeights = std::array<double, 3>;

// Coupled weak/medium/strong depth profiles.
enum class LocalSearchDepthProfile { D1, D2, D3, D4, D5, D6 };

struct LocalSearchDepthConfig {
    double weak_move_fraction{0.02};
    double medium_move_fraction{0.10};
    double strong_move_fraction{0.30};
    std::uint64_t strong_weak_call_multiplier{128};       // strong distance-call cap = multiplier x weak-probe calls
};

[[nodiscard]] LocalSearchDepthConfig local_search_depth_config(LocalSearchDepthProfile profile);
[[nodiscard]] const char* local_search_depth_profile_name(LocalSearchDepthProfile profile);
[[nodiscard]] const char* local_search_action_name(LocalSearchAction action);
[[nodiscard]] const char* local_search_policy_name(LocalSearchPolicy policy);
[[nodiscard]] const char* local_search_context_mode_name(LocalSearchContextMode mode);

#endif //MA_TYPES_HPP
