//
// Created by Yinghao Qin on 11/10/2026.
//
// Shared enums and configuration for the budget-aware bilevel memetic algorithm (BA-BMA, "MA").
// Kept separate so that parameters.hpp does not depend on the MA modules.
//

#ifndef MA_TYPES_HPP
#define MA_TYPES_HPP

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
    MatchedRandom,       // fixed random shares matching the online learner's aggregate allocation
    OnlineIndividual,    // online contextual learner (LinUCB) per individual
};

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

#endif //MA_TYPES_HPP
