#include "ma_profile.hpp"

#include "case.hpp"

namespace ma_profile {

const char* part_name(Part part) {
    switch (part) {
        case Part::Other: return "other";
        case Part::Init: return "init";
        case Part::LsProbe: return "ls_probe";
        case Part::LsContinuation: return "ls_continuation";
        case Part::LsStatic: return "ls_static";
        case Part::Follower: return "follower";
        case Part::Split: return "split";
        case Part::FinalRefine: return "final_refine";
        case Part::Context: return "context";
        case Part::LearnerSelect: return "learner_select";
        case Part::LearnerUpdate: return "learner_update";
        case Part::Feedback: return "feedback";
        case Part::ParentPool: return "parent_pool";
        case Part::Reproduction: return "reproduction";
        case Part::Population: return "population";
        case Part::Logging: return "logging";
        case Part::Count: break;
    }
    return "unknown";
}

bool budget_part(Part part) {
    switch (part) {
        case Part::Init:
        case Part::LsProbe:
        case Part::LsContinuation:
        case Part::LsStatic:
        case Part::Follower:
        case Part::Split:
        case Part::FinalRefine:
            return true;
        default:
            return false;
    }
}

Profiler& profiler() {
    static thread_local Profiler instance;  // one per thread: runs with -mth 1 profile independently
    return instance;
}

void Profiler::start(const Case* case_instance) {
    instance = case_instance;
    data = {};
    stack.clear();
    stack.push_back(Part::Other);
    active = true;
    last_time = Clock::now();
    last_calls = instance != nullptr ? instance->get_distance_calls() : 0;
}

void Profiler::stop() {
    if (!active) return;
    charge_current(Clock::now(), instance != nullptr ? instance->get_distance_calls() : 0);
    stack.clear();
    active = false;
}

void Profiler::charge_current(Clock::time_point now, std::uint64_t calls) {
    const auto index = static_cast<std::size_t>(stack.empty() ? Part::Other : stack.back());
    data.seconds[index] += std::chrono::duration<double>(now - last_time).count();
    data.distance_calls[index] += calls - last_calls;
    last_time = now;
    last_calls = calls;
}

void Profiler::enter(Part part) {
    charge_current(Clock::now(), instance != nullptr ? instance->get_distance_calls() : 0);
    stack.push_back(part);
    ++data.entries[static_cast<std::size_t>(part)];
}

void Profiler::leave() {
    charge_current(Clock::now(), instance != nullptr ? instance->get_distance_calls() : 0);
    if (stack.size() > 1) stack.pop_back();
}

}  // namespace ma_profile
