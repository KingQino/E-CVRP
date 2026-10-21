//
// Search Trajectory Network (STN) logging; see include/stn_logger.hpp.
//

#include "stn_logger.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace fs = std::filesystem;

StnLogger::StnLogger(const Case& instance, const fs::path& directory, const std::uint64_t budget_calls,
                     const double grid, const bool write_solutions)
    : instance_(instance), write_solutions_(write_solutions) {
    if (!(grid > 0.0 && grid <= 1.0)) {
        throw std::invalid_argument("stn_grid must be in (0, 1]");
    }
    grid_calls_ = std::max<std::uint64_t>(1, static_cast<std::uint64_t>(std::llround(static_cast<double>(budget_calls) * grid)));

    fs::create_directories(directory);
    const std::string name = instance.instance_name_;
    {
        std::ofstream meta(directory / ("stn_meta." + name + ".csv"));
        meta << "key,value\n"
             << "instance," << name << "\n"
             << "problem_size," << instance.problem_size_ << "\n"
             << "num_customers," << instance.num_customer_ << "\n"
             << "budget_distance_calls," << budget_calls << "\n"
             << "grid," << grid << "\n"
             << "grid_distance_calls," << grid_calls_ << "\n"
             << "fitness,upper-level objective f_U\n"
             << "signature,FNV-1a 64-bit of canonical customer routes\n";
    }
    raw_.open(directory / ("stn_raw." + name + ".csv"));
    raw_ << "step,distance_calls,evals,fitness,signature\n";
    if (write_solutions_) {
        solutions_.open(directory / ("stn_solutions." + name + ".csv"));
        solutions_ << "signature,routes\n";
    }
}

StnLogger::~StnLogger() {
    raw_.close();
    if (write_solutions_) solutions_.close();
}

void StnLogger::observe(const Individual& representative, const long step, const std::uint64_t distance_calls,
                        const bool force) {
    if (!(force || due(distance_calls)) || distance_calls == last_calls_) {
        return;
    }
    write({step, distance_calls, representative.upper_cost, canonical_routes(representative, instance_)});
}

void StnLogger::offer(const Individual& representative, const long step, const std::uint64_t distance_calls) {
    if (distance_calls == last_calls_) {
        return;
    }
    Observation observation{step, distance_calls, representative.upper_cost, canonical_routes(representative, instance_)};
    if (due(distance_calls)) {
        write(observation);
    } else {
        pending_ = std::move(observation);
        has_pending_ = true;
    }
}

void StnLogger::finish() {
    if (has_pending_ && pending_.distance_calls != last_calls_ && (last_calls_ == UINT64_MAX || pending_.distance_calls > last_calls_)) {
        write(pending_);
    }
    raw_.flush();
    if (write_solutions_) solutions_.flush();
}

void StnLogger::write(const Observation& observation) {
    const std::string hash = signature(observation.canonical);
    const double evals = instance_.problem_size_ > 0
        ? static_cast<double>(observation.distance_calls) / static_cast<double>(instance_.problem_size_) : 0.0;
    char row[192];
    std::snprintf(row, sizeof(row), "%ld,%llu,%.4f,%.6f,%s\n", observation.step,
                  static_cast<unsigned long long>(observation.distance_calls), evals, observation.fitness, hash.c_str());
    raw_ << row;

    if (write_solutions_ && seen_.insert(hash).second) {
        solutions_ << hash << "," << observation.canonical << "\n";
    }

    last_calls_ = observation.distance_calls;
    has_pending_ = false;
    // Next grid point strictly after this observation.
    next_calls_ = (observation.distance_calls / grid_calls_ + 1) * grid_calls_;
}

std::string StnLogger::canonical_routes(const Individual& individual, const Case& instance) {
    const int first_customer = instance.num_depot_;
    const int last_customer = instance.num_depot_ + instance.num_customer_ - 1;

    std::vector<std::vector<int>> routes;
    routes.reserve(static_cast<std::size_t>(individual.num_routes));
    for (int r = 0; r < individual.num_routes; ++r) {
        std::vector<int> customers;
        for (int j = 0; j < individual.num_nodes_per_route[r]; ++j) {
            const int node = individual.routes[r][j];
            if (node >= first_customer && node <= last_customer) {
                customers.push_back(node);
            }
        }
        if (customers.empty()) continue;
        if (customers.front() > customers.back()) {
            std::reverse(customers.begin(), customers.end());
        }
        routes.push_back(std::move(customers));
    }
    std::sort(routes.begin(), routes.end());

    std::string out;
    for (std::size_t r = 0; r < routes.size(); ++r) {
        if (r > 0) out += '|';
        for (std::size_t j = 0; j < routes[r].size(); ++j) {
            if (j > 0) out += '-';
            out += std::to_string(routes[r][j]);
        }
    }
    return out;
}

std::string StnLogger::signature(const std::string& canonical) {
    std::uint64_t hash = 14695981039346656037ULL;  // FNV-1a 64-bit offset basis
    for (const unsigned char c : canonical) {
        hash ^= c;
        hash *= 1099511628211ULL;                  // FNV-1a 64-bit prime
    }
    char buffer[17];
    std::snprintf(buffer, sizeof(buffer), "%016llx", static_cast<unsigned long long>(hash));
    return buffer;
}
