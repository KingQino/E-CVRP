//
// Created by Yinghao Qin on 11/10/2026.
//

#include "ma_reproduction.hpp"

#include <algorithm>
#include <cmath>
#include <set>

#include "individual.hpp"

namespace {

using AdjacencySignature = std::vector<std::uint64_t>;

std::uint64_t encode_undirected_edge(int first_node, int second_node) {
    if (first_node > second_node) {
        std::swap(first_node, second_node);
    }
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(first_node)) << 32)
        | static_cast<std::uint32_t>(second_node);
}

AdjacencySignature build_depot_aware_signature(const Individual& individual) {
    AdjacencySignature signature;
    std::size_t edge_count = 0;
    for (int route_index = 0; route_index < individual.num_routes; ++route_index) {
        if (individual.num_nodes_per_route[route_index] > 1) {
            edge_count += static_cast<std::size_t>(individual.num_nodes_per_route[route_index] - 1);
        }
    }
    signature.reserve(edge_count);

    // Route boundary edges are included and multiplicity is preserved.
    for (int route_index = 0; route_index < individual.num_routes; ++route_index) {
        for (int node_index = 1; node_index < individual.num_nodes_per_route[route_index]; ++node_index) {
            signature.push_back(encode_undirected_edge(
                individual.routes[route_index][node_index - 1],
                individual.routes[route_index][node_index]));
        }
    }
    std::sort(signature.begin(), signature.end());
    return signature;
}

double multiset_jaccard_similarity(
    const AdjacencySignature& first,
    const AdjacencySignature& second) {
    if (first.empty() && second.empty()) {
        return 1.0;
    }

    std::size_t intersection_size = 0;
    std::size_t union_size = 0;
    std::size_t first_index = 0;
    std::size_t second_index = 0;
    while (first_index < first.size() && second_index < second.size()) {
        if (first[first_index] == second[second_index]) {
            ++intersection_size;
            ++union_size;
            ++first_index;
            ++second_index;
        } else if (first[first_index] < second[second_index]) {
            ++union_size;
            ++first_index;
        } else {
            ++union_size;
            ++second_index;
        }
    }
    union_size += first.size() - first_index;
    union_size += second.size() - second_index;
    return static_cast<double>(intersection_size) / static_cast<double>(union_size);
}

}  // namespace

std::shared_ptr<Individual> MaReproduction::best_by_upper_cost(
    const std::vector<std::shared_ptr<Individual>>& population) {
    if (population.empty()) {
        return nullptr;
    }
    return *std::min_element(
        population.begin(),
        population.end(),
        [](const std::shared_ptr<Individual>& first, const std::shared_ptr<Individual>& second) {
            return first->upper_cost < second->upper_cost;
        });
}

std::shared_ptr<Individual> MaReproduction::best_by_lower_cost(
    const std::vector<std::shared_ptr<Individual>>& population) {
    if (population.empty()) {
        return nullptr;
    }
    return *std::min_element(
        population.begin(),
        population.end(),
        [](const std::shared_ptr<Individual>& first, const std::shared_ptr<Individual>& second) {
            return first->lower_cost < second->lower_cost;
        });
}

double MaReproduction::adjacency_distance(
    const ParentCandidate& first,
    const ParentCandidate& second) {
    return 1.0 - multiset_jaccard_similarity(
        first.adjacency_signature,
        second.adjacency_signature);
}

ParentCandidate MaReproduction::make_parent_candidate(const Individual& individual) {
    ParentCandidate candidate;
    candidate.chromosome = individual.get_chromosome();
    candidate.upper_cost = individual.upper_cost;
    candidate.adjacency_signature = build_depot_aware_signature(individual);
    candidate.source = &individual;
    return candidate;
}

std::vector<ParentCandidate> MaReproduction::build_parent_pool(
    const std::vector<std::shared_ptr<Individual>>& ranked_upper_solutions,
    const std::size_t desired_pool_size) {
    if (ranked_upper_solutions.empty() || desired_pool_size == 0) {
        return {};
    }

    const std::size_t candidate_limit = std::min(
        ranked_upper_solutions.size(),
        desired_pool_size * 3);
    std::set<AdjacencySignature> seen_signatures;
    std::vector<ParentCandidate> candidates;
    candidates.reserve(candidate_limit);

    for (const auto& solution : ranked_upper_solutions) {
        ParentCandidate candidate = make_parent_candidate(*solution);
        if (seen_signatures.insert(candidate.adjacency_signature).second) {
            candidates.push_back(std::move(candidate));
            if (candidates.size() == candidate_limit) {
                break;
            }
        }
    }

    const std::size_t pool_size = std::min(desired_pool_size, candidates.size());
    candidates.resize(pool_size);
    return candidates;
}

std::vector<int> MaReproduction::make_random_immigrant(
    const std::vector<int>& customers,
    std::mt19937& random_engine) {
    std::vector<int> immigrant(customers);
    std::shuffle(immigrant.begin(), immigrant.end(), random_engine);
    return immigrant;
}

void MaReproduction::partially_matched_crossover(
    std::vector<int>& first_parent,
    std::vector<int>& second_parent,
    std::mt19937& random_engine,
    ReproductionWorkspace& workspace) {
    const int chromosome_size = static_cast<int>(first_parent.size());
    std::uniform_int_distribution<> distribution(0, chromosome_size - 1);

    int first_cut = distribution(random_engine);
    int second_cut = distribution(random_engine);
    if (first_cut > second_cut) {
        std::swap(first_cut, second_cut);
    }

    auto& first_child = workspace.first_child;
    auto& second_child = workspace.second_child;
    first_child.clear();
    second_child.clear();
    first_child.reserve(chromosome_size);
    second_child.reserve(chromosome_size);
    first_child.insert(
        first_child.end(),
        first_parent.begin() + first_cut,
        first_parent.begin() + second_cut);
    second_child.insert(
        second_child.end(),
        second_parent.begin() + first_cut,
        second_parent.begin() + second_cut);

    const int max_gene = std::max(
        *std::max_element(first_parent.begin(), first_parent.end()),
        *std::max_element(second_parent.begin(), second_parent.end()));
    workspace.first_mapping.assign(static_cast<std::size_t>(max_gene + 1), -1);
    workspace.second_mapping.assign(static_cast<std::size_t>(max_gene + 1), -1);
    auto& first_mapping = workspace.first_mapping;
    auto& second_mapping = workspace.second_mapping;
    for (int i = 0; i < second_cut - first_cut; ++i) {
        first_mapping[second_child[i]] = first_child[i];
        second_mapping[first_child[i]] = second_child[i];
    }

    auto resolve_gene = [](int gene, const std::vector<int>& mapping) {
        while (mapping[gene] != -1) {
            gene = mapping[gene];
        }
        return gene;
    };

    for (int i = 0; i < chromosome_size; ++i) {
        if (i < first_cut || i >= second_cut) {
            const int first_gene = resolve_gene(first_parent[i], first_mapping);
            const int second_gene = resolve_gene(second_parent[i], second_mapping);
            first_child.push_back(second_gene);
            second_child.push_back(first_gene);
        }
    }

    first_parent.swap(first_child);
    second_parent.swap(second_child);
}

void MaReproduction::mutate_by_index_shuffle(
    std::vector<int>& chromosome,
    double mutation_probability,
    std::mt19937& random_engine) {
    const int chromosome_size = static_cast<int>(chromosome.size());
    std::uniform_real_distribution<double> probability_distribution(0.0, 1.0);
    std::uniform_int_distribution<int> swap_distribution(0, chromosome_size - 2);

    for (int i = 0; i < chromosome_size; ++i) {
        if (probability_distribution(random_engine) < mutation_probability) {
            int swap_index = swap_distribution(random_engine);
            if (swap_index >= i) {
                ++swap_index;
            }
            std::swap(chromosome[i], chromosome[swap_index]);
        }
    }
}

std::vector<std::vector<int>> MaReproduction::create_offspring(
    const std::vector<ParentCandidate>& parent_pool,
    const Individual* verified_best,
    bool has_verified_best,
    const std::vector<int>& customers,
    int offspring_count,
    int tournament_size,
    double mutation_probability,
    double gene_mutation_probability,
    double verified_upper_ratio,
    double pure_immigrant_ratio,
    std::mt19937& random_engine,
    std::uniform_real_distribution<double>& probability_distribution,
    ReproductionWorkspace& workspace,
    std::vector<int>* parent_use_counts) {
    std::vector<std::vector<int>> offspring;
    offspring.reserve(static_cast<std::size_t>(offspring_count));
    if (parent_use_counts != nullptr) {
        parent_use_counts->assign(parent_pool.size(), 0);
    }

    const int verified_upper_count = has_verified_best
        ? static_cast<int>(std::lround(offspring_count * verified_upper_ratio))
        : 0;
    const int pure_immigrant_count = static_cast<int>(
        std::lround(offspring_count * pure_immigrant_ratio));
    const int upper_parent_offspring_count = std::max(
        0,
        offspring_count - verified_upper_count - pure_immigrant_count);

    auto append_child = [&](std::vector<int>& child, int phase_target) {
        if (static_cast<int>(offspring.size()) < phase_target) {
            offspring.push_back(std::move(child));
        }
    };

    auto select_upper_parent_index = [&]() {
        const int actual_tournament_size = std::max(
            1,
            std::min(tournament_size, static_cast<int>(parent_pool.size())));
        std::uniform_int_distribution<std::size_t> distribution(0, parent_pool.size() - 1);
        std::size_t best_index = distribution(random_engine);
        for (int i = 1; i < actual_tournament_size; ++i) {
            const std::size_t challenger_index = distribution(random_engine);
            if (parent_pool[challenger_index].upper_cost < parent_pool[best_index].upper_cost) {
                best_index = challenger_index;
            }
        }
        return best_index;
    };

    auto select_diverse_mate_index = [&](std::size_t anchor_index) {
        if (parent_pool.size() <= 1) {
            return anchor_index;
        }

        auto& candidate_indices = workspace.candidate_indices;
        candidate_indices.clear();
        candidate_indices.reserve(parent_pool.size() - 1);
        for (std::size_t i = 0; i < parent_pool.size(); ++i) {
            if (i != anchor_index) {
                candidate_indices.push_back(i);
            }
        }
        std::shuffle(candidate_indices.begin(), candidate_indices.end(), random_engine);
        const std::size_t sample_size = std::min<std::size_t>(8, candidate_indices.size());
        candidate_indices.resize(sample_size);

        std::size_t best_index = candidate_indices.front();
        double best_distance = -1.0;
        for (std::size_t candidate_index : candidate_indices) {
            const double distance = adjacency_distance(
                parent_pool[anchor_index],
                parent_pool[candidate_index]);
            if (distance > best_distance + 1e-12
                || (std::fabs(distance - best_distance) <= 1e-12
                    && parent_pool[candidate_index].upper_cost < parent_pool[best_index].upper_cost)) {
                best_distance = distance;
                best_index = candidate_index;
            }
        }
        return best_index;
    };

    while (static_cast<int>(offspring.size()) < upper_parent_offspring_count) {
        const std::size_t first_parent_index = select_upper_parent_index();
        if (parent_use_counts != nullptr) {
            ++(*parent_use_counts)[first_parent_index];
        }
        std::vector<int> first_child = parent_pool[first_parent_index].chromosome;
        std::vector<int> second_child;
        if (parent_pool.size() == 1) {
            second_child = make_random_immigrant(customers, random_engine);
        } else {
            const std::size_t second_parent_index = select_diverse_mate_index(first_parent_index);
            if (parent_use_counts != nullptr) {
                ++(*parent_use_counts)[second_parent_index];
            }
            second_child = parent_pool[second_parent_index].chromosome;
        }
        partially_matched_crossover(
            first_child,
            second_child,
            random_engine,
            workspace);
        append_child(first_child, upper_parent_offspring_count);
        append_child(second_child, upper_parent_offspring_count);
    }

    const int verified_phase_target = upper_parent_offspring_count + verified_upper_count;
    if (has_verified_best) {
        const std::vector<int> verified_chromosome = verified_best->get_chromosome();
        auto& mate_indices = workspace.mate_indices;
        mate_indices.clear();
        mate_indices.reserve(parent_pool.size());
        for (std::size_t i = 0; i < parent_pool.size(); ++i) {
            if (parent_pool[i].chromosome != verified_chromosome) {
                mate_indices.push_back(i);
            }
        }
        if (mate_indices.empty()) {
            for (std::size_t i = 0; i < parent_pool.size(); ++i) {
                mate_indices.push_back(i);
            }
        }
        std::uniform_int_distribution<std::size_t> select_mate(
            0,
            mate_indices.size() - 1);
        while (static_cast<int>(offspring.size()) < verified_phase_target) {
            const std::size_t mate_index =
                mate_indices[select_mate(random_engine)];
            if (parent_use_counts != nullptr) {
                ++(*parent_use_counts)[mate_index];
            }
            std::vector<int> first_child = verified_chromosome;
            std::vector<int> second_child =
                parent_pool[mate_index].chromosome;
            partially_matched_crossover(
                first_child,
                second_child,
                random_engine,
                workspace);
            append_child(first_child, verified_phase_target);
            append_child(second_child, verified_phase_target);
        }
    }

    while (static_cast<int>(offspring.size()) < offspring_count) {
        offspring.push_back(make_random_immigrant(customers, random_engine));
    }

    for (auto& chromosome : offspring) {
        if (probability_distribution(random_engine) < mutation_probability) {
            mutate_by_index_shuffle(
                chromosome,
                gene_mutation_probability,
                random_engine);
        }
    }
    return offspring;
}
