//
// Created by Yinghao Qin on 11/10/2026.
//
// Parent-pool construction and offspring generation of the MA.
//

#ifndef MA_REPRODUCTION_HPP
#define MA_REPRODUCTION_HPP

#include <cstdint>
#include <memory>
#include <random>
#include <vector>

class Individual;

// Snapshot of an upper-level solution that can act as a parent.
struct ParentCandidate {
    std::vector<int> chromosome;                     // giant tour without depots
    double upper_cost{};
    std::vector<std::uint64_t> adjacency_signature;  // sorted multiset of depot-aware undirected edges
    const Individual* source{};
};

struct ReproductionWorkspace {
    std::vector<int> first_child;
    std::vector<int> second_child;
    std::vector<int> first_mapping;
    std::vector<int> second_mapping;
    std::vector<std::size_t> candidate_indices;
    std::vector<std::size_t> mate_indices;
};

class MaReproduction {
public:
    static std::shared_ptr<Individual> best_by_upper_cost(const std::vector<std::shared_ptr<Individual>>& population);
    static std::shared_ptr<Individual> best_by_lower_cost(const std::vector<std::shared_ptr<Individual>>& population);

    // Quality-only parent pool: best upper-cost solutions with distinct adjacency signatures.
    static std::vector<ParentCandidate> build_parent_pool(
        const std::vector<std::shared_ptr<Individual>>& ranked_upper_solutions,
        std::size_t desired_pool_size);
    static ParentCandidate make_parent_candidate(const Individual& individual);
    // 1 - multiset Jaccard similarity of the two adjacency signatures.
    static double adjacency_distance(const ParentCandidate& first, const ParentCandidate& second);

    static std::vector<int> make_random_immigrant(const std::vector<int>& customers, std::mt19937& random_engine);
    static void partially_matched_crossover(
        std::vector<int>& first_parent,
        std::vector<int>& second_parent,
        std::mt19937& random_engine,
        ReproductionWorkspace& workspace);
    static void mutate_by_index_shuffle(
        std::vector<int>& chromosome,
        double mutation_probability,
        std::mt19937& random_engine);

    // Offspring (notation of the BA-BMA paper; verified_best = x*, best complete solution so far):
    //   (1 - verified_upper_ratio - pure_immigrant_ratio) P_U x P_U PMX children
    //       (first parent: tournament on f_U; second parent: largest d_J to the first among up to
    //        8 randomly sampled members of P_U, ties broken by f_U)
    // + verified_upper_ratio x* x P_U PMX children (mates drawn at random from P_U; skipped if x* does not exist)
    // + pure_immigrant_ratio random customer permutations;
    // every chromosome is then mutated with probability mutation_probability (shuffle-index mutation).
    static std::vector<std::vector<int>> create_offspring(
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
        std::vector<int>* parent_use_counts = nullptr);
};

#endif //MA_REPRODUCTION_HPP
