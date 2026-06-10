# Algorithms

## Implemented Methods

- `Lahc`: bilevel late acceptance hill climbing using `LeaderSingle` for upper-level neighbourhood exploration and `Follower` for lower-level charging decisions.
- `TwoStageAlns`: upper-level adaptive large neighbourhood search with random, geographic, and history-based destroy methods, followed by lower-level charging refinement.

## Stopping Criteria

### Max Evaluations

The IEEE WCCI 2020 benchmark uses:

```text
Max Evals = 25,000 * pz
```

where `pz` is the number of depots, customers, and charging stations. A complete evaluation has time complexity `O(n^2)`. Accessing an arc weight contributes a fractional evaluation budget of `1 / pz`.

### Max Time

The max-time setting follows:

```text
Max Time = v * (|Vc| + |Vf|) / 100 hours
```

where `v` is set to:

- `1` for E22-E101 instances;
- `2` for X143-X916 instances;
- `3` for X1001 instances.

## WCCI2020 EVRP Benchmark Context

| Method   | Stop Criteria | Language/Tool | Code Available |
|----------|---------------|---------------|----------------|
| MILP     | Max Evals     | Gurobi        | No             |
| VNS      | Max Evals     | C++           | Yes            |
| SA       | Max Evals     | C++           | No             |
| GA       | Max Evals     | C++           | Yes            |
| HHASA-TS | Max Evals     | MATLAB        | Yes            |
| BACO     | Max Time      | C++           | Yes            |
| CBACO-I  | Max Time      | C++           | Yes            |
| TAMLS    | Max Time      | C++           | No             |
| CBMA     | Max Time      | C++           | Yes            |
| B-LAHC   | Both          | C++           | Yes            |

References and code links:

- MILP: [technical report](https://mavrovouniotis.github.io/EVRPcompetition2020/)
- VNS: [paper](https://arxiv.org/abs/2511.09570), [original code](https://github.com/wolledav/VNS-EVRP-2020), [modified code](https://github.com/KingQino/VNS-EVRP-2020)
- GA: [paper](https://www.researchgate.net/profile/Cong-Dao-Tran/publication/360604653_A_greedy_search_based_evolutionary_algorithm_for_electric_vehicle_routing_problem/links/641d203a315dfb4ccea54309/A-greedy-search-based-evolutionary-algorithm-for-electric-vehicle-routing-problem.pdf), [original code](https://github.com/NeiH4207/EVRP), [modified code](https://github.com/KingQino/HMAGS-Hien)
- HHASA-TS: [paper](https://www.sciencedirect.com/science/article/pii/S0957417424010637), [original code](https://github.com/erickre12/HHASARL.git), [modified code](https://github.com/KingQino/HHASARL/tree/main)
- BACO: [paper](https://ieeexplore.ieee.org/document/9409782), [original code](https://github.com/Flyki/CEVRP), [modified code](https://github.com/KingQino/CBACO-Yahui/)
- CBACO-I: [paper](https://ieeexplore.ieee.org/document/9684529), [original code](https://github.com/Flyki/CEVRP), [modified code](https://github.com/KingQino/CBACO-Yahui/)
- TAMLS: [paper](https://ieeexplore.ieee.org/document/10382457)
- CBMA: [paper](https://ieeexplore.ieee.org/abstract/document/10611804), [code](https://github.com/KingQino/CEVRP-with-Adaptive-Selection)

The modified reference code used for comparisons applies uniform stopping criteria and enhanced statistical logging.
