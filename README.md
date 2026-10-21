# E-CVRP

This repository contains C++ implementations for solving the Electric Capacitated Vehicle Routing Problem (E-CVRP), including bilevel late acceptance hill climbing (B-LAHC), a budget-aware bilevel memetic algorithm (BA-BMA), and a two-stage upper-level ALNS with lower-level charging refinement.

## Quick Start

Requirements:

- C++17 compiler: GCC >= 7 or Clang >= 5
- CMake >= 3.15
- OpenMP
- Optional: GoogleTest for debug-mode tests

Build:

```shell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target Run
```

Run:

```shell
./build/Run -alg lahc -ins E-n22-k4.evrp -log 1 -stp 0 -mth 1
```

For all command-line options:

```shell
./build/Run -h
```

## Running on HPC

To run all three algorithms on the 133 instances on a Slurm cluster (10 runs per instance, max-evals), use the scripts in [`scripts/hpc/`](scripts/hpc):

```shell
# on the cluster, from the directory that should hold the experiment (results are written next to source/)
git clone -b broker git@github.com:KingQino/E-CVRP.git source
./source/scripts/hpc/run-all.sh              # build, prepare and submit one Slurm array per algorithm
./source/scripts/hpc/check-progress.sh       # finished instances; add --resubmit to rerun missing ones
./source/scripts/hpc/summarize-results.sh    # result tables in results/

# on your computer
ECVRP_REMOTE=<user>@<login-node> ECVRP_REMOTE_ROOT=<experiment directory on the cluster> ./scripts/hpc/download-results.sh --summary
```

See [HPC Experiments](docs/hpc.md) for details.

## Documentation

- [Setup and Usage](docs/setup.md): build details, run examples, parameters, and instance names.
- [Architecture](docs/architecture.md): data structures, leader/follower interaction, and solver layers.
- [Algorithms](docs/algorithms.md): implemented methods, stopping criteria, and benchmark context.
- [Scripts and Experiments](docs/scripts.md): benchmark scripts, log layout, and experiment data notes.
- [HPC Experiments](docs/hpc.md): cloning on a cluster, submitting Slurm array jobs, and collecting results.
- [Search Trajectory Networks](docs/stn.md): trajectory logging for STNs-v2 (`stn-logging` branch).

## Data

The repository includes WCCI2020, WCCI2020 extension, and EVRPTW-derived instances under `data/`.

## Citation

```bibtex
@article{qin2026bilevel,
  title={Bilevel Late Acceptance Hill Climbing for the Electric Capacitated Vehicle Routing Problem},
  author={Qin, Yinghao and Bazargani, Mosab and Burke, Edmund K and Coello, Carlos A Coello and Song, Zhongmin and Chen, Jun},
  journal={arXiv preprint arXiv:2604.13013},
  year={2026}
}

@article{qin2026instance,
  title={Instance-Aware Parameter Configuration in Bilevel Late Acceptance Hill Climbing for the Electric Capacitated Vehicle Routing Problem},
  author={Qin, Yinghao and Wang, Xinwei and Bazargani, Mosab and Chen, Jun},
  journal={arXiv preprint arXiv:2605.00572},
  note={Accepted at the IEEE Congress on Evolutionary Computation (CEC)},
  year={2026}
}

@article{qin2026budget,
  title={A Budget-Aware Bilevel Memetic Algorithm for the Electric Capacitated Vehicle Routing Problem},
  author={Qin, Yinghao and Zhang, Fangfang and Song, Zhongmin and Kim, Eun-Seok and Chen, Jun},
  note={Under review at IEEE Transactions on Evolutionary Computation},
  year={2026}
}
```

## Licence

[![Licence](https://img.shields.io/badge/license-MIT-blue.svg?style=flat-square)](LICENSE)

- [MIT licence](LICENSE)
- Copyright(c) 2025 Yinghao Qin
