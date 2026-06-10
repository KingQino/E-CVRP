# E-CVRP

This repository contains C++ implementations for solving the Electric Capacitated Vehicle Routing Problem (E-CVRP), including bilevel LAHC and a two-stage upper-level ALNS with lower-level charging refinement.

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

## Documentation

- [Setup and Usage](docs/setup.md): build details, run examples, parameters, and instance names.
- [Architecture](docs/architecture.md): data structures, leader/follower interaction, and solver layers.
- [Algorithms](docs/algorithms.md): implemented methods, stopping criteria, and benchmark context.
- [Scripts and Experiments](docs/scripts.md): benchmark scripts, log layout, and experiment data notes.

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
  year={2026}
}
```

## Licence

[![Licence](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](https://yinghao.mit-license.org/license.html)

- [MIT licence](https://yinghao.mit-license.org/license.html)
- Copyright(c) 2025 Yinghao Qin
