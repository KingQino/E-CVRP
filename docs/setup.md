# Setup and Usage

## Requirements

- Compiler: GCC >= 7 or Clang >= 5 with C++17 support
- Build tool: CMake >= 3.15
- Dependency: OpenMP
- Optional: GoogleTest for Debug builds

On macOS with AppleClang, install `libomp` if OpenMP is not already available:

```shell
brew install libomp
```

## Build

Release build:

```shell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target Run
```

Debug build:

```shell
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug --target Run
```

If GoogleTest is available, the Debug build also defines a `Tests` target:

```shell
cmake --build build-debug --target Tests
ctest --test-dir build-debug
```

## Run

TwoStageAlns:

```shell
./build/Run -alg twostagealns -ins E-n22-k4.evrp -log 1 -stp 0 -mth 1
```

LAHC:

```shell
./build/Run -alg lahc -ins E-n22-k4.evrp -log 1 -stp 0 -mth 1 -his_len 5723 -max_attempts 60
```

Show help:

```shell
./build/Run -h
```

## Parameters

```text
Usage: ./Run [options]
Options:
  -alg [enum]                  : Algorithm name (Lahc or TwoStageAlns)
  -ins [filename]              : Problem instance filename
  -log [0|1]                   : Enable logging (default: 0)
  -stp [0|1]                   : Stopping criteria, 0: max-evals, 1: max-time (default: 0)
  -mth [0|1]                   : Enable multi-threading (default: 1)
  -exp [0|1]                   : Activate experimental mode (default: 0)
  -refine [0|1]                : Enable final follower refinement (default: 1)
  -seed [int]                  : Random seed (default: 0)
  -his_len [int]               : LAHC history length (default: 5723)
  -max_attempts [int]          : LAHC max attempts for LeaderSingle (default: 60)
  -low_margin [double]         : LAHC upper-cost margin to trigger follower optimisation, >= 1.0 (default: 1.05)
  -noise_lb [double]           : LAHC history-noise lower bound (default: 0.95)
  -noise_ub [double]           : LAHC history-noise upper bound (default: 1.05)
  -alns_start_dev [double]     : TwoStageAlns initial record-deviation gap (default: 0.0167)
  -alns_decay [double]         : TwoStageAlns adaptive-weight decay (default: 0.99)
  -alns_score_acc [double]     : TwoStageAlns score for accepted solutions (default: 2.0)
  -alns_score_imp [double]     : TwoStageAlns score for improving solutions (default: 4.0)
  -alns_score_best [double]    : TwoStageAlns score for new best upper solutions (default: 10.0)
  -alns_min_destroy_ratio [double] : TwoStageAlns minimum relative destroy size (default: 0.10)
  -alns_max_destroy_ratio [double] : TwoStageAlns maximum relative destroy size (default: 0.40)
  -alns_min_destroy_abs [int]  : TwoStageAlns minimum absolute destroy size (default: 5)
  -alns_max_destroy_abs [int]  : TwoStageAlns maximum absolute destroy size (default: 150)
  -alns_geo_bias [double]      : TwoStageAlns geo destroy bias (default: 5.0)
  -alns_repair_bias [double]   : TwoStageAlns randomized regret factor (default: 1.5)
```

## WCCI2020 Instance Names

```text
E-n22-k4.evrp
E-n23-k3.evrp
E-n30-k3.evrp
E-n33-k4.evrp
E-n51-k5.evrp
E-n76-k7.evrp
E-n101-k8.evrp
X-n143-k7.evrp
X-n214-k11.evrp
X-n351-k40.evrp
X-n459-k26.evrp
X-n573-k30.evrp
X-n685-k75.evrp
X-n749-k98.evrp
X-n819-k171.evrp
X-n916-k207.evrp
X-n1001-k43.evrp
```

Other supported datasets are organized under `data/WCCI2020-Extension/` and `data/EVRPTW/`.
