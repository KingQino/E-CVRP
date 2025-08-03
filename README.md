# E-CVRP

- This project contains a specific algorithm implementation for solving Electric Capacitated Vehicle Routing Problem (E-CVRP).

- Our algorithm B-LAHC is evaluated on a widely used benchmark suite developed for the IEEE WCCI-2020 Competition on Evolutionary Computation for the EVRP. [Link](https://mavrovouniotis.github.io/EVRPcompetition2020/)

- Paper:

  > Our paper.

- **Table of Contents**
  
  1. [Usage](#Usage)
  2. [Architecture](#Architecture)
  3. [Algorithms](#Algorithms)
  4. [Experiments](#Experiments)
  5. [Script](#Script)
  6. [Licence](#Licence)



## Usage

1. Requirement

   - Compiler: GCC ≥ 7 or Clang ≥ 5 (C++17 support required)
   - Build tool: CMake ≥ 3.15
   - Dependency: OpenMP (for multithreading)
   - Optional: GoogleTest (for Debug mode testing)

2. Compile

   ```shell
   mkdir build && cd build
   cmake -DCMAKE_BUILD_TYPE=Release ..
   make
   ```

3. Run

   ```shell
   ./Run -alg lahc -ins E-n22-k4.evrp -log 1 -stp 0 -mth 1 -his_len 5723 -max_attempts 60 -low_margin 1.01 -noise_lb 0.95 -noise_ub 1.05
   ```

   Parameters Instruction

   ```c++
   /* ===============================================================================*/
   Usage: ./Run [options]
   Options:
     -alg [enum]            : Algorithm name (e.g., Lahc)
     -ins [filename]        : Problem instance filename
     -log [0|1]             : Enable logging (default: 0)
     -stp [0|1]             : Stopping criteria, 0: max-evals, 1: max-time (default: 0)
     -mth [0|1]             : Enable multi-threading (default: 1)
     -exp [0|1]             : Activate experimental mode (default: 0)
     -seed [int]            : Random seed (default: 0)
     -his_len [int]         : LAHC history length (default: 5000)
     -max_attempts [int]    : max attempts (default: 25)
     -low_margin [double]   : margin is the best upper cost for lower optimisation, >= 1.0 (default: 1.10)
     -noise_lb [double]     : lower bound of noise range for history list value (default: 0.95)
     -noise_ub [double]     : upper bound of noise range for history list value (default: 1.05)
   /* ===============================================================================*/
   ```

   Instance names `-ins`

   ```
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



## Architecture

- Data Handling Layer
  - `Case`: Stores **raw E-CVRP instance data**
  - `Preprocessor`: Computes **preprocessed data**, used in the optimisation process
  - `Individual`: Stores solutions (routes for vehicles)
  - `PartialSolution`: Stores partial solutions. This structure is specifically designed to coordinate the leader and follower, to avoid pass the whole solution (too expensive)
  - `CommandLine`: Parses **runtime parameters** (e.g., `-alg lahc`, `-ins E-n22-k4.evrp`, `-log 1`, `-his_len 5000`).
  - `Parameters`: parse arguments from command line to the parameters.
- Optimisation Layer

  - `Initializer`: Solution initialisation methods, split method for splitting the chromosome into multiple routes

  - `Leader`: Optimizes **route structures**, ensuring **vehicle capacity feasibility** but **ignores charging decisions**
  - `Follower`: Makes **charging decisions**, ensuring **route feasibility for electric vehicles** while maintaining the given path.
- Heuristic Algorithm Layer

  - `HeuristicInterface`: Abstract class defining the structure for **heuristic algorithms**, it holds the unified __stop critria__ for the algorithms. 
  - `Lahc`: Bilevel late acceptance hill climbing
- Statistical Analysis Layer

  - `StatsInterface`: Provides methods for **tracking algorithm performance**, recording iteration details, and analyzing convergence.

---

- How do `Leader` and `Follower` interact?

  `Individual` and `PartialSolution` are two different information carriers, which pass the solution to <u>Optimisation Layers</u>.

  > Interact with
  >
  > - `Initializer` => using solution constructor
  >
  > - `Leader` => using function `load_solution` and `export_solution`, and all upper-level optimisation are compeleted in the `leader`
  >
  > - `Follower` => using function `load_solution` and `export_solution`, and all upper-level optimisation are compeleted in the `follower`
  >   - lower_cost

- how to make moves (<u>upper-level optimisation</u>) and update the current solution?

  - we don't need to load a `Individual` into the `leader` every time before making a move => too redundant
  - Instead, in the entire optimisation process, we load it once at the start of the optimisation, the `leader` then directly makes moves and updates the solution in place

- how to make the <u>lower-level optimisation</u>?

  - In `Follower`, load `solution`/`Individual` to its data structure, then optimising. In the end, the updated cost is exported to the `Individual`.
  - To output the final result, we just need to make the lower-level optimisation to the `Solution` upper-level solution again, and then we can get the exactly same solution. 



## Algorithms

### Stop Criteria

- Max Evals (suggested in the IEEE WCCI 2020 benchmark): 
  
  ​								![Max Evals](https://latex.codecogs.com/svg.image?\text{Max%20Evals}%20=%2025{,}000%20\times%20pz)
  
  where _pz_ is the number of depot, customers and charging stations. Each complete evaluation has a time complexity of O(n<sup>2</sup>). Notably, each access to an arc weight _d<sub>ij</sub>_ consumes a fraction _1/pz_ of the budget, meaning that every neighbourhood move contributes to the overall evaluation count.

- Max Time (introduced by Ya-Hui et al.):
  
  ​								![Max Time Formula](https://latex.codecogs.com/svg.image?\text{Max%20Time}%20=%20v%20\cdot%20\frac{|V_c|%20+%20|V_f|}{100}%20\quad%20\text{(hours)})
  
  where the parameter _v_ is set to 1, 2, and 3 for instance groups E22–E101, X143–X916, and X1001 respectively.

### Overview of Methods on the WCCI-2020 EVRP Benchmark

| Methods  | Stop Criteria | Language/Tool | Code Available |
| -------- | ------------- | ------------- | -------------- |
| MILP     | Max Evals     | Gurobi        | --             |
| VNS      | Max Evals     | C++           | Yes            |
| SA       | Max Evals     | C++           | No             |
| GA       | Max Evals     | C++           | Yes            |
| HHASA-TS | Max Evals     | MATLAB        | Yes            |
| BACO     | Max Time      | C++           | Yes            |
| CBACO-I  | Max Time      | C++           | Yes            |
| TAMLS    | Max Time      | C++           | No             |
| CBMA     | Max Time      | C++           | Yes            |
| B-LAHC   | Both          | C++           | Yes            |

- MILP: [technical report](https://mavrovouniotis.github.io/EVRPcompetition2020/)
- VNS: [original code](https://github.com/wolledav/VNS-EVRP-2020), [modified code](https://github.com/KingQino/VNS-EVRP-2020/tree/broker)
- GA: [paper](https://www.researchgate.net/profile/Cong-Dao-Tran/publication/360604653_A_greedy_search_based_evolutionary_algorithm_for_electric_vehicle_routing_problem/links/641d203a315dfb4ccea54309/A-greedy-search-based-evolutionary-algorithm-for-electric-vehicle-routing-problem.pdf), [original code](https://github.com/NeiH4207/EVRP), [modified code](https://github.com/KingQino/HMAGS-Hien)
- HHASA-TS: [paper](https://www.sciencedirect.com/science/article/pii/S0957417424010637), [code](https://github.com/erickre12/HHASARL.git)
- BACO: [paper](https://ieeexplore.ieee.org/document/9409782), [original code](https://github.com/Flyki/CEVRP), [modified code](https://github.com/KingQino/CBACO-Yahui/)
- CBACO-I: [paper](https://ieeexplore.ieee.org/document/9684529), [original code](https://github.com/Flyki/CEVRP), [modified code](https://github.com/KingQino/CBACO-Yahui/)
- TAMLS: [paper](https://ieeexplore.ieee.org/document/10382457)
- CBMA: [paper](https://ieeexplore.ieee.org/abstract/document/10611804), [code](https://github.com/KingQino/CEVRP-with-Adaptive-Selection)

*The modified code* includes my modifications on the original version, including (1) uniform stopping criteria, and (2) enhanced statistical logging.



## Experiments

- Pearson coefficient



## Script

- Experiemental results collection - `./stats/[Algorithm]/objective.sh`

  ```sh
  #!/bin/bash
  
  output_file="a.txt"
  > "$output_file" # Clear or create the output file
  
  # List of directories and their specific stats files in the desired order
  declare -A stats_files=(
      [E-n22-k4]="stats.E-n22-k4.evrp"
      [E-n23-k3]="stats.E-n23-k3.evrp"
      [E-n30-k3]="stats.E-n30-k3.evrp"
      [E-n33-k4]="stats.E-n33-k4.evrp"
      [E-n51-k5]="stats.E-n51-k5.evrp"
      [E-n76-k7]="stats.E-n76-k7.evrp"
      [E-n101-k8]="stats.E-n101-k8.evrp"
      [X-n143-k7]="stats.X-n143-k7.evrp"
      [X-n214-k11]="stats.X-n214-k11.evrp"
      [X-n351-k40]="stats.X-n351-k40.evrp"
      [X-n459-k26]="stats.X-n459-k26.evrp"
      [X-n573-k30]="stats.X-n573-k30.evrp"
      [X-n685-k75]="stats.X-n685-k75.evrp"
      [X-n749-k98]="stats.X-n749-k98.evrp"
      [X-n819-k171]="stats.X-n819-k171.evrp"
      [X-n916-k207]="stats.X-n916-k207.evrp"
      [X-n1001-k43]="stats.X-n1001-k43.evrp"
  )
  
  # Process files in the given sequence
  for dir in E-n22-k4 E-n23-k3 E-n30-k3 E-n33-k4 E-n51-k5 E-n76-k7 E-n101-k8 \
             X-n143-k7 X-n214-k11 X-n351-k40 X-n459-k26 X-n573-k30 X-n685-k75 \
             X-n749-k98 X-n819-k171 X-n916-k207 X-n1001-k43; do
      file_path="$dir/${stats_files[$dir]}"
      if [ -f "$file_path" ]; then
          tail -n 3 "$file_path" >> "$output_file"
      else
          echo "File not found: $file_path" >&2
      fi
  done
  
  # Process the output file to ensure results follow the sequence
  awk '
  /Mean/ {
      mean_value = $2;
      std_dev_value = $NF;
  }
  /Min:/ {
      min_value = $2;
      print min_value;
      print mean_value;
      print std_dev_value;
  }' "$output_file"
  
  output_file="a.txt"
  
  awk '
  /^Mean/ {
      mean = $2
      std = $NF   # Std Dev 
  }
  /^Min/ {
      min = $3
      print min
      print mean
      print std
  }
  ' "$output_file"
  
  rm -f "$output_file"
  ```

  

## Licence

[![Licence](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://badges.mit-license.org)

- **[MIT licence](http://opensource.org/licenses/mit-license.php)**
- [MIT license application](https://github.com/remy/mit-license?tab=readme-ov-file#browse-custom-themes)
- Copyright(c) 2025 Yinghao Qin
