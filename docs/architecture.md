# Architecture

The solver is organized around a bilevel view of E-CVRP:

- the upper level builds and improves customer route structures under vehicle-capacity constraints;
- the lower level inserts charging decisions so the route is feasible for electric vehicles.

## Data Handling Layer

- `Case`: stores raw E-CVRP instance data and distance/evaluation accounting.
- `Preprocessor`: computes derived data used by the optimisation process, such as customer/station lists, route capacity, cruise distance, and nearest-station lookups.
- `Individual`: stores a complete solution, including upper-level routes and lower-level cost.
- `PartialSolution`: stores a compact changed part of a solution for leader/follower coordination.
- `CommandLine`: parses runtime parameters such as `-alg`, `-ins`, `-log`, and algorithm-specific options.
- `Parameters`: owns parsed command-line settings and defaults.

## Optimisation Layer

- `Initializer`: constructs initial route sets using split, greedy sequential, Hien-style clustering, or direct encoding methods.
- `Leader`: improves route structures while enforcing vehicle-capacity feasibility and ignoring charging decisions.
- `LeaderSingle`: performs single-operator neighbourhood exploration for LAHC.
- `Follower`: inserts charging stations and evaluates/refines lower-level feasibility for a fixed route structure.

## Heuristic Layer

- `HeuristicInterface`: common heuristic lifecycle and stopping criteria.
- `Lahc`: bilevel late acceptance hill climbing.
- `TwoStageAlns`: two-stage upper-level ALNS followed by lower-level charging refinement.

## Statistics Layer

- `StatsInterface`: records evolution traces, final solutions, profiles, and aggregate statistics.

## Leader and Follower Interaction

`Individual` is the primary solution carrier. `PartialSolution` exists for lower-level updates that only need changed routes instead of a full solution copy.

The typical flow is:

1. `Initializer` creates a route structure and constructs an `Individual`.
2. `Leader` loads the `Individual`, modifies upper-level routes in place, and exports updated route metadata.
3. `Follower` loads the upper-level route structure, inserts charging decisions, and exports `lower_cost`.
4. Final output reruns lower-level optimisation on the best upper-level route structure to materialize the complete route-plus-charging solution.

## Upper-Level Moves

The leader does not reload a fresh `Individual` before every move. The solution is loaded once at the start of a leader search phase, then neighbourhood operators update the route buffers directly.

This keeps upper-level move exploration cheaper because it avoids repeated whole-solution copying.

## Lower-Level Optimisation

The follower keeps reusable route buffers for charging insertion. For a complete solution, it evaluates each route and sums the route-level lower costs. For final refinement, it can enumerate charging-station placements more thoroughly and export the materialized lower-level routes for logging.
