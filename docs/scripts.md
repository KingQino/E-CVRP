# Scripts and Experiments

## Benchmark Scripts

Benchmark scripts live under `stats/[Algorithm]/` when experiment outputs are generated.

Common entry points:

- Max-evals benchmark: `./stats/[Algorithm]/benchmark_max_evals.sh`
- Max-time benchmark: `./stats/[Algorithm]/benchmark_max_time.sh`
- Aggregate statistics: `./stats/[Algorithm]/update_all_stats.sh`
- Objective extraction: `./stats/[Algorithm]/objective.sh`

Supported algorithm folders:

- `LAHC`
- `MA`
- `TWOSTAGEALNS`

## Per-Run Logs

When `-log 1` is enabled, the solver writes per-run files under:

```text
stats/[Algorithm]/[Instance]/[Seed]/
```

Expected files:

- `evols.[Instance].csv`: evolution trace;
- `solution.[Instance].txt`: final objective and route output;
- `profile.[Instance].txt`: lightweight timing profile (LAHC, TwoStageAlns).

MA additionally writes:

- `local-search-operators.tsv`: per-generation calls, accepts, evaluations, upper gain and gamma crossings of the eight RVND operators;
- `local-search-allocation.tsv`: 10-generation totals per local-search action (non-static policies only);
- `elite-local-search.tsv`: 10-generation Elite Unrestricted totals (`online` policy only);
- `search-budget.tsv`: 10-generation split of evaluations into ordinary local search, Elite Unrestricted, follower and other work;
- `run-configuration.tsv`: gamma, local-search settings, unrestricted credit rho, depth profile and learner action-cost weight of the run.

Multi-trial summaries are written as:

```text
stats/[Algorithm]/[Instance]/stats.[Instance].evrp
```

## Experiment Data

Experimental data generated for the study are publicly available via the [Globus link](https://app.globus.org/file-manager?origin_id=8581615c-a264-4327-a9d7-28b8c027f005&origin_path=%2F).

Globus supports institutional single sign-on for many universities and research institutions in the United States, United Kingdom, and Europe. Users without institutional access can register a free Globus ID at [globus.org](https://www.globus.org) and access the public dataset.

The experiment set includes:

- correlation vs. misalignment experiments validating the surrogate objective;
- comparisons with state-of-the-art algorithms under max-evals and max-time criteria;
- ablation studies on key components;
- hyperparameter sensitivity analysis.
