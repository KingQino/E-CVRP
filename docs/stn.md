# Search Trajectory Networks

This branch (`stn-logging`) adds passive trajectory logging for [STNs-v2](https://github.com/gabro8a/STNs-v2), used to compare B-LAHC and BMA. Logging is off by default, and the solver behaves exactly as on `broker` when it is off.

## Logging

| Option | Default | Meaning |
|---|---|---|
| `-stn [0\|1]` | `0` | write the raw trajectory log |
| `-stn_grid [double]` | `0.0001` | observation grid, as a fraction of the max-evals budget (0.01%) |
| `-stn_solutions [0\|1]` | `1` | also write the routes of every logged signature |

The logged solution (the representative) is:

| Algorithm | Representative | Observation points |
|---|---|---|
| B-LAHC (`lahc`) | the current solution | after each restart's initialisation and at the end of every iteration |
| BMA (`ma`), static policy | the best individual of the current generation among those whose local search is complete | after every individual's local search (the random initial population and unsearched offspring are never logged) |
| BMA (`ma`), allocation policies | the best upper-level individual of the generation after local search | once per generation |

At each observation point, the representative is written if a grid point has been reached since the last row. That gives one row at the first observation point at or after each grid point `k * grid * budget`. The last state of the run is always written. Fitness is the upper-level objective f_U for both algorithms.

Files, in `stats/<ALG>/<instance>/<seed>/`:

| File | Columns |
|---|---|
| `stn_raw.<instance>.csv` | `step,distance_calls,evals,fitness,signature`: LAHC iteration or MA generation, exact budget counter, evaluations (`distance_calls / problem_size`), f_U with 6 decimals, signature |
| `stn_solutions.<instance>.csv` | `signature,routes`: canonical routes, written once per signature |
| `stn_meta.<instance>.csv` | problem size, budget in distance calls, grid |

### Signature

The signature identifies the routing only:

1. In each route, remove the depot and the charging stations.
2. Orient each route so that its smaller end customer comes first.
3. Sort the routes.
4. Serialize as customer ids joined by `-`, routes joined by `|` (for example `1-2-3|4-5`).
5. Hash with 64-bit FNV-1a and write 16 lowercase hex characters.

The same routing gives the same signature in both algorithms.

### No effect on the search

The logger reads the representative's stored routes and `upper_cost`. It never calls `Case::get_distance` and never draws random numbers, so the budget and the search are unchanged. To check this:

- `tests/stn_logger_test.cpp` runs B-LAHC and BMA with logging off and on, and compares the final cost and distance-call count;
- the experiment runs compare the `broker` binary with the `stn-logging` binary at `-stn 1`. Solution files, `stats.*` files and the evolution logs (without the time column) must be byte-identical.

## STNs-v2 Input

`scripts/stn/make_stn_input.py` samples the raw logs and writes the input of `create.R`:

```shell
python3 scripts/stn/make_stn_input.py --stats stats --instance X-n214-k11 \
    --alg LAHC=BLAHC --alg MA=BMA --runs 10 --interval 0.001 --out stn-data
```

This writes `stn-data/X-n214-k11/BLAHC_X-n214-k11.csv` and `BMA_X-n214-k11.csv`, with the columns `Run,Fitness,Solution`. For each run, it samples checkpoints `k * interval * budget` for `k = 0, ..., 1/interval` (1,001 rows for 0.1%). Each checkpoint takes the first raw row at or after it, or the last row if the run ended before it. Fitness is rounded to 2 decimals. Use `--collapse` to drop consecutive repeats.

`--interval` can be any multiple of the logging grid (0.1%, 0.5%, 1%, and so on) without rerunning. Algorithm names must not contain `_`, because STNs-v2 takes the name from the text before the first `_`.

STN Web rejects minimisation runs whose fitness increases. Add `--best-so-far` to replace each logged representative by the best one logged so far in the run, and `--collapse` to keep only the improvements. A run with a single row gets that row twice, because `create.R` needs at least one edge per run.

`scripts/stn/stn_v2_to_v1.py <v2 folder> <v1 folder>` converts these files into the edge lists of [STNs v1](https://github.com/gabro8a/STNs) (`Run,Fitness1,Solution1,Fitness2,Solution2`), whose `metrics-alg.R` and `metrics-merged.R` compute network metrics. `scripts/stn/trajectory_metrics.py` reports per-run best-so-far statistics from the raw logs: final f_U, number of improvements, budget used to reach the final best, and f_U at 1%, 10% and 50% of the budget.

`scripts/stn/check_stn_logs.py` reports hash collisions, f_U consistency for each signature, and signatures shared between algorithms:

```shell
python3 scripts/stn/check_stn_logs.py --stats stats --instance X-n214-k11 --alg LAHC --alg MA
```

## Experiment Settings

| Algorithm | Arguments |
|---|---|
| B-LAHC | `-alg lahc -lahc_his_len 5723 -lahc_max_attempts 60 -lahc_noise_lb 0.95 -lahc_noise_ub 1.05 -low_margin 1.05` |
| BMA | `-alg ma -low_margin 1.05 -ma_ls_policy static -ma_static_unrestricted 1` |

Both use `-stp 0 -mth 1 -log 1 -stn 1` (max-evals, seeds 1-10). These rows are in [`scripts/stn/experiments.tsv`](../scripts/stn/experiments.tsv), with `-stn_solutions 0`, because the routes files of B-LAHC reach about 50 MB per run on X-n1001-k43. The instances are in [`scripts/stn/instances.txt`](../scripts/stn/instances.txt).

## Running on HPC

The STN experiments use the [HPC scripts](hpc.md) with the STN manifest and instance list. On the cluster, in an empty experiment directory:

```shell
git clone -b stn-logging git@github.com:KingQino/E-CVRP.git source

export ECVRP_EXPERIMENTS=$PWD/source/scripts/stn/experiments.tsv
export ECVRP_INSTANCES=$PWD/source/scripts/stn/instances.txt

./source/scripts/hpc/setup-all.sh        # build, then prepare BLAHC/ and BMA/ (one array task per instance)
./source/scripts/hpc/submit-all.sh
./source/scripts/hpc/check-progress.sh   # add --resubmit for missing instances
```

Keep the two `export` lines in every new shell, because all the HPC scripts read them. X-n1001-k43 takes about 15 minutes per algorithm for 10 runs on 2 cores, so the default limit of 4 hours per array task is enough.

When all runs are finished, build the STN inputs on the cluster (needs Python 3, for example `module load python`):

```shell
./source/scripts/stn/build-stn-inputs.sh
```

This writes `stn/` in the experiment directory:

| Path | Contents |
|---|---|
| `stn/stn-web/<instance>/` | `BLAHC_<instance>.csv`, `BMA_<instance>.csv`: input of STNs-v2 and STN Web (best-so-far, improvements only) |
| `stn/stn-v1/<instance>/` | the same trajectories as STNs v1 edge lists |
| `stn/trajectory-metrics.csv` | final f_U, improvements, budget to the final best, f_U at 1%, 10% and 50% of the budget |
| `stn/checks.txt` | hash collisions, f_U consistency and shared signatures |

Download with `./scripts/hpc/download-results.sh --summary` on your computer; it includes `stn/` and the raw `stn_raw` and `stn_meta` files, so other intervals can be extracted later with `make_stn_input.py`. Then copy `stn/stn-web/<instance>/` into `data/` of STNs-v2, or `stn/stn-v1/<instance>/` into STNs v1, and run their R scripts.
