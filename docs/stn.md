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
| BMA (`ma`) | the best upper-level individual of the generation (after local search) | once per generation, plus the initial population |

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

`scripts/stn/check_stn_logs.py` reports hash collisions, f_U consistency for each signature, and signatures shared between algorithms:

```shell
python3 scripts/stn/check_stn_logs.py --stats stats --instance X-n214-k11 --alg LAHC --alg MA
```

## Experiment Settings

| Algorithm | Arguments |
|---|---|
| B-LAHC | `-alg lahc -lahc_his_len 5723 -lahc_max_attempts 60 -lahc_noise_lb 0.95 -lahc_noise_ub 1.05 -low_margin 1.05` |
| BMA | `-alg ma -low_margin 1.05 -ma_ls_policy static -ma_static_unrestricted 1` |

Both use `-stp 0 -mth 1 -log 1 -stn 1` (max-evals, seeds 1-10). On the cluster, add `-stn 1` to the arguments in `scripts/hpc/experiments.tsv`.
