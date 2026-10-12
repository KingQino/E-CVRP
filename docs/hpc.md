# Running Experiments on HPC

The scripts in [`scripts/hpc/`](../scripts/hpc) run B-LAHC, BA-BMA and TwoStageAlns on a Slurm cluster:

- all 133 instances (WCCI2020, WCCI2020-Extension, EVRPTW);
- max-evals stopping criterion (`-stp 0`);
- 10 independent runs per instance (seeds 1-10, run in parallel by `-mth 1`);
- one Slurm array per algorithm, one array task per instance.

That is 3 algorithms × 133 instances × 10 runs = 3,990 runs in 399 array tasks.

## Overview

| Step | Where | Command |
|---|---|---|
| 1. Push the code | your computer | `git push origin <branch>` |
| 2. Clone | cluster | `git clone -b <branch> git@github.com:KingQino/E-CVRP.git source` |
| 3. Build and prepare | cluster | `./source/scripts/hpc/setup-all.sh` |
| 4. Submit | cluster | `./source/scripts/hpc/submit-all.sh` |
| 5. Monitor and resubmit | cluster | `./source/scripts/hpc/check-progress.sh [--resubmit]` |
| 6. Summarize | cluster | `./source/scripts/hpc/summarize-results.sh` |
| 7. Download | your computer | `./scripts/hpc/download-results.sh [--summary]` |

Steps 3 and 4 can be done in one go with `./source/scripts/hpc/run-all.sh`.

## 1. Push the Code

The cluster clones from GitHub, so commit and push the branch you want to run first. The examples below use the `broker` branch:

```shell
git checkout broker
git add -A
git commit -m "<message>"
git push origin broker
```

Every experiment directory records the branch, commit and any local changes in `source-commit.txt`, so you can trace each result back to the code that produced it.

## 2. Clone on the Cluster

Log in and go to the directory that should hold the experiment (any empty directory, preferably on scratch storage). Clone the repository into `source/` there. All later commands on the cluster are run from this directory:

```shell
ssh <username>@login.hpc.qmul.ac.uk
cd <experiment directory>
git clone -b broker git@github.com:KingQino/E-CVRP.git source
```

Cloning over SSH needs an SSH key on the cluster that is registered with GitHub. Without one, use `https://github.com/KingQino/E-CVRP.git`.

The scripts treat the directory that contains `source/` as the experiment root, so all results stay next to the clone. After the runs, it looks like this:

```text
./
├── source/                         git clone (scripts in source/scripts/hpc/)
├── LAHC/                           one directory per row of experiments.tsv
│   ├── build/
│   │   ├── Run                     copy of the solver
│   │   ├── instances.txt           instance list of the Slurm array
│   │   ├── script.slurm            Slurm array job
│   │   └── log/                    Slurm output, one file per array task
│   ├── data -> ../source/data
│   ├── stats/LAHC/<instance>/      solver output (see Scripts and Experiments)
│   ├── configuration.tsv           algorithm, arguments, number of instances and runs
│   └── source-commit.txt           branch, commit and local changes used for the binary
├── MA/
├── TWOSTAGEALNS/
├── submitted-jobs.tsv              job ids from submit-all.sh / check-progress.sh
└── results/                        tables from summarize-results.sh
```

## 3. Build and Prepare

Run this on the login node:

```shell
./source/scripts/hpc/setup-all.sh
```

The script checks that every experiment uses a known algorithm and that all instances exist under `data/`. It loads `gcc` and `cmake` and builds `source/build/Run` in Release mode. It then creates one directory per experiment with a copy of the binary and a Slurm array script.

To change something later, such as the time limit, run `setup-one.sh <id>` again. It rewrites that experiment's script without touching its results. Use `ECVRP_SKIP_BUILD=1 ./source/scripts/hpc/setup-all.sh` to prepare without rebuilding.

The experiments are listed in [`scripts/hpc/experiments.tsv`](../scripts/hpc/experiments.tsv). It is tab-separated, and the arguments are added after `-alg <algorithm> -ins <instance> -stp 0 -mth 1 -log 1`:

| id | algorithm | arguments |
|---|---|---|
| `LAHC` | `lahc` | `-lahc_his_len 5723 -lahc_max_attempts 60` |
| `MA` | `ma` | `-low_margin 1.05 -ma_ls_policy online -ma_unrestricted_credit_rho 0.20 -ma_ls_depth d5 -ma_action_cost_weight 0.02` |
| `TWOSTAGEALNS` | `twostagealns` | (defaults) |

Each array task runs:

```shell
./Run -alg <algorithm> -ins <instance> -stp 0 -mth 1 -log 1 <arguments>
```

## 4. Submit

```shell
./source/scripts/hpc/submit-all.sh          # all experiments
./source/scripts/hpc/submit-all.sh MA       # only some of them
```

Job ids are appended to `./submitted-jobs.tsv`. To submit a single script by hand, use `sbatch MA/build/script.slurm`. Command-line options override the `#SBATCH` lines, for example `sbatch --array=0-16 ...` to run the 17 WCCI2020 instances only.

Monitor the jobs with the usual Slurm tools:

```shell
squeue -u $USER
sacct -j <job_id> --format=JobID,State,Elapsed,MaxRSS
tail -f MA/build/log/ecvrp-MA-<job_id>_<index>.out
```

## 5. Check Progress and Resubmit

```shell
./source/scripts/hpc/check-progress.sh
```

```text
experiment            finished  missing array indices
LAHC                   133/133  none
MA                     131/133  15-16
TWOSTAGEALNS           133/133  none
```

An instance counts as finished when all 10 `solution.<instance>.txt` files and `stats.<instance>.evrp` exist. The script also lists Slurm logs that mention a time limit, an out-of-memory kill or a solver error.

Once an experiment has no jobs left in the queue, resubmit only its missing indices:

```shell
./source/scripts/hpc/check-progress.sh --resubmit
```

If tasks hit the time limit, raise it first and then resubmit:

```shell
ECVRP_TIME_LIMIT=12:00:00 ./source/scripts/hpc/setup-one.sh MA
./source/scripts/hpc/check-progress.sh --resubmit MA
```

## 6. Summarize

```shell
./source/scripts/hpc/summarize-results.sh
```

This writes tab-separated tables to `./results/`. It can be run on partial results: unfinished runs are marked `NA`.

| File | Contents |
|---|---|
| `runs-all.tsv` | one row per run: experiment, algorithm, dataset, instance, seed, objective |
| `objective-all.tsv` | per experiment and instance: runs, min, mean, std, max |
| `objective-wide.tsv` | one row per instance with `<id>_min`, `<id>_mean` and `<id>_std` for each experiment (for comparison tables) |
| `ma-search-budget.tsv` | MA only: evaluations spent on ordinary local search, Elite Unrestricted, the follower and other work, summed over the 10 runs |

The objective is the first line of each `solution.<instance>.txt`, which has 5 decimals. The `stats.<instance>.evrp` files written by the solver round to 2 decimals. Use `runs-all.tsv` for paired statistical tests, because the seeds are the same across algorithms.

## 7. Download

On your own computer, from your E-CVRP clone. `ECVRP_REMOTE_ROOT` is the full path of the experiment directory on the cluster (run `pwd` there to get it):

```shell
export ECVRP_REMOTE=<username>@login.hpc.qmul.ac.uk
export ECVRP_REMOTE_ROOT=<experiment directory on the cluster>

./scripts/hpc/download-results.sh --summary   # results/, solution files, stats files, TSV logs
./scripts/hpc/download-results.sh             # everything, including the evols.*.csv traces and Slurm logs
```

Files go to `./hpc-results/<experiment root name>/`, or to `ECVRP_LOCAL_ROOT` if it is set. `--summary` skips the convergence traces, which make up most of the volume. Both modes skip the clone, the data and the binaries.

The equivalent plain command is:

```shell
rsync -avzP --exclude 'source/' --exclude 'data' --exclude 'build/Run' \
  <username>@login.hpc.qmul.ac.uk:<experiment directory on the cluster>/ \
  ./hpc-results/<name>/
```

## Customizing

**Other configurations.** Add rows to `experiments.tsv`, using a unique id and tab-separated columns. For example, a static-policy MA baseline:

```text
MA-STATIC-STRONG	ma	-low_margin 1.05 -ma_ls_policy static -ma_static_action strong
```

Each id gets its own directory and Slurm array, so variants of the same algorithm do not overwrite each other.

**Fewer instances.** Comment out lines in `instances.txt`, or point `ECVRP_INSTANCES` to another list. Array indices refer to the non-comment lines, starting at 0.

**A separate study.** Use a new directory, with its own clone, for each study (for example one for max-evals and one for max-time). That way results never mix.

**Max-time runs.** Set `ECVRP_COMMON_ARGS="-stp 1 -mth 1 -log 1"` before `setup-all.sh`, in a separate experiment root. The time limit must exceed the run time limit of the largest instance (see [Algorithms](algorithms.md#max-time)).

**Settings.** The following environment variables control the scripts:

| Variable | Default | Meaning |
|---|---|---|
| `ECVRP_ROOT` | parent of the clone | experiment root |
| `ECVRP_EXPERIMENTS` | `scripts/hpc/experiments.tsv` | experiment manifest |
| `ECVRP_INSTANCES` | `scripts/hpc/instances.txt` | instance list |
| `ECVRP_COMMON_ARGS` | `-stp 0 -mth 1 -log 1` | arguments added to every run |
| `ECVRP_PARTITION` | `compute` | Slurm partition |
| `ECVRP_CPUS` | `10` | CPUs per array task (one per run) |
| `ECVRP_MEM_PER_CPU` | `1G` | memory per CPU |
| `ECVRP_TIME_LIMIT` | `04:00:00` | wall-clock limit per array task |
| `ECVRP_SLURM_EXTRA` | (empty) | extra `sbatch` options, e.g. `--account=<project>` |
| `ECVRP_MODULES` | `gcc` | modules loaded in each job |
| `ECVRP_BUILD_MODULES` | `gcc cmake` | modules loaded for the build |
| `ECVRP_BUILD_JOBS` | `8` | parallel build jobs |

Slurm settings are written into `script.slurm` by `setup-one.sh`/`setup-all.sh`, so set them before preparing, or prepare again after changing them.

## Resources

Each array task runs its 10 seeds in parallel with OpenMP, so it requests 10 CPUs.

Max-evals runs are short. Single runs on the largest instance, X-n1001-k43, measured on a shared 2-core machine:

| Algorithm | Wall time per run | Peak memory per run |
|---|---:|---:|
| B-LAHC | 7 min | 30 MB |
| BA-BMA | 5 min | 80 MB |
| TwoStageAlns | 2 min | small |

Smaller instances finish much faster. MA's memory peaks on X-n916-k207 at about 0.3 GB per run, or about 3 GB per task.

The defaults leave a wide margin: 4 hours and 1 GB per CPU (10 GB per task). Shorter limits are usually scheduled sooner, so lower `ECVRP_TIME_LIMIT` if your cluster favours short jobs. For max-time runs, raise it above the stopping time instead. If tasks hit the limit, raise it for that experiment and resubmit the missing indices (step 5).
