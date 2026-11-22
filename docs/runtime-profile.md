# Runtime profile of BA-BMA

Branch `runtime-profile`. Measures how much wall-clock time the MA spends in operations that are counted in the
evaluation budget and in operations that are not, in answer to the question whether the distance-call budget is a
representative measure of the computational effort.

## Instrumentation

`-ma_profile 1` (with `-log 1`) writes `profile.tsv` into the run directory. The run itself is unchanged: final cost,
generations and evaluations are identical with the profile on or off (unit test
`MaRuntimeProfileTest.ProfileDoesNotChangeTheRunAndAttributesDistanceCalls`).

Time and distance calls are attributed exclusively to the innermost open part; time outside every part is `other`.

| Part | Counted in the budget | Content |
|---|---|---|
| init | yes | initial population (construction heuristic, split) |
| ls_probe | yes | Weak probe |
| ls_continuation | yes | Medium/Strong continuation |
| ls_static | yes | local search of the static policies (BMA: unrestricted RVND) |
| follower | yes | charging of the gamma-filtered individuals |
| split | yes | Prins split of the offspring |
| final_refine | yes | final exact charging refinement |
| context | no | edge signatures, structural distance, population dispersion, context vector |
| learner_select | no | scoring of the three actions and choice |
| learner_update | no | rewards and Sherman-Morrison model updates |
| feedback | no | lower-archive and parent-use credit |
| parent_pool | no | sorting by route cost, duplicate removal |
| reproduction | no | tournament selection, crossover, mutation |
| population | no | assembly of the next population (without the split) |
| logging | no | result files |
| other | no | remaining generation bookkeeping |

Columns: `part, budget_counted, seconds, distance_calls, evals, entries`, and a final `total` row (its last column is
the number of generations). Distance calls occur only in the budget-counted parts.

## Experiment

Arms (`scripts/runtime/arms.tsv`): BA-BMA (reference configuration of the ablation study) and BMA (the original BMA:
unrestricted RVND for every individual, no learner; FIXED-UNRESTRICTED of the ablation study).

Instances: the 113 distinct instances (`scripts/runtime/instances.txt`). Seeds 1-10. One Slurm task per instance on one
core of a `ddy` node; for each seed both arms run one after the other, so they share node and load.

```
scripts/hpc/build-source.sh
scripts/runtime/setup.sh
sbatch $ECVRP_ROOT/runtime-profile/job.slurm
python3 source/scripts/runtime/collect.py $ECVRP_ROOT/runtime-profile
```

`collect.py` writes `results/profile-parts.csv` (run x part) and `results/profile-runs.csv` (one row per run, with the
process wall time, host and CPU model).
