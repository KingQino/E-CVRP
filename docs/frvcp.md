# Charging-Solver Experiments

This branch (`frvcp-experiments`) compares charging methods inside the bilevel search: simple enumeration (SE, with the insert-then-remove fallback) and the exact dynamic programme (DP) that the final refinement already uses.

## Options

| Option | Default | Meaning |
|---|---|---|
| `-lower_solver se\|dp` | `se` | charging method during the search. `dp` solves every charging problem exactly; its route-to-station distance reads are charged like SE's |
| `-shadow_dp 0\|1` | `0` | SE runs only: also solve every charging problem exactly, with the reads restored afterwards, so the search and its budget are unchanged. Writes diagnostics (below) |
| `-se_both_counts 0\|1` | `0` | SE tries both station counts l_min and l_min + 1. MA always does; b-LAHC's published setting is `0` (first feasible count) |
| `-refine 0\|1` | `1` | exact DP on the final solution |

Station-to-station distances are instance preprocessing (`Preprocessor::station_distances_`), computed once and not charged, like the best-station table used by SE.

## Policies

| Policy | Run | How it is obtained |
|---|---|---|
| H–H | SE during search, no final DP | `final_mean` of a `-shadow_dp 1 -refine 0` run, or `heuristic_final_cost` in `shadow_summary` |
| H–DP | SE during search, final DP | `exact_cost_of_heuristic_final` of the same run (identical to running with `-refine 1`) |
| DP–DP | DP during search | `-lower_solver dp` |

`best_exact_cost_evaluated` is the lowest exact cost of any solution evaluated during the SE run. For b-LAHC, whose search never depends on charging costs, the gap between H–DP and this value is the loss caused by ranking candidates with SE costs.

## Shadow output

In `stats/<ALG>/<instance>/<seed>/`:

| File | Content |
|---|---|
| `shadow.<instance>.csv` | per tenth of the budget: follower calls, distinct charging routes, routes where SE is optimal, SE failures (fallback used), false infeasibility, gap sum and maximum (%), and decision disagreements: kind 0 = b-LAHC global-best update, 1 = MA choice of the best evaluated individual, 2 = MA update of the best complete solution |
| `shadow_summary.<instance>.csv` | heuristic final cost, exact cost of the same routes, best exact cost evaluated |
| `shadow_routes.<instance>.csv` | reservoir sample of up to 2,000 charging routes with heuristic and exact cost |

## Running on HPC

Use the HPC scripts with the experiment list in [`scripts/frvcp/experiments.tsv`](../scripts/frvcp/experiments.tsv) and all 133 instances. In an empty directory on the cluster:

```shell
git clone -b frvcp-experiments git@github.com:KingQino/E-CVRP.git source
export ECVRP_EXPERIMENTS=$PWD/source/scripts/frvcp/experiments.tsv
export ECVRP_TIME_LIMIT=08:00:00
./source/scripts/hpc/setup-all.sh
./source/scripts/hpc/submit-all.sh
./source/scripts/hpc/check-progress.sh          # --resubmit for missing instances
python3 source/scripts/frvcp/summarize_frvcp.py . frvcp-summary LAHC-SE1,LAHC-SE1-SHADOW,LAHC-SE2-SHADOW,LAHC-DP,MA-SE,MA-SE-SHADOW,MA-DP
```

Keep the `export` lines in every new shell. Shadow and DP runs are slower in wall-clock time than SE runs (up to about 25 times for b-LAHC), hence the 8-hour limit. On your computer, `./scripts/hpc/download-results.sh --summary` downloads the solutions, `frvcp-summary/` and the `shadow*` files.
