# Charging-Solver Experiments

This branch (`frvcp-experiments`) compares charging methods inside the bilevel search: simple enumeration (SE, with the insert-then-remove fallback) and the exact dynamic programme (DP) that the final refinement already uses.

## Options

| Option | Default | Meaning |
|---|---|---|
| `-lower_solver se\|dp\|verify` | `se` | charging method during the search. `dp` solves every charging problem exactly; its route-to-station distance reads are charged like SE's. `verify` uses SE, then re-solves a candidate exactly (reads charged) when its SE cost, or its uncharged length if SE fails, is within `-verify_margin` of the incumbent's heuristic cost (heuristic against heuristic; the incumbent is then replaced only on exact costs) |
| `-verify_margin` | `0.02` | relative margin for `verify`; the experiments use 0 and 0.005 |
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
| Verify | SE during search, exact DP on promising candidates, final DP | `-lower_solver verify` (writes `verify.<instance>.csv`: evaluations, verified, improved, rescued from false infeasibility) |

`best_exact_cost_evaluated` is the lowest exact cost of any solution evaluated during the SE run. For b-LAHC, whose search never depends on charging costs, the gap between H–DP and this value is the loss caused by ranking candidates with SE costs.

## Shadow output

In `stats/<ALG>/<instance>/<seed>/`:

| File | Content |
|---|---|
| `shadow.<instance>.csv` | per tenth of the budget: follower calls, distinct charging routes, routes where SE is optimal, SE failures (fallback used), false infeasibility, gap sum and maximum (%), and decision disagreements: kind 0 = b-LAHC global-best update, 1 = MA choice of the best evaluated individual, 2 = MA update of the best complete solution |
| `shadow_summary.<instance>.csv` | heuristic final cost, exact cost of the same routes, best exact cost evaluated |
| `shadow_routes.<instance>.csv` | reservoir sample of up to 2,000 charging routes with heuristic and exact cost |

## Running on HPC

Use the HPC scripts with the experiment list in [`scripts/frvcp/experiments.tsv`](../scripts/frvcp/experiments.tsv) (b-LAHC) and all 133 instances. The BMA rows are in [`experiments-ma.tsv`](../scripts/frvcp/experiments-ma.tsv), for an optional separate run. In an empty directory on the cluster:

```shell
git clone -b frvcp-experiments git@github.com:KingQino/E-CVRP.git source
export ECVRP_EXPERIMENTS=$PWD/source/scripts/frvcp/experiments.tsv
export ECVRP_TIME_LIMIT=08:00:00
./source/scripts/hpc/setup-all.sh
./source/scripts/hpc/submit-all.sh
./source/scripts/hpc/check-progress.sh          # --resubmit for missing instances
python3 source/scripts/frvcp/summarize_frvcp.py . frvcp-summary LAHC-SE1,LAHC-SE1-SHADOW,LAHC-SE2-SHADOW,LAHC-DP,LAHC-VERIFY-M0,LAHC-VERIFY-M005
```

Keep the `export` lines in every new shell. Shadow and DP runs are slower in wall-clock time than SE runs (up to about 25 times for b-LAHC), hence the 8-hour limit. On your computer, `./scripts/hpc/download-results.sh --summary` downloads the solutions, `frvcp-summary/` and the `shadow*` files.

Runs made with commit fda5b4b (ids `LAHC-VERIFY`, `LAHC-VERIFY-02`) compared the heuristic cost of each candidate with the exact cost of a verified incumbent; after the first verification few candidates passed the screen, so those results understate the variant. To rerun only the corrected variant, use `experiments-verify.tsv`.

For the BMA rows, use a separate directory with `export ECVRP_EXPERIMENTS=$PWD/source/scripts/frvcp/experiments-ma.tsv` and the ids `MA-SE,MA-SE-SHADOW,MA-DP`.

## Letter tables

`scripts/frvcp/analyse_letter.py <results_dir> data <out_dir> [route_bench.csv]` builds the search-level table (gap to H-DP and win/tie/loss per group, duplicate instances removed) and, given a route benchmark, the route-level table. The benchmark times SE (first count), SE (both counts) and the DP on the routes sampled by shadow runs (`shadow_routes.*.csv`, columns `instance,seed,stage,heuristic_cost,exact_cost,route`):

```shell
g++ -std=c++20 -O3 -fopenmp -Iinclude -Iexternal/include scripts/frvcp/frvcp_bench.cpp $(ls src/*.cpp) -o frvcp_bench
./frvcp_bench data route_sample.csv route_bench.csv
```
