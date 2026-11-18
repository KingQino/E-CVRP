# BA-BMA Ablation Study

Reference (revised BA-BMA): `-low_margin 1.05 -ma_ls_policy online -ma_unrestricted_credit_rho 0.0 -ma_ls_depth d5 -ma_action_cost_weight 0.02`, with SWAP\* on. Every arm changes one component of it. The experiment lists are in [`scripts/ablation/`](../scripts/ablation/).

## Options

| Option | Default | Meaning |
|---|---|---|
| `-ma_ls_policy rank` | | Rule-based adaptive allocation (RAA). After the weak probe of the whole generation, the individuals still eligible (not at a local optimum) are ranked by probe efficiency, ties broken by the lower route cost after the probe; the top share gets Strong, the next Medium, the rest Weak |
| `-ma_ls_policy matched_random` | | The same shares, assigned at random over the eligible individuals of the generation |
| `-ma_action_shares w,m,s` | old matched shares | Weak/medium/strong shares among the eligible individuals, for `rank` and `matched_random`. Counts per generation use largest-remainder rounding |
| `-ma_action_shares_file path` | | TSV `instance weak medium strong` with one row per instance; overrides `-ma_action_shares` |
| `-ma_reward_weights L,G,P` | `0.45,0.25,0.20` | Weights of the lower-archive, gamma-crossing and parent-use reward terms |
| `-ma_context full\|no_probe\|none` | `full` | `no_probe`: probe success rate, probe improvement, probe efficiency and probe cost are 0 for the models (the probe still runs). `none`: context-free, the models see only the constant term |
| `-ma_freeze_learning_at f` | `1` | Stop updating the reward and cost models (and their observation counts) from the first generation that starts with at least a fraction `f` of the budget used. Action selection, its 5% random exploration and the UCB bonus continue on the frozen models |
| `-ma_action_cost_weight 0` | `0.02` | Reward-only action score (no cost penalty) |
| `-ma_swap_star 0` | `1` | No SWAP\* in any local search |

`matched_random` and `rank` probe the whole generation before deciding, so every candidate keeps its own local-search workspace until its continuation; on X-n916-k207 a run then needs about 0.5 GB instead of 0.3 GB. The reference online policy is unchanged and reproduces the earlier runs exactly; the shares, weights, context, freeze point and SWAP\* setting are written to `run-configuration.tsv`.

## Arms

| Id | Stage | Change from the reference |
|---|---|---|
| BA-BMA | 1 | (reference) |
| FIXED-WEAK / FIXED-MEDIUM / FIXED-STRONG | 1 | `-ma_ls_policy static -ma_static_action weak\|medium\|strong` |
| FIXED-UNRESTRICTED | 1 | `-ma_ls_policy static -ma_static_unrestricted 1` |
| RAA-1 / RAA-2 / RAA-3 | 1 | `-ma_ls_policy rank -ma_action_shares 0.5,0.4,0.1` / `0.4,0.4,0.2` / `0.3,0.4,0.3` |
| FREEZE-25 / -50 / -75 | 1 | `-ma_freeze_learning_at 0.25` / `0.50` / `0.75` |
| NO-PROBE-FEATURES | 1 | `-ma_context no_probe` |
| CONTEXT-FREE | 1 | `-ma_context none` |
| NO-L / NO-G / NO-P | 1 | `-ma_reward_weights 0,0.5,0.4` / `0.623077,0,0.276923` / `0.578571,0.321429,0` (remaining weights rescaled to the total 0.90) |
| REWARD-ONLY | 1 | `-ma_action_cost_weight 0` |
| NO-SWAPSTAR | 1 | `-ma_swap_star 0` |
| MATCHED-RANDOM | 2 | `-ma_ls_policy matched_random` with BA-BMA's per-instance shares |
| RAA-M | 2 | `-ma_ls_policy rank` with BA-BMA's per-instance shares |

The shares of stage 2 are those of the individuals eligible after the probe (weak = selections - forced local optima), per run, averaged over the ten BA-BMA runs of each instance (`scripts/ablation/matched_shares.py`).

## Running on the HPC

In a new, empty experiment directory:

```shell
git clone -b ma-ablation git@github.com:KingQino/E-CVRP.git source

# Stage 1: the reference and every arm that does not depend on its shares
export ECVRP_EXPERIMENTS=$PWD/source/scripts/ablation/stage1.tsv
./source/scripts/hpc/setup-all.sh
./source/scripts/hpc/submit-all.sh
./source/scripts/hpc/check-progress.sh          # --resubmit for missing instances

# Stage 2, once BA-BMA has finished on all instances
python3 source/scripts/ablation/matched_shares.py BA-BMA matched-shares.tsv
sed "s#@SHARES@#$PWD/matched-shares.tsv#" source/scripts/ablation/stage2.tsv > stage2.tsv
export ECVRP_EXPERIMENTS=$PWD/stage2.tsv
./source/scripts/hpc/setup-all.sh
./source/scripts/hpc/submit-all.sh
./source/scripts/hpc/check-progress.sh

# Summary of all 20 configurations
{ cat source/scripts/ablation/stage1.tsv; tail -n +2 stage2.tsv; } > all.tsv
export ECVRP_EXPERIMENTS=$PWD/all.tsv
./source/scripts/hpc/summarize-results.sh
```

Keep the `export` line of the current stage in every new shell. `matched-shares.tsv` must stay where it is, since the stage-2 runs read it.
