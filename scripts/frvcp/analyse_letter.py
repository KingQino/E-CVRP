#!/usr/bin/env python3
"""Tables for the charging letter from the b-LAHC runs of branch frvcp-experiments.

Usage: python3 analyse_letter.py <results_dir> <data_dir> <out_dir> [route_bench.csv]

<results_dir> holds one directory per experiment id (LAHC-SE1, LAHC-SE1-SHADOW, ...), as downloaded from the HPC.
Instances whose .evrp file equals an earlier one apart from NAME/COMMENT are dropped (20 of the 133).
Policies (per run): H-H and H-DP from LAHC-SE1-SHADOW (heuristic final cost and its exact cost), BestSeen (lowest exact
cost of any solution evaluated by the same run), SE2 H-DP (LAHC-SE2-SHADOW), DP-DP (LAHC-DP), Verify-* (LAHC-VERIFY-*).
Each policy is compared with H-DP per instance: gap of the means (%), and win/tie/loss at p < 0.05 (Wilcoxon signed-rank
for policies from the same runs, Mann-Whitney U otherwise).
Writes per_seed.csv, policies.csv, vs_hdp.csv, search_table.csv, route_table.csv (if route_bench.csv is given).
"""
import csv, hashlib, re, sys
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.stats import mannwhitneyu, wilcoxon

GROUPS = ["E", "F", "M", "X", "Small-5", "Small-10", "Small-15", "c", "r", "rc"]


def group_of(i):
    m = re.match(r"^([EFMX])-n", i)
    if m:
        return m.group(1)
    m = re.match(r"^(C|R|RC)\d+-(\d+)$", i)
    if m:
        return "Small-" + m.group(2)
    return re.match(r"^(rc|c|r)", i).group(1)


def duplicates(data_dir):
    seen, dup = {}, set()
    for f in sorted(Path(data_dir).rglob("*.evrp")):
        key = hashlib.md5(re.sub(r"^(NAME|COMMENT).*$", "", f.read_text(), flags=re.M).encode()).hexdigest()
        if key in seen:
            dup.add(f.stem)
        else:
            seen[key] = f.stem
    return dup


def read_runs(root):
    rows = []
    for exp in sorted(p for p in Path(root).iterdir() if p.is_dir() and p.name.startswith("LAHC-")):
        for inst in sorted((exp / "stats" / "LAHC").iterdir()):
            for seed in sorted((s for s in inst.iterdir() if s.name.isdigit()), key=lambda s: int(s.name)):
                r = dict(experiment=exp.name, instance=inst.name, seed=int(seed.name),
                         final=float((seed / f"solution.{inst.name}.txt").read_text().split("\n", 1)[0]))
                for name in ("shadow_summary", "verify"):
                    p = seed / f"{name}.{inst.name}.csv"
                    if p.exists():
                        r.update({k: float(v) for k, v in next(csv.DictReader(p.open())).items()})
                rows.append(r)
    return pd.DataFrame(rows)


def main():
    root, data_dir, out = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=True)
    d = read_runs(root)
    d.to_csv(out / "per_seed.csv", index=False)
    dup = duplicates(data_dir)
    d = d[~d.instance.isin(dup)]
    by = lambda e: d[d.experiment == e].set_index(["instance", "seed"])
    s1, s2 = by("LAHC-SE1-SHADOW"), by("LAHC-SE2-SHADOW")
    pol = {"H-H": s1.heuristic_final_cost, "H-DP": s1.exact_cost_of_heuristic_final,
           "BestSeen": s1.best_exact_cost_evaluated, "SE2 H-DP": s2.exact_cost_of_heuristic_final,
           "DP-DP": by("LAHC-DP").final}
    for e in sorted(x for x in d.experiment.unique() if x.startswith("LAHC-VERIFY")):
        pol[e.replace("LAHC-", "")] = by(e).final
    P = pd.DataFrame(pol)
    P.to_csv(out / "policies.csv")
    paired = {"H-H", "BestSeen"}
    rows = []
    for inst, g in P.groupby(level=0):
        ref = g["H-DP"].values
        for k in pol:
            if k == "H-DP":
                continue
            x = g[k].values
            if np.allclose(x, ref, atol=1e-6):
                res = "T"
            else:
                try:
                    pv = wilcoxon(x, ref).pvalue if k in paired else mannwhitneyu(x, ref).pvalue
                except ValueError:
                    pv = 1.0
                res = "T" if pv >= 0.05 else ("W" if x.mean() < ref.mean() else "L")
            rows.append(dict(instance=inst, group=group_of(inst), policy=k,
                             gap=100 * (x.mean() - ref.mean()) / ref.mean(), res=res))
    R = pd.DataFrame(rows)
    R.to_csv(out / "vs_hdp.csv", index=False)
    order = [k for k in pol if k != "H-DP"]
    wtl = lambda s: "{}/{}/{}".format(*(int((s == c).sum()) for c in "WTL"))
    table = []
    for g in GROUPS + ["All"]:
        sel = R if g == "All" else R[R.group == g]
        row = {"group": g, "n": sel.instance.nunique()}
        for k in order:
            s = sel[sel.policy == k]
            row[k] = f"{s.gap.mean():+.2f} ({wtl(s.res)})"
        table.append(row)
    pd.DataFrame(table).to_csv(out / "search_table.csv", index=False)
    print(pd.DataFrame(table).to_string(index=False))
    if len(sys.argv) > 4:
        o = pd.read_csv(sys.argv[4])
        o = o[(o.dp_cost > 0) & ~o.instance.isin(dup)]
        fam = lambda g: "WCCI" if g in "EFMX" else ("Small" if g.startswith("Small") else "Schneider")
        o["family"] = o.instance.map(lambda i: fam(group_of(i)))
        rows = []
        for f in ["WCCI", "Small", "Schneider", "All"]:
            s = o if f == "All" else o[o.family == f]
            for k, name in (("se1", "SE (first count)"), ("se2", "SE (both counts)"), ("dp", "DP")):
                c = s[k + "_cost"]
                gap = np.where(c < 0, np.nan, 100 * (c - s.dp_cost) / s.dp_cost)
                t = s.assign(gap=gap, fail=c < 0, opt=np.where(c < 0, np.nan, gap < 1e-6)).groupby("instance")
                rows.append(dict(family=f, solver=name, instances=s.instance.nunique(), routes=len(s),
                                 optimal_pct=100 * t.opt.mean().mean(), mean_gap_pct=t.gap.mean().mean(),
                                 max_gap_pct=np.nanmax(gap), fail_pct=100 * t.fail.mean().mean(),
                                 median_us=t[k + "_us"].median().mean(), p95_us=s[k + "_us"].quantile(.95),
                                 reads=t[k + "_reads"].mean().mean()))
        pd.DataFrame(rows).round(3).to_csv(out / "route_table.csv", index=False)
        print(pd.DataFrame(rows).round(2).to_string(index=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
