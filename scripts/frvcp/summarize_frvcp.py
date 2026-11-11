#!/usr/bin/env python3
"""Summarise the charging-solver experiments (branch frvcp-experiments).

Reads, for every experiment directory <root>/<id>/stats/<ALG>/<instance>/<seed>/:
  solution.<instance>.txt         final reported cost (first line)
  shadow_summary.<instance>.csv   SE runs with -shadow_dp 1: heuristic final cost, exact cost of the same routes,
                                  best exact cost of any evaluated solution
  shadow.<instance>.csv           SE runs: per budget tenth, route-level and decision statistics

Writes to <out>:
  per_instance.csv   one row per experiment and instance (means over seeds)
  per_group.csv      means per instance group (E, F, M, X, Small-5/10/15, c, r, rc)

Usage: python3 summarize_frvcp.py <root> <out_dir> <id>[,<id>...]
"""

import csv
import re
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path

GROUPS = ["E", "F", "M", "X", "Small-5", "Small-10", "Small-15", "c", "r", "rc"]


def group_of(instance: str) -> str:
    m = re.match(r"^([EFMX])-n", instance)
    if m:
        return m.group(1)
    m = re.match(r"^(C|R|RC)\d+-(\d+)$", instance)
    if m:
        return "Small-" + m.group(2)
    return re.match(r"^(rc|c|r)", instance).group(1)


def read_first_line(path: Path):
    try:
        return float(path.read_text().split("\n", 1)[0])
    except (OSError, ValueError):
        return None


def summarise_instance(inst_dir: Path, instance: str) -> dict:
    final, hh, hdp, best_seen = [], [], [], []
    totals = defaultdict(float)
    for seed_dir in sorted(p for p in inst_dir.iterdir() if p.is_dir()):
        value = read_first_line(seed_dir / f"solution.{instance}.txt")
        if value is not None:
            final.append(value)
        summary = seed_dir / f"shadow_summary.{instance}.csv"
        if summary.exists():
            row = next(csv.DictReader(summary.open()))
            hh.append(float(row["heuristic_final_cost"]))
            hdp.append(float(row["exact_cost_of_heuristic_final"]))
            best_seen.append(float(row["best_exact_cost_evaluated"]))
        stages = seed_dir / f"shadow.{instance}.csv"
        if stages.exists():
            for row in csv.DictReader(stages.open()):
                for key, value in row.items():
                    if key == "stage":
                        continue
                    if key == "gap_max":
                        totals[key] = max(totals[key], float(value))
                    else:
                        totals[key] += float(value)
    out = {"runs": len(final), "final_mean": st.mean(final) if final else None}
    if hh:
        out.update(hh_mean=st.mean(hh), hdp_mean=st.mean(hdp), best_exact_seen_mean=st.mean(best_seen))
    if totals:
        routes = totals["charging_routes"]
        solved = routes - totals["false_infeasible"] - totals["both_infeasible"]
        out.update(
            calls=totals["calls"],
            charging_routes=routes,
            se_optimal_pct=100 * totals["se_optimal"] / solved if solved else None,
            false_infeasible_pct=100 * totals["false_infeasible"] / routes if routes else None,
            se_failed_pct=100 * totals["se_failed"] / routes if routes else None,
            mean_gap_pct=totals["gap_sum"] / solved if solved else None,
            max_gap_pct=totals["gap_max"],
        )
        for k, name in ((0, "lahc_best_update"), (1, "ma_best_choice"), (2, "ma_best_update")):
            d = totals[f"decisions_{k}"]
            if d:
                out[f"{name}_disagree_pct"] = 100 * totals[f"disagreements_{k}"] / d
    return out


def main() -> int:
    root, out_dir, ids = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3].split(",")
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for exp in ids:
        stats = root / exp / "stats"
        if not stats.exists():
            print(f"missing {stats}", file=sys.stderr)
            continue
        for alg_dir in stats.iterdir():
            for inst_dir in sorted(p for p in alg_dir.iterdir() if p.is_dir()):
                row = {"experiment": exp, "instance": inst_dir.name, "group": group_of(inst_dir.name)}
                row.update(summarise_instance(inst_dir, inst_dir.name))
                rows.append(row)
    keys = sorted({k for r in rows for k in r}, key=lambda k: (k not in ("experiment", "instance", "group"), k))
    with (out_dir / "per_instance.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    with (out_dir / "per_group.csv").open("w", newline="") as f:
        w = None
        for exp in ids:
            for g in GROUPS:
                sel = [r for r in rows if r["experiment"] == exp and r["group"] == g]
                if not sel:
                    continue
                agg = {"experiment": exp, "group": g, "instances": len(sel)}
                for k in keys:
                    vals = [r[k] for r in sel if isinstance(r.get(k), (int, float)) and r.get(k) is not None]
                    if vals and k not in ("runs",):
                        agg[k] = round(st.mean(vals), 4)
                if w is None:
                    w = csv.DictWriter(f, fieldnames=["experiment", "group", "instances"] + [k for k in keys if k not in ("experiment", "instance", "group")])
                    w.writeheader()
                w.writerow({k: v for k, v in agg.items() if k in w.fieldnames})
    print(f"wrote {out_dir}/per_instance.csv and per_group.csv ({len(rows)} rows)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
