#!/usr/bin/env python3
"""Per-run trajectory metrics from raw STN logs (best-so-far f_U), averaged over runs.

Columns: algorithm, instance, final_mean, final_best, final_worst, improvements (mean number of best-so-far changes),
budget_to_final (mean fraction of the budget used when the final best-so-far was first logged),
fu_at_1pct / fu_at_10pct / fu_at_50pct (mean best-so-far f_U at that fraction of the budget), distinct_ends.

Example:
    python3 scripts/stn/trajectory_metrics.py --stats stats --alg LAHC=BLAHC --alg MA=BMA X-n214-k11 E-n101-k8
"""

import argparse
import sys
import csv
from pathlib import Path
from statistics import mean


def run_metrics(run_dir: Path, instance: str) -> dict:
    with (run_dir / f"stn_meta.{instance}.csv").open(newline="") as f:
        budget = int({r["key"]: r["value"] for r in csv.DictReader(f)}["budget_distance_calls"])
    best, best_calls, changes, trace = None, 0, 0, []
    with (run_dir / f"stn_raw.{instance}.csv").open(newline="") as f:
        for r in csv.DictReader(f):
            calls, fitness = int(r["distance_calls"]), float(r["fitness"])
            if best is None or fitness < best[0]:
                if best is not None:
                    changes += 1
                best, best_calls = (fitness, r["signature"]), calls
            trace.append((calls, best[0]))

    def at(fraction: float) -> float:
        value = trace[0][1]
        for calls, fitness in trace:
            if calls > fraction * budget:
                break
            value = fitness
        return value

    return {"final": best[0], "end": best[1], "changes": changes, "to_final": best_calls / budget,
            "p1": at(0.01), "p10": at(0.10), "p50": at(0.50)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stats", required=True, type=Path)
    parser.add_argument("--alg", action="append", required=True, metavar="FOLDER=NAME")
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("instances", nargs="+")
    args = parser.parse_args()

    writer = csv.writer(sys.stdout)
    writer.writerow(["algorithm", "instance", "final_mean", "final_best", "final_worst", "improvements",
                     "budget_to_final", "fu_at_1pct", "fu_at_10pct", "fu_at_50pct", "distinct_ends"])
    for instance in args.instances:
        for spec in args.alg:
            folder, _, name = spec.partition("=")
            runs = [run_metrics(args.stats / folder / instance / str(s), instance) for s in range(1, args.runs + 1)]
            finals = [r["final"] for r in runs]
            writer.writerow([name or folder, instance, f"{mean(finals):.2f}", f"{min(finals):.2f}", f"{max(finals):.2f}",
                             f"{mean(r['changes'] for r in runs):.1f}", f"{mean(r['to_final'] for r in runs):.3f}",
                             f"{mean(r['p1'] for r in runs):.2f}", f"{mean(r['p10'] for r in runs):.2f}",
                             f"{mean(r['p50'] for r in runs):.2f}", len({r['end'] for r in runs})])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
