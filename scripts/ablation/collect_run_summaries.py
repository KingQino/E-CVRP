#!/usr/bin/env python3
"""Collects the run-summary.tsv of every run into one CSV (run on the cluster after the jobs have finished).

Usage: python3 collect_run_summaries.py <experiment root> <experiments.tsv> <out.csv>

For every experiment id in <experiments.tsv>, reads <root>/<id>/stats/MA/<instance>/<seed>/run-summary.tsv and writes one
row per run: experiment, instance, seed and the run-summary columns. Runs without a summary are listed on stderr. The
final cost is checked against the first line of the run's solution file.
"""
import csv
import sys
from pathlib import Path


def main() -> int:
    root, experiments_file, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    ids = [row["id"] for row in csv.DictReader(experiments_file.open(), delimiter="\t")]
    rows, missing, mismatched = [], 0, 0
    for experiment in ids:
        base = root / experiment / "stats" / "MA"
        if not base.is_dir():
            print(f"missing experiment {experiment}", file=sys.stderr)
            continue
        for instance_dir in sorted(p for p in base.iterdir() if p.is_dir()):
            for seed_dir in sorted((p for p in instance_dir.iterdir() if p.is_dir() and p.name.isdigit()),
                                   key=lambda p: int(p.name)):
                summary = seed_dir / "run-summary.tsv"
                if not summary.exists():
                    missing += 1
                    print(f"no summary: {seed_dir}", file=sys.stderr)
                    continue
                row = next(csv.DictReader(summary.open(), delimiter="\t"))
                solution = seed_dir / f"solution.{instance_dir.name}.txt"
                if solution.exists():
                    reported = float(solution.read_text().split("\n", 1)[0])
                    if abs(reported - float(row["final_cost"])) > 1e-4:
                        mismatched += 1
                        print(f"cost mismatch: {seed_dir}", file=sys.stderr)
                row.pop("seed", None)
                rows.append({"experiment": experiment, "instance": instance_dir.name, "seed": seed_dir.name, **row})
    if not rows:
        print("no runs found", file=sys.stderr)
        return 1
    with out.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {out}: {len(rows)} runs, {missing} without summary, {mismatched} cost mismatches")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
