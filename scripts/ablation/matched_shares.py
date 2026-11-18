#!/usr/bin/env python3
"""Per-instance action shares of the reference BA-BMA, for the matched-random and RAA-M arms.

Usage: python3 matched_shares.py <reference experiment dir> <out.tsv>

Reads <dir>/stats/MA/<instance>/<seed>/local-search-allocation.tsv of every run. For each run, the shares are those of
the individuals still eligible after the weak probe: weak = selections - forced local optima, medium and strong =
selections. The shares of an instance are the means over its runs. Output: instance, weak, medium, strong, runs.
"""
import csv
import sys
from collections import defaultdict
from pathlib import Path


def run_shares(path: Path):
    counts = defaultdict(float)
    with path.open() as f:
        for row in csv.DictReader(f, delimiter="\t"):
            selections = float(row["selections"])
            if row["action"] == "weak":
                selections -= float(row["forced_local_optima"])
            counts[row["action"]] += selections
    total = sum(counts[a] for a in ("weak", "medium", "strong"))
    if total <= 0:
        return None
    return [counts[a] / total for a in ("weak", "medium", "strong")]


def main() -> int:
    root, out = Path(sys.argv[1]), Path(sys.argv[2])
    stats = root / "stats" / "MA"
    rows = []
    for instance_dir in sorted(p for p in stats.iterdir() if p.is_dir()):
        shares = [s for s in (run_shares(seed / "local-search-allocation.tsv")
                              for seed in instance_dir.iterdir()
                              if seed.is_dir() and (seed / "local-search-allocation.tsv").exists()) if s]
        if not shares:
            print(f"no allocation logs for {instance_dir.name}", file=sys.stderr)
            continue
        mean = [sum(s[i] for s in shares) / len(shares) for i in range(3)]
        rows.append([instance_dir.name] + [f"{v:.6f}" for v in mean] + [len(shares)])
    with out.open("w", newline="") as f:
        w = csv.writer(f, delimiter="\t")
        w.writerow(["instance", "weak", "medium", "strong", "runs"])
        w.writerows(rows)
    print(f"wrote {out}: {len(rows)} instances")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
