#!/usr/bin/env python3
"""Sanity checks for raw STN logs of one instance.

- hash collisions: one signature mapped to two different canonical routings (needs stn_solutions files);
- fitness consistency: the largest spread of f_U recorded for the same signature (should be ~0);
- per algorithm: raw rows, distinct signatures, and signatures shared between algorithms.

Example:
    python3 scripts/stn/check_stn_logs.py --stats stats --instance X-n214-k11 --alg LAHC --alg MA --runs 10
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stats", required=True, type=Path)
    parser.add_argument("--instance", required=True)
    parser.add_argument("--alg", action="append", required=True, help="solver output folder, e.g. LAHC (repeatable)")
    parser.add_argument("--runs", type=int, default=10)
    args = parser.parse_args()

    routes_of = {}
    collisions = 0
    fitness_of = defaultdict(list)
    signatures_by_alg = {}
    for folder in args.alg:
        rows, signatures = 0, set()
        for run in range(1, args.runs + 1):
            run_dir = args.stats / folder / args.instance / str(run)
            with (run_dir / f"stn_raw.{args.instance}.csv").open(newline="") as f:
                for r in csv.DictReader(f):
                    rows += 1
                    signatures.add(r["signature"])
                    fitness_of[r["signature"]].append(float(r["fitness"]))
            solutions = run_dir / f"stn_solutions.{args.instance}.csv"
            if solutions.exists():
                with solutions.open(newline="") as f:
                    for r in csv.DictReader(f):
                        previous = routes_of.setdefault(r["signature"], r["routes"])
                        collisions += previous != r["routes"]
        signatures_by_alg[folder] = signatures
        print(f"{folder}: {rows} raw rows, {len(signatures)} distinct signatures")

    spread = max((max(v) - min(v) for v in fitness_of.values()), default=0.0)
    rounding_splits = sum(len({f"{x:.2f}" for x in v}) > 1 for v in fitness_of.values())
    print(f"hash collisions: {collisions}")
    print(f"max f_U spread for one signature: {spread:.3e} ({rounding_splits} signatures with two rounded values)")
    algs = list(signatures_by_alg)
    for i, a in enumerate(algs):
        for b in algs[i + 1:]:
            print(f"signatures shared by {a} and {b}: {len(signatures_by_alg[a] & signatures_by_alg[b])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
