#!/usr/bin/env python3
"""Convert raw STN logs (stn_raw.<instance>.csv) into STNs-v2 input files.

For every algorithm and run, the representative solution is sampled at checkpoints k * interval * budget
(k = 0, 1, ..., 1/interval): the first raw observation at or after the checkpoint, or the last observation of the
run if the run ended before it. Fitness is rounded to 2 decimals; Solution is the 16-hex canonical signature.

Output: <out>/<instance>/<NAME>_<instance>.csv with the three columns Run,Fitness,Solution, as read by
create.R in https://github.com/gabro8a/STNs-v2 (the algorithm name is the text before the first "_").

Example:
    python3 scripts/stn/make_stn_input.py --stats stats --instance X-n214-k11 \
        --alg LAHC=BLAHC --alg MA=BMA --runs 10 --interval 0.001 --out stn-data
"""

import argparse
import csv
import sys
from pathlib import Path


def read_meta(path: Path) -> dict:
    with path.open(newline="") as f:
        return {row["key"]: row["value"] for row in csv.DictReader(f)}


def read_raw(path: Path) -> list:
    with path.open(newline="") as f:
        return [(int(r["distance_calls"]), float(r["fitness"]), r["signature"]) for r in csv.DictReader(f)]


def best_so_far(observations: list) -> list:
    """Replaces each observation by the best one logged so far in the run (strictly lower fitness wins)."""
    best, rows = None, []
    for calls, fitness, signature in observations:
        if best is None or fitness < best[1]:
            best = (calls, fitness, signature)
        rows.append((calls, best[1], best[2]))
    return rows


def sample(observations: list, budget: int, interval: float) -> list:
    checkpoints = int(round(1.0 / interval))
    rows, i = [], 0
    for k in range(checkpoints + 1):
        target = k * interval * budget
        while i < len(observations) and observations[i][0] < target:
            i += 1
        rows.append(observations[min(i, len(observations) - 1)])
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stats", required=True, type=Path, help="stats root containing <FOLDER>/<instance>/<seed>/")
    parser.add_argument("--instance", required=True, help="instance name without .evrp")
    parser.add_argument("--alg", action="append", required=True, metavar="FOLDER=NAME",
                        help="solver output folder and STN algorithm name, e.g. LAHC=BLAHC (repeatable)")
    parser.add_argument("--runs", type=int, default=10, help="number of runs (seeds 1..runs)")
    parser.add_argument("--interval", type=float, default=0.001, help="checkpoint interval, fraction of the budget")
    parser.add_argument("--best-so-far", action="store_true",
                        help="use the best logged representative so far, so fitness never increases (needed by STNWeb)")
    parser.add_argument("--collapse", action="store_true", help="drop consecutive repeats of the same (Fitness, Solution)")
    parser.add_argument("--out", required=True, type=Path, help="output root (STNs-v2 data folder)")
    args = parser.parse_args()

    out_dir = args.out / args.instance
    out_dir.mkdir(parents=True, exist_ok=True)
    for spec in args.alg:
        folder, _, name = spec.partition("=")
        name = name or folder
        if "_" in name:
            sys.exit(f"algorithm name must not contain '_': {name}")
        out_path = out_dir / f"{name}_{args.instance}.csv"
        with out_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Run", "Fitness", "Solution"])
            for run in range(1, args.runs + 1):
                run_dir = args.stats / folder / args.instance / str(run)
                meta = read_meta(run_dir / f"stn_meta.{args.instance}.csv")
                grid = float(meta["grid"])
                ratio = args.interval / grid
                if abs(ratio - round(ratio)) > 1e-9:
                    print(f"warning: interval {args.interval} is not a multiple of the logging grid {grid}", file=sys.stderr)
                observations = read_raw(run_dir / f"stn_raw.{args.instance}.csv")
                if args.best_so_far:
                    observations = best_so_far(observations)
                if not observations:
                    sys.exit(f"empty raw log: {run_dir}")
                rows = []
                for _, fitness, signature in sample(observations, int(meta["budget_distance_calls"]), args.interval):
                    row = (f"{fitness:.2f}", signature)
                    if args.collapse and rows and row == rows[-1]:
                        continue
                    rows.append(row)
                if len(rows) == 1:
                    # create.R needs at least one edge per run; the repeated row becomes a self-loop it removes.
                    rows.append(rows[0])
                for row in rows:
                    writer.writerow([run, *row])
        print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
