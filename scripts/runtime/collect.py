#!/usr/bin/env python3
"""Collects the runtime profile into two CSV files (run on the cluster, standard library only).

Usage: collect.py <runtime-profile directory> [output directory, default: <dir>/results]

  profile-parts.csv  one row per run and part: arm, instance, seed, part, budget_counted, seconds, distance_calls,
                     evals, entries
  profile-runs.csv   one row per run: profiled seconds split into budget-counted and uncounted parts, the learner
                     (context + learner_select + learner_update), generations, final cost, evaluations, and the
                     process wall time, host and CPU model from timing/<instance>.tsv
"""
import csv
import os
import sys

LEARNER = ("context", "learner_select", "learner_update")


def read_tsv(path):
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle, delimiter="\t"))


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    root = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(root, "results")
    os.makedirs(out_dir, exist_ok=True)

    timing = {}
    timing_dir = os.path.join(root, "timing")
    for name in sorted(os.listdir(timing_dir)) if os.path.isdir(timing_dir) else []:
        for row in read_tsv(os.path.join(timing_dir, name)):
            timing[(row["arm"], row["instance"], row["seed"])] = row

    arms = [row["id"] for row in read_tsv(os.path.join(root, "arms.tsv"))]
    part_rows, run_rows = [], []
    for arm in arms:
        base = os.path.join(root, arm, "stats", "MA")
        if not os.path.isdir(base):
            continue
        for instance in sorted(os.listdir(base)):
            instance_dir = os.path.join(base, instance)
            for seed in sorted(os.listdir(instance_dir)):
                run_dir = os.path.join(instance_dir, seed)
                profile = os.path.join(run_dir, "profile.tsv")
                if not os.path.isfile(profile):
                    continue
                rows = read_tsv(profile)
                counted = uncounted = learner = 0.0
                total = {}
                seconds_by_part = {}
                for row in rows:
                    if row["part"] == "total":
                        total = row
                        continue
                    seconds = float(row["seconds"])
                    seconds_by_part[row["part"]] = seconds
                    if row["budget_counted"] == "1":
                        counted += seconds
                    else:
                        uncounted += seconds
                    if row["part"] in LEARNER:
                        learner += seconds
                    part_rows.append({"arm": arm, "instance": instance, "seed": seed, **row})
                summary = read_tsv(os.path.join(run_dir, "run-summary.tsv"))[0]
                clock = timing.get((arm, instance, seed), {})
                run_rows.append({
                    "arm": arm, "instance": instance, "seed": seed,
                    "profiled_seconds": counted + uncounted,
                    "counted_seconds": counted, "uncounted_seconds": uncounted, "learner_seconds": learner,
                    **{f"s_{part}": value for part, value in seconds_by_part.items()},
                    "profiled_distance_calls": total.get("distance_calls", ""),
                    "generations": summary["generations"], "final_cost": summary["final_cost"],
                    "total_evals": summary["total_evals"],
                    "process_seconds": clock.get("process_seconds", ""),
                    "host": clock.get("host", ""), "cpu": clock.get("cpu", ""),
                })

    for name, rows in (("profile-parts.csv", part_rows), ("profile-runs.csv", run_rows)):
        path = os.path.join(out_dir, name)
        fields = list(rows[0].keys()) if rows else []
        with open(path, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        print(f"{path}: {len(rows)} rows")


if __name__ == "__main__":
    main()
