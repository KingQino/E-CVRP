#!/usr/bin/env python3
"""Convert STNs-v2 input files (Run,Fitness,Solution) into STNs v1 edge lists.

STNs v1 (https://github.com/gabro8a/STNs) reads one transition per line:
Run,Fitness1,Solution1,Fitness2,Solution2, where each row links two consecutive rows of the same run.

Example:
    python3 scripts/stn/stn_v2_to_v1.py stn-web/X-n214-k11 stn-v1/X-n214-k11
"""

import csv
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    source, target = Path(sys.argv[1]), Path(sys.argv[2])
    target.mkdir(parents=True, exist_ok=True)
    for path in sorted(source.glob("*.csv")):
        with path.open(newline="") as f:
            rows = [(int(r["Run"]), r["Fitness"], r["Solution"]) for r in csv.DictReader(f)]
        with (target / path.name).open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Run", "Fitness1", "Solution1", "Fitness2", "Solution2"])
            edges = 0
            for (run1, fit1, sol1), (run2, fit2, sol2) in zip(rows, rows[1:]):
                if run1 == run2:
                    writer.writerow([run1, fit1, sol1, fit2, sol2])
                    edges += 1
        print(f"wrote {target / path.name}: {edges} transitions")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
