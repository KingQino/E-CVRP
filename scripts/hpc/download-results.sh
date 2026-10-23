#!/usr/bin/env bash
# Run on your own computer: copies the experiment directory from the cluster with rsync.
# Usage: download-results.sh [--summary]
#   --summary  only results/, configuration files and per-run solution/stats/TSV files, plus STN logs (stn/, stn_raw, stn_meta)
#              (skips the large evols.*.csv traces, binaries and Slurm logs).
#
# Required:  ECVRP_REMOTE       e.g. abc123@login.hpc.qmul.ac.uk
#            ECVRP_REMOTE_ROOT  experiment directory on the cluster (the one containing source/; run pwd there)
# Optional:  ECVRP_LOCAL_ROOT   local destination (default: ./hpc-results/<basename of ECVRP_REMOTE_ROOT>)
set -euo pipefail

: "${ECVRP_REMOTE:?set ECVRP_REMOTE, e.g. abc123@login.hpc.qmul.ac.uk}"
: "${ECVRP_REMOTE_ROOT:?set ECVRP_REMOTE_ROOT, the experiment root on the cluster}"
local_root="${ECVRP_LOCAL_ROOT:-./hpc-results/$(basename "$ECVRP_REMOTE_ROOT")}"
mkdir -p "$local_root"

if [[ "${1:-}" == "--summary" ]]; then
    rsync -avzP \
        --exclude 'source/' --exclude 'data' \
        --include '*/' \
        --include 'results/**' \
        --include 'configuration.tsv' --include 'source-commit.txt' --include 'submitted-jobs.tsv' \
        --include 'solution.*.txt' --include 'stats.*.evrp' --include '*.tsv' \
        --include 'stn/**' --include 'stn_raw.*.csv' --include 'stn_meta.*.csv' \
        --exclude '*' \
        "$ECVRP_REMOTE:$ECVRP_REMOTE_ROOT/" "$local_root/"
else
    rsync -avzP \
        --exclude 'source/' --exclude 'data' --exclude 'build/Run' \
        "$ECVRP_REMOTE:$ECVRP_REMOTE_ROOT/" "$local_root/"
fi
echo "Downloaded to $local_root"
