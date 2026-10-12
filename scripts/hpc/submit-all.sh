#!/usr/bin/env bash
# Submits the Slurm array of every experiment (or only the ids given as arguments).
# Usage: submit-all.sh [experiment-id ...]
set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/common.sh"

if (( $# > 0 )); then ids=("$@"); else mapfile -t ids < <(experiment_ids); fi
submitted="$root_dir/submitted-jobs.tsv"
[[ -f "$submitted" ]] || printf 'id\tjob_id\tarray\tsubmitted_at\n' > "$submitted"
for id in "${ids[@]}"; do
    slurm_script="$root_dir/$id/build/script.slurm"
    [[ -f "$slurm_script" ]] || { echo "Error: missing $slurm_script; run setup-one.sh $id first" >&2; exit 1; }
    # shellcheck disable=SC2086
    job_id="$(sbatch --parsable $slurm_extra "$slurm_script")"
    printf '%s\t%s\t%s\t%s\n' "$id" "$job_id" "all" "$(date -Is)" | tee -a "$submitted"
done
