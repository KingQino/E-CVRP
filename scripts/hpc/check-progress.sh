#!/usr/bin/env bash
# Reports finished instances per experiment and the array indices still missing.
# Usage: check-progress.sh [--resubmit] [experiment-id ...]
#   --resubmit  resubmit only the missing array indices of experiments that have no job in the queue.
set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/common.sh"

resubmit=0
if [[ "${1:-}" == "--resubmit" ]]; then resubmit=1; shift; fi
if (( $# > 0 )); then ids=("$@"); else mapfile -t ids < <(experiment_ids); fi
total="$(instance_names | wc -l | tr -d ' ')"
submitted="$root_dir/submitted-jobs.tsv"

printf '%-20s %9s  %s\n' "experiment" "finished" "missing array indices"
for id in "${ids[@]}"; do
    if [[ ! -d "$root_dir/$id/build" ]]; then
        printf '%-20s %9s  %s\n' "$id" "-" "(not set up)"
        continue
    fi
    missing="$(missing_indices "$id" | compress_indices)"
    count_missing=0
    [[ -n "$missing" ]] && count_missing="$(missing_indices "$id" | wc -l | tr -d ' ')"
    printf '%-20s %4d/%-4d  %s\n' "$id" "$((total - count_missing))" "$total" "${missing:-none}"

    # Slurm logs that report a failure (time limit, out of memory, solver error).
    failed_logs="$(grep -lE 'DUE TO TIME LIMIT|oom-kill|Out Of Memory|Error' "$root_dir/$id/build/log/"*.out 2>/dev/null || true)"
    if [[ -n "$failed_logs" ]]; then
        echo "    logs reporting failures:"
        printf '      %s\n' $failed_logs
    fi

    if (( resubmit )) && [[ -n "$missing" ]]; then
        if command -v squeue >/dev/null 2>&1 && [[ -n "$(squeue -h -u "$USER" -n "ecvrp-$id" 2>/dev/null)" ]]; then
            echo "    skipped resubmission: ecvrp-$id still has jobs in the queue"
            continue
        fi
        # shellcheck disable=SC2086
        job_id="$(sbatch --parsable --array="$missing" $slurm_extra "$root_dir/$id/build/script.slurm")"
        [[ -f "$submitted" ]] || printf 'id\tjob_id\tarray\tsubmitted_at\n' > "$submitted"
        printf '%s\t%s\t%s\t%s\n' "$id" "$job_id" "$missing" "$(date -Is)" >> "$submitted"
        echo "    resubmitted --array=$missing as job $job_id"
    fi
done
