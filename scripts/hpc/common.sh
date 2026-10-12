#!/usr/bin/env bash
# Shared settings and helpers for the HPC scripts. Sourced by the other scripts; not run directly.
#
# Directory layout on the cluster (ECVRP_ROOT defaults to the parent of the cloned repository):
#
#   $ECVRP_ROOT/
#     source/                      the git clone (these scripts live in source/scripts/hpc/)
#     <id>/                        one directory per row of experiments.tsv
#       build/Run                  copy of the solver binary
#       build/instances.txt        instance list used by the Slurm array
#       build/script.slurm         Slurm array job (one task per instance, 10 runs per task)
#       build/log/                 Slurm output, one file per array task
#       data -> source/data
#       stats/<ALGORITHM>/<instance>/...   solver output (see docs/scripts.md)
#       configuration.tsv, source-commit.txt
#     submitted-jobs.tsv           job ids written by submit-all.sh
#     results/                     tables written by summarize-results.sh
#
# Every variable below can be overridden from the environment.
set -euo pipefail

hpc_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source_dir="${ECVRP_SOURCE:-$(cd -- "$hpc_dir/../.." && pwd)}"
root_dir="${ECVRP_ROOT:-$(dirname -- "$source_dir")}"
experiments_file="${ECVRP_EXPERIMENTS:-$hpc_dir/experiments.tsv}"
instances_file="${ECVRP_INSTANCES:-$hpc_dir/instances.txt}"
results_dir="${ECVRP_RESULTS:-$root_dir/results}"

# Arguments passed to every run: max-evals stopping criterion, 10 runs (seeds 1-10) in parallel, logging on.
common_args="${ECVRP_COMMON_ARGS:--stp 0 -mth 1 -log 1}"
runs_per_instance=10

# Slurm resources (QMUL Apocrita defaults; adjust for other clusters).
slurm_partition="${ECVRP_PARTITION:-compute}"
slurm_cpus="${ECVRP_CPUS:-10}"
slurm_mem_per_cpu="${ECVRP_MEM_PER_CPU:-1G}"
slurm_time_limit="${ECVRP_TIME_LIMIT:-04:00:00}"
slurm_extra="${ECVRP_SLURM_EXTRA:-}"            # e.g. "--account=myproject --qos=normal"
run_modules="${ECVRP_MODULES:-gcc}"              # modules loaded inside each job
build_modules="${ECVRP_BUILD_MODULES:-gcc cmake}" # modules loaded for the build

load_modules() {
    if type module >/dev/null 2>&1; then
        module purge
        # shellcheck disable=SC2086
        module load $1
    fi
}

# Instance names, without comments or blank lines.
instance_names() {
    sed 's/\r$//' "$instances_file" | grep -vE '^[[:space:]]*(#|$)' | sed 's/[[:space:]]*$//'
}

instance_stem() {
    local name="${1##*/}"
    printf '%s\n' "${name%.evrp}"
}

# Experiment ids in manifest order.
experiment_ids() {
    awk -F '\t' 'NR > 1 && $1 != "" && $1 !~ /^#/ { print $1 }' "$experiments_file"
}

# Prints "<algorithm>\t<arguments>" for one id.
experiment_row() {
    awk -F '\t' -v id="$1" '
        NR > 1 && $1 == id { print $2 "\t" $3; found = 1; exit }
        END { if (!found) exit 1 }
    ' "$experiments_file"
}

# Output folder name used by the solver (stats/<ALGORITHM>/...).
algorithm_folder() {
    printf '%s\n' "${1^^}"
}

validate_inputs() {
    [[ -f "$experiments_file" ]] || { echo "Error: missing $experiments_file" >&2; return 1; }
    [[ -f "$instances_file" ]] || { echo "Error: missing $instances_file" >&2; return 1; }

    local count
    count="$(instance_names | wc -l | tr -d ' ')"
    (( count > 0 )) || { echo "Error: no instances in $instances_file" >&2; return 1; }

    local duplicates
    duplicates="$(experiment_ids | sort | uniq -d)"
    [[ -z "$duplicates" ]] || { echo "Error: duplicate experiment ids: $duplicates" >&2; return 1; }

    local id algorithm arguments
    while IFS= read -r id; do
        IFS=$'\t' read -r algorithm arguments <<< "$(experiment_row "$id")"
        case "${algorithm,,}" in
            lahc|ma|twostagealns) ;;
            *) echo "Error: experiment $id has unknown algorithm '$algorithm' (lahc, ma, twostagealns)" >&2; return 1 ;;
        esac
    done < <(experiment_ids)

    local name missing=0
    while IFS= read -r name; do
        if [[ -z "$(find "$source_dir/data" -name "$name" -print -quit)" ]]; then
            echo "Error: instance not found under $source_dir/data: $name" >&2
            missing=1
        fi
    done < <(instance_names)
    return "$missing"
}

# Array indices (0-based) of the instances whose 10 runs are not all finished for one experiment.
missing_indices() {
    local id="$1" algorithm folder index=0 name stem seed complete
    IFS=$'\t' read -r algorithm _ <<< "$(experiment_row "$id")"
    folder="$(algorithm_folder "$algorithm")"
    while IFS= read -r name; do
        stem="$(instance_stem "$name")"
        complete=1
        [[ -f "$root_dir/$id/stats/$folder/$stem/stats.$stem.evrp" ]] || complete=0
        for seed in $(seq 1 "$runs_per_instance"); do
            [[ -f "$root_dir/$id/stats/$folder/$stem/$seed/solution.$stem.txt" ]] || complete=0
        done
        (( complete )) || printf '%d\n' "$index"
        index=$((index + 1))
    done < <(instance_names)
}

# Compresses "0 1 2 5 7 8" into "0-2,5,7-8" (sbatch --array syntax).
compress_indices() {
    awk '
        { v[NR] = $1 }
        END {
            out = ""
            i = 1
            while (i <= NR) {
                j = i
                while (j < NR && v[j + 1] == v[j] + 1) j++
                part = (i == j) ? v[i] : v[i] "-" v[j]
                out = (out == "") ? part : out "," part
                i = j + 1
            }
            print out
        }
    '
}
