#!/usr/bin/env bash
# Collects the final objective of every run into tables under $ECVRP_ROOT/results/.
# Works on partial results: unfinished instances are reported and marked NA.
#
#   runs-all.tsv       experiment, algorithm, dataset, instance, seed, objective (one row per run)
#   objective-all.tsv  experiment, algorithm, instance, runs, min, mean, std, max
#   objective-wide.tsv instance, then <id>_min / <id>_mean / <id>_std per experiment
#   objective-legacy-<id>.txt  per experiment, in instances.txt order: three lines per instance
#                      (min, mean, std) with two decimals, NA if not all runs finished
#   ma-search-budget.tsv  MA only: evaluation split (ordinary LS, Elite Unrestricted, follower, other) per instance
#
# The objective is the first line of stats/<ALG>/<instance>/<seed>/solution.<instance>.txt (5 decimals).
set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/common.sh"

mkdir -p "$results_dir"
runs_all="$results_dir/runs-all.tsv"
objective_all="$results_dir/objective-all.tsv"
objective_wide="$results_dir/objective-wide.tsv"
budget_all="$results_dir/ma-search-budget.tsv"

dataset_of() {
    local path
    path="$(find "$source_dir/data" -name "$1" -print -quit)"
    basename "$(dirname "$path")"
}

printf 'experiment\talgorithm\tdataset\tinstance\tseed\tobjective\n' > "$runs_all"
printf 'experiment\tinstance\tnormal_ls_evals\telite_ls_evals\tfollower_evals\tother_evals\ttotal_evals\tnormal_ls_share\telite_ls_share\tfollower_share\tother_share\n' > "$budget_all"
incomplete=0
mapfile -t ids < <(experiment_ids)
mapfile -t cases < <(instance_names)
declare -A datasets
for name in "${cases[@]}"; do datasets["$name"]="$(dataset_of "$name")"; done

for id in "${ids[@]}"; do
    IFS=$'\t' read -r algorithm _ <<< "$(experiment_row "$id")"
    folder="$(algorithm_folder "$algorithm")"
    for name in "${cases[@]}"; do
        stem="$(instance_stem "$name")"
        budget_files=()
        for seed in $(seq 1 "$runs_per_instance"); do
            run_dir="$root_dir/$id/stats/$folder/$stem/$seed"
            solution="$run_dir/solution.$stem.txt"
            if [[ -f "$solution" ]]; then
                value="$(sed -n '1p' "$solution" | tr -d '\r')"
            else
                value="NA"
                incomplete=$((incomplete + 1))
            fi
            printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$id" "${algorithm,,}" "${datasets[$name]}" "$stem" "$seed" "$value" >> "$runs_all"
            [[ -f "$run_dir/search-budget.tsv" ]] && budget_files+=("$run_dir/search-budget.tsv")
        done
        if (( ${#budget_files[@]} > 0 )); then
            awk -F '\t' -v id="$id" -v inst="$stem" '
                FNR > 1 { n += $2; e += $3; f += $4; o += $5; t += $6 }
                END {
                    if (t > 0) printf "%s\t%s\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\n",
                        id, inst, n, e, f, o, t, n / t, e / t, f / t, o / t
                }
            ' "${budget_files[@]}" >> "$budget_all"
        fi
    done
done

# Per experiment and instance: runs, min, mean, sample std, max over the finished runs.
awk -F '\t' 'BEGIN { OFS = "\t"; print "experiment", "algorithm", "instance", "runs", "min", "mean", "std", "max" }
    NR > 1 {
        key = $1 FS $2 FS $4
        if (!(key in seen)) { seen[key] = 1; order[++size] = key }
        if ($6 == "NA") next
        # Welford update (stable when all runs are equal)
        n[key]++; d = $6 - mu[key]; mu[key] += d / n[key]; m2[key] += d * ($6 - mu[key])
        if (!(key in mn) || $6 < mn[key]) mn[key] = $6
        if (!(key in mx) || $6 > mx[key]) mx[key] = $6
    }
    END {
        for (i = 1; i <= size; ++i) {
            k = order[i]
            if (n[k] == 0) { print k, 0, "NA", "NA", "NA", "NA"; continue }
            var = n[k] > 1 ? m2[k] / (n[k] - 1) : 0
            if (var < 0) var = 0
            printf "%s\t%d\t%.5f\t%.5f\t%.5f\t%.5f\n", k, n[k], mn[k], mu[k], sqrt(var), mx[k]
        }
    }
' "$runs_all" > "$objective_all"

# Wide table: one row per instance, min/mean/std columns per experiment (NA if not all runs finished).
awk -F '\t' -v ids="$(printf '%s ' "${ids[@]}")" -v runs="$runs_per_instance" 'BEGIN { OFS = "\t"; m = split(ids, id, " ") }
    NR > 1 {
        if (!($3 in seen)) { seen[$3] = 1; order[++size] = $3 }
        complete = ($4 == runs)
        mn[$1, $3] = complete ? $5 : "NA"; me[$1, $3] = complete ? $6 : "NA"; sd[$1, $3] = complete ? $7 : "NA"
    }
    END {
        header = "instance"
        for (j = 1; j <= m; ++j) header = header OFS id[j] "_min" OFS id[j] "_mean" OFS id[j] "_std"
        print header
        for (i = 1; i <= size; ++i) {
            inst = order[i]; line = inst
            for (j = 1; j <= m; ++j) {
                k = id[j] SUBSEP inst
                line = line OFS ((k in mn) ? mn[k] : "NA") OFS ((k in me) ? me[k] : "NA") OFS ((k in sd) ? sd[k] : "NA")
            }
            print line
        }
    }
' "$objective_all" > "$objective_wide"

# Legacy format (as in CEVRP-with-Adaptive-Selection): min, mean, std per instance, one value per line.
legacy_files=()
for id in "${ids[@]}"; do
    legacy="$results_dir/objective-legacy-$id.txt"
    awk -F '\t' -v id="$id" -v runs="$runs_per_instance" '
        NR == FNR {
            if (FNR > 1 && $1 == id && $6 != "NA") {
                k = $4; n[k]++; d = $6 - mu[k]; mu[k] += d / n[k]; m2[k] += d * ($6 - mu[k])
                if (!(k in mn) || $6 < mn[k]) mn[k] = $6
            }
            next
        }
        {
            k = $0
            if (n[k] == runs) {
                printf "%.2f\n%.2f\n%.2f\n", mn[k], mu[k], (runs > 1 ? sqrt(m2[k] / (runs - 1)) : 0)
            } else {
                print "NA"; print "NA"; print "NA"
            }
        }
    ' "$runs_all" <(for name in "${cases[@]}"; do instance_stem "$name"; done) > "$legacy"
    legacy_files+=("$legacy")
done

echo "Wrote $runs_all"
echo "Wrote $objective_all"
echo "Wrote $objective_wide"
printf 'Wrote %s\n' "${legacy_files[@]}"
echo "Wrote $budget_all"
if (( incomplete > 0 )); then
    echo "Warning: $incomplete runs have no solution file yet (marked NA); see check-progress.sh" >&2
fi
