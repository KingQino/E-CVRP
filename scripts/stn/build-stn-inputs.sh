#!/usr/bin/env bash
# Builds the STN input files from the raw logs of the STN experiments (B-LAHC and BMA).
# Run on the cluster from the experiment root (the directory containing source/), or on your computer
# from a downloaded copy of it.
#
# Usage: ./source/scripts/stn/build-stn-inputs.sh [experiment root]
#
# Reads    <root>/<LAHC id>/stats/LAHC/<instance>/<seed>/stn_raw.*.csv and <root>/<MA id>/stats/MA/...
# Writes   <root>/stn/stn-web/<instance>/{BLAHC,BMA}_<instance>.csv   STNs-v2 / STN Web input (best-so-far)
#          <root>/stn/stn-v1/<instance>/{BLAHC,BMA}_<instance>.csv    STNs v1 input (edge lists)
#          <root>/stn/trajectory-metrics.csv                            per-algorithm convergence metrics
#          <root>/stn/checks.txt                                        signature and consistency checks
#
# Optional environment variables:
#   ECVRP_STN_LAHC_ID   experiment id of B-LAHC in experiments.tsv (default BLAHC)
#   ECVRP_STN_MA_ID     experiment id of BMA (default BMA)
#   ECVRP_INSTANCES     instance list (default scripts/stn/instances.txt)
#   ECVRP_STN_INTERVAL  checkpoint interval as a fraction of the budget (default 0.001)
#   ECVRP_STN_RUNS      number of runs (default 10)
set -euo pipefail

stn_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source_dir="$(cd -- "$stn_dir/../.." && pwd)"
root_dir="${1:-$(dirname -- "$source_dir")}"
lahc_id="${ECVRP_STN_LAHC_ID:-BLAHC}"
ma_id="${ECVRP_STN_MA_ID:-BMA}"
instances_file="${ECVRP_INSTANCES:-$stn_dir/instances.txt}"
interval="${ECVRP_STN_INTERVAL:-0.001}"
runs="${ECVRP_STN_RUNS:-10}"
out_dir="$root_dir/stn"
python="${PYTHON:-python3}"

mkdir -p "$out_dir/stn-web" "$out_dir/stn-v1"
: > "$out_dir/checks.txt"
done_instances=()

while IFS= read -r name; do
    stem="${name%.evrp}"
    complete=1
    for seed in $(seq 1 "$runs"); do
        for path in "$root_dir/$lahc_id/stats/LAHC/$stem/$seed" "$root_dir/$ma_id/stats/MA/$stem/$seed"; do
            [[ -f "$path/stn_raw.$stem.csv" && -f "$path/solution.$stem.txt" ]] || complete=0
        done
    done
    if (( ! complete )); then
        echo "skip $stem: not all $runs runs of $lahc_id and $ma_id are finished" >&2
        continue
    fi

    "$python" "$stn_dir/make_stn_input.py" --stats "$root_dir" --instance "$stem" --runs "$runs" \
        --alg "$lahc_id/stats/LAHC=BLAHC" --alg "$ma_id/stats/MA=BMA" \
        --interval "$interval" --best-so-far --collapse --out "$out_dir/stn-web" > /dev/null
    "$python" "$stn_dir/stn_v2_to_v1.py" "$out_dir/stn-web/$stem" "$out_dir/stn-v1/$stem" > /dev/null
    {
        echo "== $stem"
        "$python" "$stn_dir/check_stn_logs.py" --stats "$root_dir" --instance "$stem" --runs "$runs" \
            --alg "$lahc_id/stats/LAHC" --alg "$ma_id/stats/MA"
    } >> "$out_dir/checks.txt"
    done_instances+=("$stem")
    echo "ok $stem"
done < <(sed 's/\r$//' "$instances_file" | grep -vE '^[[:space:]]*(#|$)' | sed 's/[[:space:]]*$//')

if (( ${#done_instances[@]} > 0 )); then
    "$python" "$stn_dir/trajectory_metrics.py" --stats "$root_dir" --runs "$runs" \
        --alg "$lahc_id/stats/LAHC=BLAHC" --alg "$ma_id/stats/MA=BMA" "${done_instances[@]}" \
        > "$out_dir/trajectory-metrics.csv"
fi
echo "STN inputs in $out_dir"
