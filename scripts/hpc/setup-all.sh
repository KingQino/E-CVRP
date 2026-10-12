#!/usr/bin/env bash
# Builds the solver (skip with ECVRP_SKIP_BUILD=1) and prepares every experiment in experiments.tsv.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/common.sh"
validate_inputs

if [[ "${ECVRP_SKIP_BUILD:-0}" != "1" ]]; then
    "$script_dir/build-source.sh"
fi
while IFS= read -r id; do
    "$script_dir/setup-one.sh" "$id"
done < <(experiment_ids)
echo "Prepared all experiments under $root_dir. Submit them with submit-all.sh."
