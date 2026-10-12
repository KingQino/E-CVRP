#!/usr/bin/env bash
# Builds the Release solver in the cloned repository (run on the login node).
set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/common.sh"

load_modules "$build_modules"
cmake -S "$source_dir" -B "$source_dir/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$source_dir/build" --target Run -j "${ECVRP_BUILD_JOBS:-8}"
echo "Built $source_dir/build/Run"
