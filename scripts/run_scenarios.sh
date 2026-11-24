#!/usr/bin/env bash
set -euo pipefail

if (( $# < 3 )); then
    echo "Usage: $0 BINARY INSTANCE SCENARIOS [SCENARIOS...]" >&2
    exit 2
fi

binary=$1
instance=$2
shift 2

output_dir=${OUTPUT_DIR:-results/local-sweep}
seed=${SEED:-989}
mkdir -p "$output_dir"

for scenarios in "$@"; do
    "$binary" "$instance" -type 38 -veh "${VEHICLES:-1}" -stock "${STOCK:-100}" \
        -seed "$seed" -demandseed "$seed" -scenario "$scenarios" \
        -iter "${ITERATIONS:-1}" -threads "${THREADS:-1}" \
        -sol "$output_dir/scenarios-$scenarios.sol"
done
