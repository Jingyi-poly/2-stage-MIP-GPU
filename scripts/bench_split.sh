#!/usr/bin/env bash
set -euo pipefail

if (( $# < 2 || $# > 4 )); then
    echo "Usage: $0 BASELINE_GPU OPTIMIZED_GPU [REPEATS=3] [OUTPUT_DIR=results/split-benchmark]" >&2
    exit 2
fi

baseline=$1
optimized=$2
repeats=${3:-3}
output=${4:-results/split-benchmark}
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

if [[ ! -x "$baseline" || ! -x "$optimized" || ! "$repeats" =~ ^[1-9][0-9]*$ ]]; then
    echo "Provide two executable paths and a positive repeat count." >&2
    exit 2
fi

mkdir -p "$output"
results=$(mktemp -d "$output/run.XXXXXX")
printf 'instance,scenarios,seed,mode,batch,total_seconds\n' > "$results/times.csv"

run_case() {
    local binary=$1 instance=$2 scenarios=$3 seed=$4 mode=$5
    shift 5
    local stem="$results/${instance}_${scenarios}_${seed}_${mode}"
    "$binary" "$root/Instances/CVRP/$instance" "$stem.sol" \
        -seed "$seed" -nthreads 4 -nextrascen "$((scenarios-1))" \
        -freqPrint 999999 -iterLim 16 "$@" > "$stem.log" 2>&1
    local seconds
    seconds=$(awk '/^Total time:[[:space:]]*[0-9]/{ t = $3; sub(/s$/, "", t) } END { if (t == "") exit 1; print t }' "$stem.log")
    printf '%s,%s,%s,%s,1,%s\n' "$instance" "$scenarios" "$seed" "$mode" "$seconds" >> "$results/times.csv"
}

for instance in X-n106-k14.vrp X-n143-k7.vrp; do
    for scenarios in 1000 10000; do
        for ((seed=1; seed<=repeats; seed++)); do
            run_case "$baseline" "$instance" "$scenarios" "$seed" baseline
            run_case "$optimized" "$instance" "$scenarios" "$seed" optimized -gpuBatchSize 1
        done
    done
done
printf 'Benchmark results in %s\n' "$results"
