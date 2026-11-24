#!/usr/bin/env bash
#SBATCH --job-name=dsirp
#SBATCH --cpus-per-task=4
set -euo pipefail

if (( $# < 3 )); then
    echo "Usage: sbatch --array=0-N $0 BINARY INSTANCE SCENARIO0 [SCENARIO1...]" >&2
    exit 2
fi

binary=$1
instance=$2
shift 2
scenarios=("$@")
task=${SLURM_ARRAY_TASK_ID:?Submit as a zero-based SLURM job array.}
if (( task >= ${#scenarios[@]} )); then
    echo "SLURM task $task has no assigned scenario count." >&2
    exit 2
fi

cd -- "${SLURM_SUBMIT_DIR:?Submit from the repository root.}"
export THREADS=${THREADS:-${SLURM_CPUS_PER_TASK:-1}}
export OUTPUT_DIR=${OUTPUT_DIR:-results/slurm/${SLURM_ARRAY_JOB_ID:?}/$task}
exec bash scripts/run_scenarios.sh "$binary" "$instance" "${scenarios[$task]}"
