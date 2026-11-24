# Running scenario sweeps

Run the examples from the repository root after building the executable as described in the [solver README](../README.md). Both runners accept an executable, an instance, and a list of scenario counts. The examples keep the seed, fleet size, stockout setting, and iteration budget fixed across scenario counts.

For a sequential run on the local CPU:

~~~bash
bash scripts/run_scenarios.sh ./build/irp Data/Small/Istanze0105h3/0.dat 1 2 4
~~~

For a SLURM array with one task per scenario count:

~~~bash
mkdir -p results/slurm
sbatch --array=0-2 --output=results/slurm/%A_%a.out \
  scripts/slurm/run_scenarios.sbatch ./build/irp Data/Small/Istanze0105h3/0.dat 1 2 4
~~~

Set `THREADS`, `VEHICLES`, `STOCK`, `SEED`, or `ITERATIONS` in the environment to override the defaults. On SLURM the default thread count follows `SLURM_CPUS_PER_TASK`. The SLURM example uses cluster-supplied resources, with scenario size set by its array index. For simultaneous local sweeps, set a distinct `OUTPUT_DIR` per invocation.
