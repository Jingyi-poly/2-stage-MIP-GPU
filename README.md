# Stochastic pickup and delivery with time windows

The Amdahl memetic solver handles the pickup-and-delivery problem with time
windows and stochastic demand (PDPTW-SD). It uses crossover, local improvement,
and large-neighborhood search. Native CPU and CUDA backends evaluate load
violations along fixed routes. The score adds distance, time-window violations,
and the mean capacity-violation penalty over demand scenarios. The optional
SCIP benchmark solves a finite-scenario model with hard time windows and
penalized overloads.

## Build and run on CPU

Use Linux or WSL, CMake 3.18 or later, a C++17 compiler, and OpenMP.

~~~sh
cmake -S src -B build-cpu -DCMAKE_BUILD_TYPE=Release
cmake --build build-cpu --parallel
OMP_NUM_THREADS=2 ./build-cpu/matrix_model_eval_test LiLim LC1_2_1 8
mkdir -p amdahl_sol_data
./build-cpu/amdahl src/config.txt
~~~

The parity check evaluates a reference route plan and compares the accumulated
distance, capacity violation, and time-window violation with a separate matrix
evaluation. With `src/config.txt`, the solver reads Li & Lim instance
`LC1_2_1` and writes solution records within `amdahl_sol_data`. Use
`resultFolder` in the configuration to select another writable folder relative
to the working directory.

## CUDA and SCIP

~~~sh
cmake -S src -B build-cuda -DCMAKE_BUILD_TYPE=Release \
  -DUSE_CUDA=ON -DCUDA_ARCH=75
cmake --build build-cuda --parallel
OMP_NUM_THREADS=2 ./build-cuda/matrix_model_eval_test LiLim LC1_2_1 8
~~~

Set `CUDA_ARCH` to your GPU's compute capability without the decimal point.
The standalone solver selects CUDA capacity evaluation with `useGpu=1` in its
configuration. The parity tool reports a separate GPU result when the CUDA
evaluator is active.

For the SCIP comparison, install SCIP's C headers, shared library, and
runtime dependencies. Supply `CMAKE_PREFIX_PATH` for a custom installation.

~~~sh
cmake -S src -B build-scip -DCMAKE_BUILD_TYPE=Release -DUSE_SCIP=ON
cmake --build build-scip --parallel --target pdptw_scip_benchmark
mkdir -p results
./build-scip/pdptw_scip_benchmark --instance lc202 --requests 3 --routes 2 \
  --scenarios 3 --seed 42 --time-limit 2 --warm-start off \
  --trace results/scip-smoke.csv
~~~

The SCIP trace records the feasible incumbent and dual bound by elapsed time.
The executable accepts `--help` for the full list of options. Its scenario
count includes the nominal slot.

## Benchmarks and data

| File or directory | Use |
| --- | --- |
| `src/domain/eval/` | CPU, CUDA, and full-route matrix evaluators |
| `src/domain/solver/memetic/` | Population search and route neighborhoods |
| `src/paper_experiments.cc` | Seeded search and held-out scoring experiments |
| `src/scenario_eval_benchmark.cc`, `src/gpu_memory_benchmark.cc` | Evaluation throughput and GPU workspace measurements |
| `src/pdptw_scip_benchmark.cc` | Small finite-scenario SCIP reference model |
| `src/data/instance/`, `src/data/bks/` | Instances and best-known or warm-start routes |
| `experiments/pdptw_iclr/` | Five-experiment pilot runner and plotter |

The pilot runner can collect a single experiment into a fresh local directory.
Use Python 3.10 or later and install Matplotlib for plotting.

~~~sh
python3 experiments/pdptw_iclr/run_pdptw_pilot.py --only oos \
  --bin-dir build-cpu --data-dir results/pdptw/data --raw-dir results/pdptw/raw
python3 experiments/pdptw_iclr/plot_pdptw_experiments.py --only oos \
  --data-dir results/pdptw/data --output-dir results/pdptw/figures
~~~

The standard stochastic evaluation grid uses one nominal slot plus the
reported number of random draws. The finite-scenario SCIP pilot uses its own
smaller grid. The pilot commands generate new data. See
[source provenance](docs/provenance.md) for the relationship between this
branch and the later manuscript figures.

The [Li & Lim benchmark](https://www.sintef.no/projectweb/top/pdptw/li-lim-benchmark/)
provides the instance family and reference routes. The vendored nlohmann JSON
header retains its upstream attribution and [MIT license](src/third_party/nlohmann/LICENSE.MIT).
