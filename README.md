# VRPSD and PCTSP Split solvers

This branch maintains the CPU and CUDA Split evaluators for vehicle routing with stochastic demands and the optional-visit prize-collecting variant. The search is based on [HGS-CVRP](https://github.com/vidalt/HGS-CVRP). For the ICLR experiments, use the [paper snapshot](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/5ac5cb954d928bbc0407a5a9ffb1ed25ae5f14a9) and the [paper](https://arxiv.org/abs/2602.05179v3).

## CPU quick start

Use CMake 3.18 or later and a C++17 compiler with OpenMP, such as GCC on Linux or WSL. CTest also uses a C compiler. Run the commands from the repository root.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
mkdir -p results
./build/hgs Instances/CVRP/X-n106-k14.vrp results/vrpsd.sol \
    -seed 1 -nthreads 2 -nextrascen 7 -iterLim 1
./build/hgs Instances/CVRP/X-n106-k14.vrp results/pctsp-hgs.sol \
    -seed 1 -nthreads 2 -nextrascen 7 -iterLim 1 -optionalVisit 1
```

The solver writes nominal routes and the reported penalized objective to the specified solution file. For optional visits, the routes list selected customers only. The objective averages the nominal demand scenario and the additional generated scenarios. The solver reports run progress on standard output.

| Option | Use |
| --- | --- |
| `-nextrascen N` | Generate `N` additional scenarios, for `N+1` scenarios total. |
| `-seed S`, `-nthreads T` | Set the seed and the number of CPU threads. |
| `-iterLim I`, `-timeLim S` | Set the iteration limit or a wall-clock budget in seconds. |
| `-maxClient C` | Limit the instance to its first `C` customers. |
| `-optionalVisit 1`, `-skipPenScale X` | Allow customer omission and set the skip-penalty multiplier. |

## Score a fixed visit order

`cross_validate` takes a plain-text permutation of all customers in the instance. It reports the capacity-only CPU Split objective and a quadratic reference on the same seeded scenarios.

```sh
./build/cross_validate Instances/CVRP/X-n106-k14.vrp \
    Instances/CVRP/X-n106-k14.vrp.cvrp_chromT.txt -seed 1 -nextrascen 7 -nthreads 2
```

## Optional CUDA build

Install a CUDA toolkit and a compatible NVIDIA driver. Supply the CUDA architecture for your GPU. The example uses compute capability 7.5.

```sh
cmake -S . -B build_cuda -DCMAKE_BUILD_TYPE=Release -DENABLE_CUDA=ON \
    -DCMAKE_CUDA_ARCHITECTURES=75
cmake --build build_cuda --parallel
ctest --test-dir build_cuda --output-on-failure
mkdir -p results
./build_cuda/hgs_cuda Instances/CVRP/X-n106-k14.vrp results/vrpsd-cuda.sol \
    -seed 1 -nthreads 2 -nextrascen 7 -iterLim 1 -gpuBatchSize 1
```

CTest checks the CPU and CUDA objectives on the same fixed tour and eight scenarios. This check passed with GCC 13 and CUDA 12.6 on an RTX 2080 Ti. Use `-DCMAKE_CUDA_COMPILER=/path/to/nvcc` when the toolkit is installed outside the usual search paths.

## Optional Gurobi baselines

Set `GUROBI_HOME` to the Gurobi SDK directory containing `include/` and `lib/`. The binaries require a license at runtime and print their results to standard output.

```sh
cmake -S . -B build_gurobi -DCMAKE_BUILD_TYPE=Release -DENABLE_GUROBI=ON \
    -DGUROBI_HOME="$GUROBI_HOME"
cmake --build build_gurobi --parallel
mkdir -p results
./build_gurobi/pctsp_exact Instances/CVRP/X-n106-k14.vrp results/gurobi.sol \
    -seed 1 -nthreads 1 -maxClient 6 -nextrascen 7 -timeLim 25 -optionalVisit 0
```

| Executable | Mode |
| --- | --- |
| `pctsp_exact` | With `-optionalVisit 0`, solves the all-visit stochastic MILP. With `-optionalVisit 1`, solves a deterministic optional-visit model, then improves it with stochastic local search. |
| `pctsp_exact_cuda` | Built when both options are enabled. Uses GPU Split in the all-visit Benders path. |
| `cvrp_exact` | Separate two-stage CVRP Gurobi baseline in `pctsp_exact/`. Configure it with `cmake -S pctsp_exact -B build_cvrp_exact -DGUROBI_HOME="$GUROBI_HOME"` and build that directory. |

The Gurobi targets build with Gurobi 12.0.1. The runtime check requires a configured license.

## Benchmarks and project files

- `Program/` contains the split operators and solver entry points. `Test/` contains CTest checks.
- `Instances/CVRP/` contains benchmark inputs and fixed permutations used for scoring. `pctsp/` contains the Python PCTSP baselines and their input instances. `baseline/` and `exact_baseline.py` provide additional CVRP baseline tools.
- `scripts/bench_split.sh BASELINE_GPU OPTIMIZED_GPU [REPEATS] [OUTPUT_DIR]` runs matching seeded, single-candidate CUDA timings and saves the per-run logs and CSV. The baseline is [d29b135](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/d29b135fe95f27ca61e2454c926ec2aa275f036b), immediately before the batched optimization. Pass each revision's `hgs_cuda` executable path.
- The [original development snapshot](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/8f7f7cd5ba6881ad11c9739ac3c7f71e18541fef) preserves earlier analysis scripts, figures, and outputs. Keep new runs in `results/` and local build directories.

See [LICENSE](LICENSE) for the HGS-CVRP MIT notice.
