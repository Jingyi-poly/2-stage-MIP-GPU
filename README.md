# PCTSP-exact with level-synchronous branch-and-bound

This branch implements the optional-visit, finite-scenario stochastic
prize-collecting routing model. SCIP solves node LP relaxations in CPU
worker waves. The master batches integral tours at a level barrier, evaluates
their Split recourse with OpenMP or CUDA, and adds integer L-shaped cuts.
The bounds in the trace are certified at completed barriers.

The objective adds the skip penalties of omitted customers to the scenario
mean of capacity-penalized Split cost. Split allows an unrestricted number
of routes. An optimality certificate applies to this finite scenario model and
the recorded numerical tolerances. The solution file gives the selected
customers and their visit order. Use the [VRPSD and PCTSP-HGS branch](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/solver/vrpsd-pctsp)
for heuristic search.

## Build and verify

The CPU build uses CMake 3.18+, a C++17 compiler with OpenMP, and the
[SCIP Optimization Suite](https://www.scipopt.org/) CMake package.
Set `-DSCIP_DIR=/path/to/lib/cmake/scip` if CMake needs the package location.

```sh
git clone --branch solver/pctsp-exact --single-branch https://github.com/Jingyi-poly/2-stage-MIP-GPU.git
cd 2-stage-MIP-GPU
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
mkdir -p results
./build/pctsp_level_exact Test/tiny-n5-k2.vrp results/pctsp-level.sol \
    -scenarioSeed 11 -penaltySeed 1729 -algoSeed 1 -nextrascen 3 \
    -optionalVisit 1 -skipPenScale 1.2 -timeLim 30 -nthreads 2 \
    -gpuBatchSize 8 -traceFile results/pctsp-level.csv
```

The tests compare both SCIP solvers against enumeration on a small instance.
`-nextrascen K` adds `K` stochastic draws to one base slot. In the level
solver, `-nthreads` sets the number of CPU node workers and `-gpuBatchSize`
sets the number of tours in an evaluation batch, on either backend.

### CUDA recourse

Install a CUDA toolkit and driver. Set the CUDA architecture to match the
GPU. The following example targets a Turing GPU.

```sh
cmake -S . -B build_cuda -DCMAKE_BUILD_TYPE=Release -DENABLE_CUDA=ON \
    -DCMAKE_CUDA_ARCHITECTURES=75 -DENABLE_GPU_TESTS=ON
cmake --build build_cuda --parallel
ctest --test-dir build_cuda -R 'fixed_tour_cpu_cuda|pctsp_level_exact_cuda_tiny' --output-on-failure
```

`pctsp_level_exact_cuda` uses the same SCIP algorithm and evaluates tours
with CUDA Split. Use the CPU example's options plus `-deviceId 0`. For
comparison, `pctsp_scip_outer` and `pctsp_scip_outer_cuda` keep SCIP's own
outer search. The optional `ENABLE_GUROBI` targets `pctsp_exact` and
`pctsp_exact_cuda` refer to the legacy baselines.

## Manuscript scenario-count experiment

[Figure 11 protocol and reference values](reproduction/figure-11/README.md)
record the weighted CPU configuration, fixed skip penalties, and evaluation
settings for the first six customers of `X-n101-k25`. The corresponding
source adds lossless aggregation of identical scenario vectors to CPU
tour scoring. The search checks the weighted score against uncompressed
evaluation on a reference tour before branching. Other demand models use
the uncompressed OpenMP oracle.

The Split/HGS code retains the upstream MIT notice in [LICENSE](LICENSE).
