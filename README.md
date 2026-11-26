# Two-stage stochastic routing with CPU and GPU recourse evaluation

This repository provides five solver methods for four stochastic routing problems. Each method links to its source and build instructions. The paper branches preserve the versions cited in [From Sequential to Parallel (ICLR 2026)](https://arxiv.org/abs/2602.05179).

| Method | Implementation | ICLR 2026 snapshot |
| --- | --- | --- |
| VRPSD-HGS | Shared CPU/CUDA Split and HGS on [solver/vrpsd-pctsp](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/solver/vrpsd-pctsp#cpu-quick-start) | [CVRPSD-split-GPU](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/CVRPSD-split-GPU) |
| PCTSP-HGS | Optional-visit HGS (`-optionalVisit 1`) on [solver/vrpsd-pctsp](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/solver/vrpsd-pctsp#cpu-quick-start) | — |
| PCTSP-exact | SCIP level-synchronous branch-and-bound with CPU and CUDA Split evaluators on [solver/pctsp-exact](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/solver/pctsp-exact) | — |
| PDPTW | Memetic search with a CPU/CUDA stochastic capacity evaluator on [solver/pdptw](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/solver/pdptw) | — |
| DSIRP-HGS | Order-up-to inventory routing on [solver/dsirp-hgs](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/solver/dsirp-hgs) | [OUIRP-GPU](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/OUIRP-GPU) |

## Build from source

### VRPSD CPU example

```sh
git clone --branch solver/vrpsd-pctsp --single-branch https://github.com/Jingyi-poly/2-stage-MIP-GPU.git split-routing
cd split-routing
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
mkdir -p results
./build/hgs Instances/CVRP/X-n106-k14.vrp results/vrpsd.sol -seed 1 -nthreads 2 -nextrascen 7 -iterLim 1
```

Run PCTSP-HGS from the same build by enabling optional visits.

```sh
./build/hgs Instances/CVRP/X-n106-k14.vrp results/pctsp-hgs.sol -seed 1 -nthreads 2 -nextrascen 7 -iterLim 1 -optionalVisit 1
```

### PCTSP-exact and PDPTW

Use the branch quick starts to build and run [PCTSP-exact with SCIP](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/solver/pctsp-exact) and [PDPTW with its bundled example](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/solver/pdptw).

### DSIRP CPU example

```sh
git clone --branch solver/dsirp-hgs --single-branch https://github.com/Jingyi-poly/2-stage-MIP-GPU.git dsirp-hgs
cd dsirp-hgs
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/irp Data/Small/Istanze0105h3/0.dat -type 38 -veh 1 -stock 100 -seed 989 -demandseed 989 -scenario 2 -iter 1 -threads 1
```

The solver READMEs describe optional dependencies, accelerator builds, and input formats. Use the pinned versions in the [reproduction and archive index](docs/reproduction.md) to reproduce published results and retrieve historical files.
