# Reproduction and archive index

The ICLR 2026 paper links to the two branches below. Their tips identify the original implementations. Use the annotated paper tags to pin these versions in scripts.

| Application | Paper branch | Tag | Commit |
| --- | --- | --- | --- |
| CVRPSD | [CVRPSD-split-GPU](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/CVRPSD-split-GPU) | `paper/iclr2026-cvrpsd` | `5ac5cb954d928bbc0407a5a9ffb1ed25ae5f14a9` |
| DSIRP | [OUIRP-GPU](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/OUIRP-GPU) | `paper/iclr2026-dsirp` | `f3931bd798a2e82048f3744ea48c66405437d059` |

Check out a tag in a separate clone. The snapshot contains its original documentation, instances, and reference outputs.

```sh
git clone https://github.com/Jingyi-poly/2-stage-MIP-GPU.git paper-reproduction
git -C paper-reproduction switch --detach paper/iclr2026-cvrpsd
```

The separate 100-customer, six-day DSIRP scenario-count experiment starts from the OUIRP implementation. It uses an adapted CPU HGS that shares the first-day route and delivery quantities across scenarios, followed by neighborhood search and a streaming evaluator at larger scenario counts. These extensions identify a separate experiment from the tagged paper implementation.

## Solver branches for the extended results

| Method | Source and quick start |
| --- | --- |
| VRPSD-HGS and PCTSP-HGS | [CPU/CUDA Split and HGS](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/solver/vrpsd-pctsp). The PCTSP variant enables optional visits. |
| PCTSP-exact | [SCIP level-synchronous branch-and-bound](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/solver/pctsp-exact). The branch documents CPU and CUDA build targets and the exact solver checks. |
| PDPTW | [Memetic PDPTW solver](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/solver/pdptw). The branch documents the stochastic capacity objective, CPU and CUDA evaluators, and Li–Lim examples. |
| DSIRP-HGS | [Order-up-to inventory routing](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/solver/dsirp-hgs). The branch documents the standalone CPU build and GPU RPC setup. |

Each branch documents the inputs needed for its build and examples. To reproduce a particular result, pin the solver revision along with the instance, scenario generator and parameters, seeds, and run budget used for that result.

## Former branches

| Former branch content | Tag | Commit | Contents to consult |
| --- | --- | --- | --- |
| Modular inventory-routing `main` | `archive/main-irp-modular-9f7c90e` | `9f7c90eba1e14f5bb7ae9a7359dfdf6311dedd36` | Enhanced demand generator, modular C++ source and RPC scripts. The snapshot's CMake file refers to an untracked `include/irp/` tree. |
| `Split_Perf` | `archive/split-perf-8f7f7cd` | `8f7f7cd5ba6881ad11c9739ac3c7f71e18541fef` | Original Split/PCTSP source, CUDA builds, benchmark outputs, and Gurobi experiments. |
| `working` | `archive/working-d29b135` | `d29b135fe95f27ca61e2454c926ec2aa275f036b` | Transitional Split and PCTSP implementation. This commit is an ancestor of `Split_Perf`. |
| `IRP-OU-CPU-SingleScenario` | `archive/irp-ou-cpu-single-scenario-ae12562` | `ae12562a9eea5eb14036ea0deb0e9c3e500066a3` | Vectorized single-scenario DP in `MatrixSolver.cpp` and `MatrixSolver.h`, along with the Makefile and reference instance. |

The single-scenario snapshot documents `make` followed by `./irp Data/Small/0.dat -seed 1000 -type 38 -veh 1 -stock 200`. Its recorded objectives are 3599.1789 for `-stock 200` and 3277.3938 for `-stock 100`.

## Files used in maintained builds

| Content | Maintained location | Historical location |
| --- | --- | --- |
| Split source, tests, and instances | `Program/`, `Test/`, and `Instances/` on `solver/vrpsd-pctsp`. `scripts/bench_split.sh` compares two supplied executables. | `archive/split-perf-8f7f7cd` contains `anpy/`, the prior `build_cuda/` tree, and the original PCTSP result CSVs. |
| DSIRP source and experiment inputs | The source, 936 instance files under `Data/`, the RPC protocol, and local and SLURM runners on `solver/dsirp-hgs`. New solution and timing files go to `results/` by default. | `paper/iclr2026-dsirp` contains the historical `STsol` and `STbks` snapshots, timing logs, and original cluster scripts. |
| Modular IRP and single-scenario reference source | The snapshots linked above contain the enhanced demand generator and the single-scenario `MatrixSolver`, respectively. | `archive/main-irp-modular-9f7c90e` and `archive/irp-ou-cpu-single-scenario-ae12562`. |

The solver branches retain the original instance filenames. Each solver's README records the input formats, dataset references, and dependency requirements.
