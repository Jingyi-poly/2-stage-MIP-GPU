# Dynamic stochastic inventory routing with HGS

This branch maintains the order-up-to hybrid genetic search solver for the dynamic stochastic inventory routing problem (DSIRP). The native C++ evaluator runs on CPU. The optional gRPC client sends inventory-reinsertion evaluations to the Python/PyTorch server.

## CPU quick start

Use a C++17 compiler and CMake 3.16 or newer on Linux or WSL.

~~~bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure

./build/irp Data/Small/Istanze0105h3/0.dat \
  -type 38 -veh 1 -stock 100 -seed 989 -demandseed 989 \
  -scenario 2 -iter 1 -threads 1
~~~

CTest runs the same instance with one and two scenarios. To reproduce a single-scenario run, change the command to `-scenario 1`. Positive `-threads` values select the native evaluator. By default, the solver uses the available CPU threads.

Solutions are appended to a per-run `STsol-…` file under `results/`. Timing statistics use the same filename with a `.timing.log` suffix. Supply `-sol PATH` to pick a path, or use a distinct path for each concurrent run. The `-bks PATH` argument sets the best-known-solution path carried by the instance parameters. The current executable exports the solution file described above.

## Optional GPU evaluator

The client build needs the Protobuf compiler and C++ libraries, the gRPC C++ libraries, and `grpc_cpp_plugin`. On Ubuntu these are provided by `libprotobuf-dev`, `protobuf-compiler`, `libgrpc++-dev`, and `protobuf-compiler-grpc`. CMake generates C++ stubs from `rpc/irp.proto` in the build directory.

~~~bash
cmake -S . -B build-grpc -DCMAKE_BUILD_TYPE=Release -DIRP_ENABLE_GRPC=ON
cmake --build build-grpc --parallel
~~~

In a Python environment with a CUDA-enabled PyTorch build, NumPy, and `grpcio-tools`, generate the Python stubs from the same schema and start the evaluator.

~~~bash
python -m grpc_tools.protoc -I rpc --python_out=rpc --grpc_python_out=rpc rpc/irp.proto
python rpc/server.py
~~~

From another terminal in the repository, select the gRPC client at `localhost:50051`.

~~~bash
./build-grpc/irp Data/Small/Istanze0105h3/0.dat \
  -type 38 -veh 1 -stock 100 -seed 989 -demandseed 989 \
  -scenario 2 -iter 1 -control_day_1 1 -threads 0 -sol results/rpc-smoke.sol
~~~

The PyTorch evaluator uses CUDA when `torch.cuda.is_available()` is true. The one- and two-scenario native CPU runs were verified on Ubuntu 24.04 in WSL. The gRPC/CUDA run requires a separate environment with the optional dependencies.

## Files and reproduction

- `Data/` contains the original benchmark instance inputs. The [data index](Data/README.md) identifies the smoke-test and 100-customer inputs.
- `Params_list/` holds alternate parameter source snapshots. `Params.cpp` supplies the build.
- `rpc/irp.proto` is the interface shared by the C++ client and the Python evaluator.
- `scripts/` contains local and SLURM runners. See the [runner documentation](scripts/README.md).

The [ICLR 2026 DSIRP implementation](https://github.com/Jingyi-poly/2-stage-MIP-GPU/tree/f3931bd798a2e82048f3744ea48c66405437d059) pins the upstream snapshot and contains the historical outputs, logs, job scripts, and experimental source copies. Its archived solutions have prefixes `STsol` and `STbks`, and prior timing artifacts include `cputime` records.

The later local 100-customer, six-day experiment uses the same `abs5n100_4.dat` input and extends this solver with a shared first-day route and replenishment representation, a later-stage neighborhood search, and a streaming evaluator.

If you use this implementation, cite [Zhao et al., *From Sequential to Parallel: Reformulating Dynamic Programming as GPU Kernels for Large-Scale Stochastic Combinatorial Optimization*, ICLR 2026](https://arxiv.org/abs/2602.05179).
