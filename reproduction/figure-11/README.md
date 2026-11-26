# Manuscript PCTSP-exact scenario-count experiment

The data in this directory pin the six-client CPU level-B&B experiment
behind Figure 11. [Training protocol](training_protocol.json) records the
source and input hashes, seeds, grid, and numerical settings.
[Training audit](training_audit.json) records 35 certificates. The solver
and evaluator hashes identify the original binaries.

## One training cell

From the repository root after the CPU build, use the first six customers of
`X-n101-k25` and the [fixed skip penalties](fixed_penalties_6.csv). The
example runs seed 580001 at `N=100` and independently scores its certificate
on that training prefix. The weighted solver reports scenario multiplicities,
weighted/uncompressed parity, and a numerical bound gap.

```sh
mkdir -p results
export PCTSP_SKIP_PENALTIES_FILE="$PWD/reproduction/figure-11/fixed_penalties_6.csv"
./build/pctsp_level_exact Instances/CVRP/X-n101-k25.vrp results/figure-11.sol \
    -scenarioSeed 580001 -algoSeed 680001 -nextrascen 99 -maxClient 6 \
    -optionalVisit 1 -penaltySeed 1729 -skipPenScale 0.6603201305112942 \
    -gpuBatchSize 4 -nthreads 4 -timeLim 600 -log 0 \
    -traceFile results/figure-11.csv -traceEvery 1 -fixedRouteCapacity 1 \
    -demandDistribution stress -demandStressProb 0.00001 \
    -demandStressScale 40000 -demandCorrelation 0.8
./build/cross_validate Instances/CVRP/X-n101-k25.vrp results/figure-11.sol \
    -nextrascen 99 -scenarioSeed 580001 -seed 580001 -maxClient 6 \
    -penaltySeed 1729 -skipPenScale 0.6603201305112942 -nthreads 4 \
    -fixedRouteCapacity 1 -skipNominal 0 -demandDistribution stress \
    -demandStressProb 0.00001 -demandStressScale 40000 -demandCorrelation 0.8
```

The training design uses seeds 580001–580005, algorithm seed
`scenarioSeed + 100000`, and `-nextrascen N-1` for every grid point in the
protocol. Independently score each resulting policy on its seed's complete
clean pool, retain the best policy through each `N`, and freeze those
policies before applying perturbations to the test copy.

[Figure summary](figure_summary.csv) records both the clean historical
mean and the plotted mean from frozen policies. The latter used a separate
noise-overlay scoring executable, with row selection by the smallest 10,000
`(splitmix64(780001 + row), row)` pairs, using zero-based rows 0 through
999999. For a selected row, the multiplier of each zero-based component
`j` was `0.99 + 0.02 * ((splitmix64(1780001 + 1000 * row + j) >> 11) * 2^-53)`.
The command above verifies the training solve and unperturbed scoring.
The archived noisy means are included as reference data.
