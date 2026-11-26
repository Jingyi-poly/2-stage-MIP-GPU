#ifndef PCTSP_EXACT_SCIP_H
#define PCTSP_EXACT_SCIP_H

#include "PCTSPExact.h"

#include <functional>
#include <limits>
#include <utility>
#include <vector>

// Batch form kept separate from TourEvalFunc so the master can submit its
// certified incumbent together with deterministic unseen neighbors in one GPU
// launch.  Only the first tour is needed for correctness; the others add valid
// cuts early and improve GPU utilization.
using TourBatchEvalFunc = std::function<std::vector<double>(
    const std::vector<std::vector<int>>&)>;

struct SCIPBendersStats {
    double lowerBound = -std::numeric_limits<double>::infinity();
    double upperBound = std::numeric_limits<double>::infinity();
    double relativeGap = std::numeric_limits<double>::infinity();
    int iterations = 0;
    int optimalityCuts = 0;
    int evaluatedTours = 0;
    int evaluationBatches = 0;
    int requestedBatchSize = 1;
    int maxActualBatchSize = 0;
    double masterSolveTime = 0.0;
    double evaluationTime = 0.0;
    bool hitTimeLimit = false;
};

struct SCIPBendersTracePoint {
    double timeSeconds = 0.0;
    int iteration = 0;
    int evaluatedTours = 0;
    double incumbentCost = std::numeric_limits<double>::infinity();
    double dualBound = -std::numeric_limits<double>::infinity();
    double proofGap = std::numeric_limits<double>::infinity();
};

// Exact stochastic optional-visit giant-tour Split-objective solver.
//
// The CPU SCIP master chooses a client subset and one depot-rooted Hamiltonian
// cycle through that subset (all clients are fixed selected when optionalVisit
// is disabled).  Its second-stage cost is evaluated outside SCIP (CPU Split DP
// or the supplied GPU evaluator), then a globally valid, subset-conditioned
// distance-anchored integer L-shaped cut is added and the master is solved
// again.  Thus SCIP's dual bound remains a rigorous lower bound while evaluated
// selected tours provide rigorous upper bounds.  The empty subset is feasible.
class PCTSPExactSCIP {
public:
    PCTSPExactSCIP(const Params& params,
                   double timeLimit = 600.0,
                   bool verbose = true);

    PCTSPResult solveFullStochasticMILP(TourEvalFunc tourEval = nullptr,
                                        int batchSize = 1);

    PCTSPResult solveFullStochasticMILPBatch(TourBatchEvalFunc batchEval,
                                             int batchSize = 1);

    // Reference O(n^2) capacity-penalized (no duration) Split oracle, exposed for CUDA
    // cross-validation and regression tests.
    double evaluateTourCPU(const std::vector<int>& perm) const;
    void validateWeightedScenarioOracle() const;

    const SCIPBendersStats& getLastStats() const { return lastStats; }
    const std::vector<SCIPBendersTracePoint>& getLastTrace() const {
        return lastTrace;
    }

private:
    const Params& params;
    double timeLimit;
    bool verbose;
    int n;
    SCIPBendersStats lastStats;
    std::vector<SCIPBendersTracePoint> lastTrace;
    std::vector<std::pair<int, int>> scenarioTypes; // representative row, exact multiplicity

    double splitDP(const std::vector<int>& perm, int scenario) const;
    double evaluateTourCPUUncompressed(const std::vector<int>& perm) const;
};

#endif
