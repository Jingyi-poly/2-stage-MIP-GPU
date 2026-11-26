#ifndef PCTSP_LEVEL_BB_SCIP_H
#define PCTSP_LEVEL_BB_SCIP_H

#include "PCTSPExactSCIP.h"

#include <cstdint>
#include <limits>
#include <vector>

// Options for the external, level-synchronous branch-and-bound driver.
//
// The batch evaluator returns point estimates.  Correctness therefore relies
// on the caller-supplied oracle slack containing the point-estimate error:
// cuts use estimate-slack and incumbent upper bounds use estimate+slack.
// Set both oracle slacks to zero only for an exact/certified evaluator.
struct PCTSPLevelBBOptions {
    double timeLimit = 600.0;
    int evaluationBatchSize = 32;
    int nodeParallelism = 1;
    double oracleAbsoluteSlack = 1e-7;
    double oracleRelativeSlack = 1e-10;
    double absoluteGapTolerance = 1e-5;
    double relativeGapTolerance = 1e-6;
    double integralityTolerance = 1e-7;

    // SCIP's floating-point LP dual bound is shifted down by
    // lpBoundSafetySlack * max(1, |bound|) before it enters the certificate.
    double lpBoundSafetySlack = 1e-7;
    bool verbose = true;
};

struct PCTSPLevelBBStats {
    double lowerBound = 0.0;
    double upperBound = std::numeric_limits<double>::infinity();
    double relativeGap = std::numeric_limits<double>::infinity();

    int levelsCompleted = 0;
    int currentDepth = 0;
    int cutEpochs = 0;
    std::uint64_t nodesCreated = 0;
    std::uint64_t nodeLPSolves = 0;
    std::uint64_t incompleteNodeLPSolves = 0;
    std::uint64_t nodesInfeasible = 0;
    std::uint64_t nodesBoundPruned = 0;
    std::uint64_t nodesBranched = 0;
    std::uint64_t nodesFathomed = 0;
    std::uint64_t openNodes = 0;

    int optimalityCuts = 0;
    int evaluatedTours = 0;
    int evaluationBatches = 0;
    int requestedBatchSize = 1;
    int maxActualBatchSize = 0;
    int nodeParallelism = 1;

    // nodeLPSolveTime is the sum over workers; levelLPWallTime is elapsed wall
    // time across all parallel LP waves.
    double nodeLPSolveTime = 0.0;
    double levelLPWallTime = 0.0;
    double evaluationTime = 0.0;

    double oracleAbsoluteSlack = 0.0;
    double oracleRelativeSlack = 0.0;
    double maxAppliedOracleSlack = 0.0;
    double totalAppliedOracleSlack = 0.0;
    double maxAppliedLPBoundSlack = 0.0;

    bool hitTimeLimit = false;
    bool oracleSlackPreventedProof = false;
};

struct PCTSPLevelBBTracePoint {
    double timeSeconds = 0.0;
    int depth = 0;
    int cutEpoch = 0;
    std::uint64_t openNodes = 0;
    std::uint64_t nodeLPSolves = 0;
    int evaluatedTours = 0;
    int optimalityCuts = 0;
    double lowerBound = 0.0;
    double upperBound = std::numeric_limits<double>::infinity();
    double relativeGap = std::numeric_limits<double>::infinity();
    double levelLPWallTime = 0.0;
    double evaluationTime = 0.0;
};

// Exact (up to the requested gap tolerance and certified oracle/LP slacks)
// optional-visit stochastic giant-tour solver.
//
// SCIP is used only as a continuous node-LP engine.  The class owns the B&B
// frontier explicitly, processes one tree depth at a time, batches distinct
// integral tours across that depth, installs globally valid exact-tour integer
// L-shaped cuts, and reprocesses the depth to cut closure before branching.
class PCTSPLevelBBSCIP {
public:
    explicit PCTSPLevelBBSCIP(
        const Params& params,
        PCTSPLevelBBOptions options = PCTSPLevelBBOptions{});

    PCTSPResult solveFullStochasticMILPBatch(TourBatchEvalFunc batchEval);

    const PCTSPLevelBBStats& getLastStats() const { return lastStats; }
    const std::vector<PCTSPLevelBBTracePoint>& getLastTrace() const {
        return lastTrace;
    }

private:
    const Params& params;
    PCTSPLevelBBOptions options;
    PCTSPLevelBBStats lastStats;
    std::vector<PCTSPLevelBBTracePoint> lastTrace;
};

#endif
