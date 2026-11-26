#include "AlgorithmParameters.h"
#include "InstanceCVRPLIB.h"
#include "PCTSPExactSCIP.h"
#include "PCTSPLevelBBSCIP.h"
#include "Params.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

struct CaseSummary {
    double objective = std::numeric_limits<double>::infinity();
    double lowerBound = -std::numeric_limits<double>::infinity();
    std::size_t selectedClients = 0;
    int levelsCompleted = 0;
    int cutEpochs = 0;
    std::uint64_t nodeLPSolves = 0;
    std::size_t tracePoints = 0;
};

double skipCost(const Params& params, const std::vector<int>& permutation)
{
    std::vector<bool> selected(params.nbClients + 1, false);
    for (int client : permutation)
        selected[client] = true;

    double value = 0.0;
    for (int client = 1; client <= params.nbClients; ++client) {
        if (!selected[client])
            value += params.cli[client].skipPenalty;
    }
    return value;
}

CaseSummary runCase(const std::string& instancePath,
                    double skipPenaltyScale,
                    bool expectEmpty,
                    bool expectProperSubset)
{
    AlgorithmParameters ap = default_algorithm_parameters();
    ap.optionalVisit = 1;
    ap.skipPenScale = skipPenaltyScale;
    ap.scenarioSeed = 11;
    ap.penaltySeed = 17;
    ap.algoSeed = 23;
    ap.n_extra_senarios = 3;
    ap.nthreads = 1;

    InstanceCVRPLIB instance(instancePath, true);
    Params params(instance.x_coords, instance.y_coords, instance.dist_mtx,
                  instance.service_time, instance.demands,
                  instance.vehicleCapacity, instance.durationLimit,
                  std::numeric_limits<int>::max(),
                  instance.isDurationConstraint, false, ap);
    params.generate_scenario_demands(ap.n_extra_senarios);
    params.update_max_vehi();
    params.generate_skip_penalties();
    if (params.nbClients > 9)
        throw std::invalid_argument(
            "level-B&B brute-force regression is limited to 9 clients");

    // Keep the enumeration oracle independent of the level-synchronous B&B
    // implementation.  PCTSPExactSCIP exposes the same reference O(n^2)
    // stochastic Split evaluator used by the existing tiny regression.
    PCTSPExactSCIP oracle(params, 30.0, false);
    double bruteObjective = std::numeric_limits<double>::infinity();
    std::vector<int> bruteTour;
    const unsigned subsetCount = 1U << params.nbClients;
    for (unsigned mask = 0; mask < subsetCount; ++mask) {
        std::vector<int> permutation;
        for (int client = 1; client <= params.nbClients; ++client) {
            if ((mask & (1U << (client - 1))) != 0U)
                permutation.push_back(client);
        }

        do {
            const double objective = oracle.evaluateTourCPU(permutation)
                + skipCost(params, permutation);
            if (objective < bruteObjective) {
                bruteObjective = objective;
                bruteTour = permutation;
            }
        } while (std::next_permutation(
            permutation.begin(), permutation.end()));
    }

    if (expectEmpty && !bruteTour.empty())
        throw std::runtime_error(
            "low-penalty brute-force reference is not the empty subset");
    if (expectProperSubset &&
        (bruteTour.empty() ||
         bruteTour.size() == static_cast<std::size_t>(params.nbClients)))
        throw std::runtime_error(
            "subset-case brute-force reference is not a proper nonempty subset");

    PCTSPLevelBBOptions options;
    options.timeLimit = 30.0;
    options.evaluationBatchSize = 4;
    options.nodeParallelism = 2;
    // The callback below is the deterministic CPU reference oracle, so no
    // sampling/numerical envelope is needed around its returned value.
    options.oracleAbsoluteSlack = 0.0;
    options.oracleRelativeSlack = 0.0;
    options.absoluteGapTolerance = 1e-6;
    options.relativeGapTolerance = 1e-7;
    options.integralityTolerance = 1e-7;
    options.lpBoundSafetySlack = 1e-9;
    options.verbose = false;

    TourBatchEvalFunc batchEvaluator =
        [&oracle](const std::vector<std::vector<int>>& tours) {
            std::vector<double> costs;
            costs.reserve(tours.size());
            for (const std::vector<int>& tour : tours)
                costs.push_back(oracle.evaluateTourCPU(tour));
            return costs;
        };

    PCTSPLevelBBSCIP solver(params, options);
    const PCTSPResult exact =
        solver.solveFullStochasticMILPBatch(std::move(batchEvaluator));
    const PCTSPLevelBBStats& stats = solver.getLastStats();
    const std::vector<PCTSPLevelBBTracePoint>& trace = solver.getLastTrace();
    const double tolerance = 2e-6 * std::max(1.0, std::abs(bruteObjective));

    if (!exact.optimal)
        throw std::runtime_error(
            "level-synchronous SCIP did not close the tiny proof gap");
    if (stats.hitTimeLimit)
        throw std::runtime_error(
            "level-synchronous SCIP unexpectedly hit the tiny time limit");
    if (stats.oracleSlackPreventedProof)
        throw std::runtime_error(
            "zero-slack reference oracle unexpectedly prevented proof closure");
    if (stats.incompleteNodeLPSolves != 0)
        throw std::runtime_error(
            "an incomplete node LP was accepted in an optimal certificate");
    if (std::abs(exact.objValue - bruteObjective) > tolerance)
        throw std::runtime_error(
            "level-synchronous SCIP objective differs from brute force");
    if (stats.lowerBound > bruteObjective + tolerance)
        throw std::runtime_error(
            "final certified lower bound exceeds the brute-force optimum");
    if (stats.upperBound < bruteObjective - tolerance)
        throw std::runtime_error(
            "final incumbent upper bound is below the brute-force optimum");
    if (std::abs(stats.lowerBound - bruteObjective) > tolerance ||
        std::abs(stats.upperBound - bruteObjective) > tolerance)
        throw std::runtime_error(
            "final lower/upper bounds did not close at the brute-force optimum");

    std::vector<int> exactPermutation;
    if (exact.tour.size() >= 2)
        exactPermutation.assign(exact.tour.begin() + 1,
                                exact.tour.end() - 1);
    const double serializedObjective = oracle.evaluateTourCPU(exactPermutation)
        + skipCost(params, exactPermutation);
    if (std::abs(serializedObjective - exact.objValue) > tolerance)
        throw std::runtime_error(
            "reported selected tour does not reproduce the exact objective");
    if (expectEmpty && !exactPermutation.empty())
        throw std::runtime_error(
            "level-synchronous SCIP failed to return the empty optimum");
    if (expectProperSubset &&
        (exactPermutation.empty() ||
         exactPermutation.size() ==
             static_cast<std::size_t>(params.nbClients)))
        throw std::runtime_error(
            "level-synchronous SCIP failed to return a proper subset optimum");

    if (stats.levelsCompleted <= 0 || stats.nodesCreated == 0 ||
        stats.nodeLPSolves == 0)
        throw std::runtime_error(
            "level-synchronous statistics did not record any B&B work");
    if (stats.nodeParallelism != options.nodeParallelism)
        throw std::runtime_error(
            "reported node parallelism differs from the requested value");
    if (stats.requestedBatchSize != options.evaluationBatchSize)
        throw std::runtime_error(
            "reported evaluator batch size differs from the requested value");
    if (expectProperSubset &&
        (stats.cutEpochs <= 0 || stats.evaluationBatches <= 0 ||
         stats.evaluatedTours <= 0 || stats.maxActualBatchSize <= 0))
        throw std::runtime_error(
            "subset case did not exercise a cut/evaluation epoch");
    if (trace.empty())
        throw std::runtime_error(
            "level-synchronous solver did not emit a bound trace");

    double previousLowerBound = -std::numeric_limits<double>::infinity();
    double previousUpperBound = std::numeric_limits<double>::infinity();
    int previousCutEpoch = -1;
    bool sawPositiveCutEpoch = false;
    for (const PCTSPLevelBBTracePoint& point : trace) {
        if (!std::isfinite(point.lowerBound))
            throw std::runtime_error("trace contains a non-finite lower bound");
        if (point.lowerBound + tolerance < previousLowerBound)
            throw std::runtime_error("trace lower bound is not monotone");
        if (std::isfinite(previousUpperBound) &&
            (!std::isfinite(point.upperBound) ||
             point.upperBound > previousUpperBound + tolerance))
            throw std::runtime_error("trace upper bound is not monotone");
        if (point.cutEpoch < previousCutEpoch)
            throw std::runtime_error("trace cut epoch moved backwards");

        // This is the executable bound_valid check: every trace certificate
        // must enclose the independently enumerated optimum.
        const bool boundValid =
            point.lowerBound <= bruteObjective + tolerance &&
            (!std::isfinite(point.upperBound) ||
             point.upperBound >= bruteObjective - tolerance);
        if (!boundValid)
            throw std::runtime_error(
                "trace bounds do not enclose the brute-force optimum");

        previousLowerBound = point.lowerBound;
        previousUpperBound = point.upperBound;
        previousCutEpoch = point.cutEpoch;
        sawPositiveCutEpoch = sawPositiveCutEpoch || point.cutEpoch > 0;
    }
    if (expectProperSubset && !sawPositiveCutEpoch)
        throw std::runtime_error(
            "trace did not expose a positive cut epoch for the subset case");

    return CaseSummary{
        exact.objValue,
        stats.lowerBound,
        exactPermutation.size(),
        stats.levelsCompleted,
        stats.cutEpochs,
        stats.nodeLPSolves,
        trace.size(),
    };
}

} // namespace

int main(int argc, char** argv)
{
    try {
        if (argc != 2)
            throw std::invalid_argument(
                "usage: test_pctsp_level_bb_bruteforce <tiny.vrp>");

        const CaseSummary empty = runCase(argv[1], 0.1, true, false);
        const CaseSummary subset = runCase(argv[1], 1.2, false, true);

        std::cout << "PCTSP_LEVEL_BB_BRUTE_FORCE_PASS"
                  << " empty_objective=" << empty.objective
                  << " empty_selected=" << empty.selectedClients
                  << " subset_objective=" << subset.objective
                  << " subset_lower_bound=" << subset.lowerBound
                  << " subset_selected=" << subset.selectedClients
                  << " levels=" << subset.levelsCompleted
                  << " epochs=" << subset.cutEpochs
                  << " node_lp_solves=" << subset.nodeLPSolves
                  << " trace_points=" << subset.tracePoints << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "PCTSP_LEVEL_BB_BRUTE_FORCE_FAIL "
                  << error.what() << '\n';
        return 1;
    }
}
