#include "PCTSPExactSCIP.h"
#include "PCTSPLevelBBSCIP.h"
#include "commandline.h"
#include "ScenarioDemandSampler.h"
#include "InstanceCVRPLIB.h"
#include "FixedSkipPenalties.h"

#ifdef PCTSP_LEVEL_USE_CUDA
#include "SplitCUDA.h"
#include <cuda_runtime_api.h>
#endif

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <omp.h>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

#ifdef PCTSP_LEVEL_USE_CUDA
// The CUDA and CPU Split evaluators use different floating-point reduction
// orders.  Bounds therefore use an explicit interval that is wider than the
// per-candidate validation threshold below.
constexpr double kGpuOracleAbsoluteSlack = 1e-7;
constexpr double kGpuOracleRelativeSlack = 1e-7;
#endif

const char* backendName()
{
#ifdef PCTSP_LEVEL_USE_CUDA
    return "GPU";
#else
    return "CPU";
#endif
}

void writeTrace(const std::string& path,
                const std::vector<PCTSPLevelBBTracePoint>& trace,
                const PCTSPLevelBBStats& stats)
{
    if (path.empty())
        return;
    const std::filesystem::path tracePath(path);
    if (!tracePath.parent_path().empty())
        std::filesystem::create_directories(tracePath.parent_path());
    std::ofstream output(tracePath);
    if (!output)
        throw std::runtime_error("cannot open trace file: " + path);

    output << "time_s,event_index,level,epoch,frontier_nodes,"
              "nodes_processed,tours_evaluated,exact_tour_cuts,"
              "incumbent_cost,dual_bound,proof_gap,max_actual_batch,"
              "lp_time_s,evaluation_time_s,bound_valid\n";
    output << std::setprecision(17);
    for (std::size_t index = 0; index < trace.size(); ++index) {
        const PCTSPLevelBBTracePoint& point = trace[index];
        output << point.timeSeconds << ',' << index << ',' << point.depth << ','
               << point.cutEpoch << ',' << point.openNodes << ','
               << point.nodeLPSolves << ',' << point.evaluatedTours << ','
               << point.optimalityCuts << ',' << point.upperBound << ','
               << point.lowerBound << ',' << point.relativeGap << ','
               << stats.maxActualBatchSize << ',' << point.levelLPWallTime << ','
               << point.evaluationTime << ",1\n";
    }
}

void writeSolution(const std::string& path,
                   const Params& params,
                   const AlgorithmParameters& ap,
                   const PCTSPResult& result,
                   const PCTSPLevelBBStats& stats)
{
    std::ofstream output(path);
    if (!output)
        throw std::runtime_error("cannot open solution file: " + path);

    std::vector<bool> selected(params.nbClients + 1, false);
    for (int node : result.selectedNodes) {
        if (node >= 0 && node <= params.nbClients)
            selected[node] = true;
    }
    std::vector<int> selectedTour;
    for (int node : result.tour) {
        if (node != 0)
            selectedTour.push_back(node);
    }
    std::vector<int> skipped;
    for (int node = 1; node <= params.nbClients; ++node) {
        if (!selected[node])
            skipped.push_back(node);
    }

    output << "Model optional-visit-unlimited-route-count-capacity-penalized-split"
           << "\nAlgorithm level-synchronous-external-bb-integer-l-shaped"
           << "\nBackend " << backendName()
           << "\nScenarios " << params.n_scenarios
           << "\nSeed " << ap.seed
           << "\nScenarioSeed " << params.ap.scenarioSeed
           << "\nDemandDistribution " << scenario_demand_distribution_name(params.ap.demandDistribution)
           << "\nPenaltySeed " << params.ap.penaltySeed
           << "\nSkipPenScale " << params.ap.skipPenScale
           << "\nSkipPenalties";
    for (int node = 1; node <= params.nbClients; ++node)
        output << ' ' << std::setprecision(17) << params.cli[node].skipPenalty;
    output
           << "\nAlgoSeed " << params.ap.algoSeed
           << "\nLevelBatchSize " << stats.requestedBatchSize
           << "\nNodeParallelism " << stats.nodeParallelism
           << "\nTour: 0";
    for (int node : selectedTour)
        output << ' ' << node;
    for (int node : skipped)
        output << ' ' << node;
    output << " 0\nSelectedTour:";
    for (int node : selectedTour)
        output << ' ' << node;
    output << "\nSkipped:";
    for (int node : skipped)
        output << ' ' << node;
    output << std::fixed << std::setprecision(10)
           << "\nCost " << result.objValue
           << "\nUpperBound " << stats.upperBound
           << "\nLowerBound " << stats.lowerBound
           << "\nRelativeGap " << stats.relativeGap
           << "\nOptimal " << (result.optimal ? 1 : 0);
    write_scenario_demand_parameters(output, params.ap);
    output << '\n';
}

} // namespace

int main(int argc, char* argv[])
{
    const auto processStart = std::chrono::steady_clock::now();
    try {
        CommandLine commandline(argc, argv);
        omp_set_num_threads(std::max(1, commandline.ap.nthreads));
        if (commandline.nbVeh != INT_MAX) {
            throw std::invalid_argument(
                "level-B&B exact matches the unlimited-route-count Split "
                "oracle; a fixed -veh value is not supported");
        }
        if (commandline.verbose)
            print_algorithm_parameters(commandline.ap);

        std::cout << "----- READING INSTANCE: " << commandline.pathInstance
                  << '\n';
        InstanceCVRPLIB instance(commandline.pathInstance,
                                 commandline.isRoundingInteger);
        if (instance.isDurationConstraint) {
            throw std::invalid_argument(
                "level-B&B exact supports capacity-penalized Split without "
                "route-duration constraints");
        }
        Params params(instance.x_coords, instance.y_coords, instance.dist_mtx,
                      instance.service_time, instance.demands,
                      instance.vehicleCapacity, instance.durationLimit,
                      commandline.nbVeh, instance.isDurationConstraint,
                      commandline.verbose, commandline.ap);
        params.generate_scenario_demands(commandline.ap.n_extra_senarios);
        params.update_max_vehi();
        params.generate_skip_penalties();
        apply_fixed_skip_penalties(params);

        const double solveLimit = commandline.ap.timeLim > 0
            ? commandline.ap.timeLim
            : 600.0;
        const int batchCapacity =
            std::max(1, commandline.ap.gpuBatchSize);
        PCTSPExactSCIP cpuOracle(params, solveLimit, false);
        cpuOracle.validateWeightedScenarioOracle();
        TourBatchEvalFunc batchEvaluator;

#ifdef PCTSP_LEVEL_USE_CUDA
        const cudaError_t setDeviceStatus =
            cudaSetDevice(commandline.ap.deviceId);
        if (setDeviceStatus != cudaSuccess) {
            throw std::runtime_error(
                std::string("cudaSetDevice failed: ") +
                cudaGetErrorString(setDeviceStatus));
        }
        cudaDeviceProp deviceProperties{};
        const cudaError_t propertiesStatus = cudaGetDeviceProperties(
            &deviceProperties, commandline.ap.deviceId);
        if (propertiesStatus != cudaSuccess) {
            throw std::runtime_error(
                std::string("cudaGetDeviceProperties failed: ") +
                cudaGetErrorString(propertiesStatus));
        }
        std::cout << "GPU level evaluator: " << deviceProperties.name
                  << " (device " << commandline.ap.deviceId
                  << ", level batch capacity " << batchCapacity << ")\n";
        auto splitCuda = std::make_unique<SplitCUDA>(params, batchCapacity);
        std::cout << "GPU_MEMORY allocated_bytes="
                  << splitCuda->getAllocatedDeviceBytes()
                  << " requested_bytes=" << splitCuda->getRequestedDeviceBytes()
                  << " peak_bytes=" << splitCuda->getAllocatedDeviceBytes()
                  << " free_before_bytes=" << splitCuda->getFreeBeforeBytes()
                  << " free_after_bytes=" << splitCuda->getFreeAfterBytes()
                  << " max_actual_batch=" << batchCapacity
                  << " oom=0"
                  << '\n';
        if (commandline.ap.gpuMemoryOnly) {
            std::cout << "GPU_MEMORY_ONLY complete level_batch_capacity="
                      << batchCapacity << " scenarios=" << params.n_scenarios
                      << '\n';
            return 0;
        }

        const int fullClientCount = params.nbClients;
        bool validatedFirstBatch = false;
        batchEvaluator =
            [&](const std::vector<std::vector<int>>& tours) {
                if (static_cast<int>(tours.size()) >
                    splitCuda->getMaxBatchSize()) {
                    throw std::runtime_error(
                        "level candidate batch exceeds SplitCUDA capacity");
                }
                std::vector<std::unique_ptr<Individual>> storage;
                storage.reserve(tours.size());
                for (const auto& tour : tours) {
                    storage.emplace_back(
                        std::make_unique<Individual>(params, true));
                    storage.back()->chromT = tour;
                }
                std::map<int, std::vector<std::size_t>> groups;
                for (std::size_t index = 0; index < tours.size(); ++index)
                    groups[static_cast<int>(tours[index].size())].push_back(index);

                std::vector<double> costs(tours.size(), 0.0);
                try {
                    for (const auto& [activeClients, indices] : groups) {
                        if (activeClients == 0)
                            continue;
                        params.nbClients = activeClients;
                        splitCuda->setActiveClients(activeClients);
                        std::vector<Individual*> batch;
                        batch.reserve(indices.size());
                        for (std::size_t index : indices)
                            batch.push_back(storage[index].get());
                        splitCuda->reset_batch(static_cast<int>(batch.size()));
                        splitCuda->preprocess_batch(batch, params.nbVehicles);
                        splitCuda->generate_split_batch(
                            static_cast<int>(batch.size()));
                        const std::vector<double> meanCosts =
                            splitCuda->evaluateMeanCostsOnGPU_batch(batch);
                        for (std::size_t local = 0; local < indices.size(); ++local)
                            costs[indices[local]] = meanCosts[local];
                    }
                } catch (...) {
                    params.nbClients = fullClientCount;
                    splitCuda->setActiveClients(fullClientCount);
                    throw;
                }
                params.nbClients = fullClientCount;
                splitCuda->setActiveClients(fullClientCount);

                if (!validatedFirstBatch && !tours.empty()) {
                    double maximumDifference = 0.0;
                    for (std::size_t index = 0; index < tours.size(); ++index) {
                        const double cpuCost =
                            cpuOracle.evaluateTourCPU(tours[index]);
                        const double tolerance = kGpuOracleAbsoluteSlack
                            + kGpuOracleRelativeSlack
                                * std::max(1.0, std::abs(cpuCost));
                        const double difference =
                            std::abs(costs[index] - cpuCost);
                        maximumDifference = std::max(
                            maximumDifference, difference);
                        if (difference > tolerance) {
                            throw std::runtime_error(
                                "GPU/CPU level recourse validation failed");
                        }
                    }
                    std::cout << "GPU/CPU first level-batch validation: passed "
                              << "(candidates " << tours.size()
                              << ", max_abs_diff " << maximumDifference
                              << ")\n";
                    validatedFirstBatch = true;
                }
                return costs;
            };
#else
        if (commandline.ap.gpuMemoryOnly)
            throw std::invalid_argument(
                "-gpuMemoryOnly requires pctsp_level_exact_cuda");
        batchEvaluator =
            [&cpuOracle](const std::vector<std::vector<int>>& tours) {
                std::vector<double> costs;
                costs.reserve(tours.size());
                for (const auto& tour : tours)
                    costs.push_back(cpuOracle.evaluateTourCPU(tour));
                return costs;
            };
#endif

        PCTSPLevelBBOptions options;
        options.timeLimit = solveLimit;
        options.evaluationBatchSize = batchCapacity;
        options.nodeParallelism = std::max(1, commandline.ap.nthreads);
        options.verbose = commandline.verbose;
#ifdef PCTSP_LEVEL_USE_CUDA
        options.oracleAbsoluteSlack = kGpuOracleAbsoluteSlack;
        options.oracleRelativeSlack = kGpuOracleRelativeSlack;
#else
        options.oracleAbsoluteSlack = 1e-9;
        options.oracleRelativeSlack = 1e-12;
#endif

        PCTSPLevelBBSCIP solver(params, options);
        const PCTSPResult result =
            solver.solveFullStochasticMILPBatch(std::move(batchEvaluator));
        const PCTSPLevelBBStats& stats = solver.getLastStats();
        writeTrace(commandline.traceFile, solver.getLastTrace(), stats);

        std::cout << std::fixed << std::setprecision(6)
                  << "\nRESULT SUMMARY (LEVEL-SYNCHRONOUS B&B + "
                  << backendName() << " Split)\n"
                  << "  model: optional-visit stochastic Split\n"
                  << "  algorithm: external level-synchronous B&B, global "
                     "integer L-shaped cuts\n"
                  << "  stochastic cost: " << result.objValue << '\n'
                  << "  certified upper bound: " << stats.upperBound << '\n'
                  << "  lower bound:      " << stats.lowerBound << '\n'
                  << "  relative gap:     " << 100.0 * stats.relativeGap
                  << "%\n"
                  << "  proven optimal:   " << (result.optimal ? "yes" : "no")
                  << '\n'
                  << "  hit time limit:   "
                  << (stats.hitTimeLimit ? "yes" : "no") << '\n'
                  << "  levels completed: " << stats.levelsCompleted << '\n'
                  << "  cut epochs:       " << stats.cutEpochs << '\n'
                  << "  nodes created:    " << stats.nodesCreated << '\n'
                  << "  node LP solves:   " << stats.nodeLPSolves << '\n'
                  << "  open nodes:       " << stats.openNodes << '\n'
                  << "  evaluated tours:  " << stats.evaluatedTours << '\n'
                  << "  evaluation batches: " << stats.evaluationBatches << '\n'
                  << "  max actual level batch: "
                  << stats.maxActualBatchSize << '/'
                  << stats.requestedBatchSize << '\n'
                  << "  exact-tour cuts:  " << stats.optimalityCuts << '\n'
                  << "  oracle interval:  +/- ("
                  << stats.oracleAbsoluteSlack << " + "
                  << stats.oracleRelativeSlack << " * |Q|)\n"
                  << "  node LP wall time: " << stats.levelLPWallTime << " s\n"
                  << "  recourse time:    " << stats.evaluationTime << " s\n"
                  << "  selected clients: "
                  << (result.selectedNodes.empty()
                          ? 0
                          : static_cast<int>(result.selectedNodes.size()) - 1)
                  << '/' << params.nbClients << '\n';

        if (!std::isfinite(result.objValue) || result.tour.empty())
            return 2;
        writeSolution(commandline.pathSolution, params, commandline.ap,
                      result, stats);
    } catch (const std::exception& error) {
        std::cerr << "EXCEPTION | " << error.what() << '\n';
        return 1;
    } catch (const std::string& error) {
        std::cerr << "EXCEPTION | " << error << '\n';
        return 1;
    }

    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - processStart).count();
    std::cout << "Total wall time: " << elapsed << " s\n";
    return 0;
}
