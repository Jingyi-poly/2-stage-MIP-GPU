#include "PCTSPExactSCIP.h"
#include "SplitCUDA.h"
#include "commandline.h"
#include "InstanceCVRPLIB.h"

#include <cuda_runtime_api.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <omp.h>

int main(int argc, char* argv[])
{
    const auto start = std::chrono::steady_clock::now();
    try {
        CommandLine commandline(argc, argv);
        omp_set_num_threads(commandline.ap.nthreads);
        if (commandline.nbVeh != INT_MAX) {
            throw std::invalid_argument(
                "SCIP+CUDA exact currently matches the unlimited-route-count "
                "Split "
                "oracle; a fixed -veh value is not yet supported");
        }
        if (commandline.verbose)
            print_algorithm_parameters(commandline.ap);

        std::cout << "----- READING INSTANCE: " << commandline.pathInstance << '\n';
        InstanceCVRPLIB cvrp(commandline.pathInstance,
                             commandline.isRoundingInteger);
        if (cvrp.isDurationConstraint) {
            throw std::invalid_argument(
                "SCIP+CUDA exact currently supports the capacity-penalized "
                "Split objective without duration constraints; route-duration "
                "penalties are not implemented by "
                "the GPU Split oracle");
        }
        Params params(cvrp.x_coords, cvrp.y_coords, cvrp.dist_mtx,
                      cvrp.service_time, cvrp.demands,
                      cvrp.vehicleCapacity, cvrp.durationLimit,
                      commandline.nbVeh, cvrp.isDurationConstraint,
                      commandline.verbose, commandline.ap);
        params.generate_scenario_demands(commandline.ap.n_extra_senarios);
        params.update_max_vehi();
        params.generate_skip_penalties();

        const int batchCapacity = std::max(1, commandline.ap.gpuBatchSize);
        const cudaError_t setDeviceStatus = cudaSetDevice(commandline.ap.deviceId);
        if (setDeviceStatus != cudaSuccess) {
            throw std::runtime_error(
                std::string("cudaSetDevice failed: ") +
                cudaGetErrorString(setDeviceStatus));
        }
        cudaDeviceProp deviceProperties{};
        const cudaError_t propertiesStatus =
            cudaGetDeviceProperties(&deviceProperties, commandline.ap.deviceId);
        if (propertiesStatus != cudaSuccess) {
            throw std::runtime_error(
                std::string("cudaGetDeviceProperties failed: ") +
                cudaGetErrorString(propertiesStatus));
        }
        std::cout << "GPU evaluator: " << deviceProperties.name
                  << " (device " << commandline.ap.deviceId
                  << ", batch capacity " << batchCapacity << ")\n";
        const double solveLimit = commandline.ap.timeLim > 0
            ? commandline.ap.timeLim
            : 600.0;
        PCTSPExactSCIP solver(params, solveLimit, commandline.verbose);
        SplitCUDA splitCuda(params, batchCapacity);
        std::cout << "GPU_MEMORY allocated_bytes="
                  << splitCuda.getAllocatedDeviceBytes()
                  << " requested_bytes=" << splitCuda.getRequestedDeviceBytes()
                  << " free_before_bytes=" << splitCuda.getFreeBeforeBytes()
                  << " free_after_bytes=" << splitCuda.getFreeAfterBytes()
                  << '\n';
        const int fullClientCount = params.nbClients;
        bool validatedFirstGpuBatch = false;
        TourBatchEvalFunc gpuBatchEval =
            [&](const std::vector<std::vector<int>>& tours) {
                if (static_cast<int>(tours.size()) > splitCuda.getMaxBatchSize())
                    throw std::runtime_error(
                        "SCIP candidate batch exceeds SplitCUDA capacity");

                std::vector<std::unique_ptr<Individual>> storage;
                storage.reserve(tours.size());
                for (const auto& tour : tours) {
                    storage.emplace_back(std::make_unique<Individual>(params, true));
                    storage.back()->chromT = tour;
                }

                // SplitCUDA requires a common active length within a launch.
                // Exact-tour neighbors normally share a subset, but grouping
                // makes the evaluator correct for any future mixed-size batch.
                std::map<int, std::vector<std::size_t>> groups;
                for (std::size_t index = 0; index < tours.size(); ++index)
                    groups[static_cast<int>(tours[index].size())].push_back(index);

                std::vector<double> costs(tours.size(), 0.0);
                try {
                    for (const auto& [activeClients, indices] : groups) {
                        if (activeClients == 0)
                            continue; // Empty selected set has zero recourse.

                        params.nbClients = activeClients;
                        splitCuda.setActiveClients(activeClients);
                        std::vector<Individual*> batch;
                        batch.reserve(indices.size());
                        for (std::size_t index : indices)
                            batch.push_back(storage[index].get());

                        splitCuda.reset_batch(static_cast<int>(batch.size()));
                        splitCuda.preprocess_batch(batch, params.nbVehicles);
                        splitCuda.generate_split_batch(
                            static_cast<int>(batch.size()));
                        const std::vector<double> meanCosts =
                            splitCuda.evaluateMeanCostsOnGPU_batch(batch);
                        for (std::size_t local = 0; local < indices.size(); ++local)
                            costs[indices[local]] = meanCosts[local];
                    }
                } catch (...) {
                    params.nbClients = fullClientCount;
                    splitCuda.setActiveClients(fullClientCount);
                    throw;
                }
                params.nbClients = fullClientCount;
                splitCuda.setActiveClients(fullClientCount);

                // In verbose/debug runs, certify the first GPU master-tour
                // value against the independent O(n^2) CPU oracle.  Production
                // runs can use -log 0 to avoid this one-time validation cost.
                if (commandline.verbose && !validatedFirstGpuBatch &&
                    !tours.empty()) {
                    const double cpuCost = solver.evaluateTourCPU(tours.front());
                    const double tolerance =
                        1e-7 * std::max(1.0, std::abs(cpuCost));
                    if (std::abs(costs.front() - cpuCost) > tolerance) {
                        throw std::runtime_error(
                            "GPU/CPU Split cross-validation failed: GPU=" +
                            std::to_string(costs.front()) + ", CPU=" +
                            std::to_string(cpuCost));
                    }
                    std::cout << "GPU/CPU first-tour validation: passed (cost "
                              << cpuCost << ")\n";
                    validatedFirstGpuBatch = true;
                }
                return costs;
            };

        const PCTSPResult result = solver.solveFullStochasticMILPBatch(
            std::move(gpuBatchEval), batchCapacity);
        const SCIPBendersStats& stats = solver.getLastStats();

        if (!commandline.traceFile.empty()) {
            const std::filesystem::path tracePath(commandline.traceFile);
            if (!tracePath.parent_path().empty())
                std::filesystem::create_directories(tracePath.parent_path());
            std::ofstream traceOutput(tracePath);
            if (!traceOutput)
                throw std::runtime_error(
                    "cannot open trace file: " + commandline.traceFile);
            traceOutput
                << "time_s,iteration,evaluated_tours,incumbent_cost,"
                   "dual_bound,proof_gap\n";
            traceOutput << std::setprecision(17);
            for (const SCIPBendersTracePoint& point : solver.getLastTrace()) {
                traceOutput << point.timeSeconds << ',' << point.iteration << ','
                            << point.evaluatedTours << ','
                            << point.incumbentCost << ',' << point.dualBound << ','
                            << point.proofGap << '\n';
            }
        }

        std::cout << std::fixed << std::setprecision(6)
                  << "\nRESULT SUMMARY (SCIP + GPU Split)\n"
                  << "  model: "
                  << (params.ap.optionalVisit
                          ? "optional-visit stochastic Split"
                          : "all-client stochastic Split")
                  << "\n"
                  << "  stochastic cost: " << result.objValue << '\n'
                  << "  lower bound:      " << stats.lowerBound << '\n'
                  << "  relative gap:     " << 100.0 * stats.relativeGap << "%\n"
                  << "  proven optimal:   " << (result.optimal ? "yes" : "no")
                  << "; max actual batch: " << stats.maxActualBatchSize << '/'
                  << stats.requestedBatchSize << '\n'
                  << "  hit time limit:   "
                  << (stats.hitTimeLimit ? "yes" : "no") << '\n'
                  << "  evaluated tours:  " << stats.evaluatedTours << '\n'
                  << "  evaluation batches: " << stats.evaluationBatches << '\n'
                  << "  exact-tour cuts:  " << stats.optimalityCuts << '\n'
                  << "  master time:      " << stats.masterSolveTime << " s\n"
                  << "  GPU evaluator time: " << stats.evaluationTime << " s\n"
                  << "  selected clients: "
                  << (result.selectedNodes.empty()
                          ? 0
                          : static_cast<int>(result.selectedNodes.size()) - 1)
                  << '/' << params.nbClients << '\n'
                  << "  chromT:";
        for (int node : result.tour)
            if (node != 0)
                std::cout << ' ' << node;
        std::cout << '\n';
        if (result.tour.empty())
            return 2;

        std::ofstream solutionFile(commandline.pathSolution);
        if (!solutionFile)
            throw std::runtime_error(
                "cannot open solution file: " + commandline.pathSolution);
        std::vector<bool> selected(params.nbClients + 1, false);
        for (int node : result.selectedNodes)
            selected[node] = true;
        std::vector<int> skipped;
        for (int node = 1; node <= params.nbClients; ++node) {
            if (!selected[node])
                skipped.push_back(node);
        }
        solutionFile << "Model "
                     << (params.ap.optionalVisit
                             ? "optional-visit-unlimited-route-count-capacity-penalized-split"
                             : "all-client-unlimited-route-count-capacity-penalized-split")
                     << "\nScenarios " << params.n_scenarios
                     << "\nSeed " << commandline.ap.seed
                     << "\nScenarioSeed " << params.ap.scenarioSeed
                     << "\nPenaltySeed " << params.ap.penaltySeed
                     << "\nSkipPenScale " << params.ap.skipPenScale
                     << "\nAlgoSeed " << params.ap.algoSeed
                     << "\nTour: 0";
        // Preserve a complete chromosome for legacy/OOS readers while
        // SelectedTour records the exact route evaluated by the master.
        for (int node : result.tour) {
            if (node != 0)
                solutionFile << ' ' << node;
        }
        for (int node : skipped)
            solutionFile << ' ' << node;
        solutionFile << " 0";
        solutionFile << "\nSelectedTour:";
        for (int node : result.tour) {
            if (node != 0)
                solutionFile << ' ' << node;
        }
        solutionFile << "\nSkipped:";
        for (int node : skipped)
            solutionFile << ' ' << node;
        solutionFile << std::fixed << std::setprecision(10)
                     << "\nCost " << result.objValue
                     << "\nLowerBound " << stats.lowerBound
                     << "\nRelativeGap " << stats.relativeGap
                     << "\nOptimal " << (result.optimal ? 1 : 0) << '\n';
    } catch (const std::exception& error) {
        std::cerr << "EXCEPTION | " << error.what() << '\n';
        return 1;
    } catch (const std::string& error) {
        std::cerr << "EXCEPTION | " << error << '\n';
        return 1;
    }

    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    std::cout << "Total wall time: " << elapsed << " s\n";
    return 0;
}
