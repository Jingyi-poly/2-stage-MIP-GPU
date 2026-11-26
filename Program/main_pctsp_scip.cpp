#include "PCTSPExactSCIP.h"
#include "commandline.h"
#include "InstanceCVRPLIB.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <omp.h>

int main(int argc, char* argv[])
{
    const auto start = std::chrono::steady_clock::now();
    try {
        CommandLine commandline(argc, argv);
        omp_set_num_threads(commandline.ap.nthreads);
        if (commandline.nbVeh != INT_MAX) {
            throw std::invalid_argument(
                "SCIP exact currently matches the unlimited-route-count Split "
                "oracle; a fixed -veh value is not yet supported");
        }
        if (commandline.verbose)
            print_algorithm_parameters(commandline.ap);

        std::cout << "----- READING INSTANCE: " << commandline.pathInstance << '\n';
        InstanceCVRPLIB cvrp(commandline.pathInstance,
                             commandline.isRoundingInteger);
        if (cvrp.isDurationConstraint) {
            throw std::invalid_argument(
                "SCIP exact currently supports the capacity-penalized Split "
                "objective without duration constraints; "
                "route-duration penalties are not part of its Split oracle");
        }
        Params params(cvrp.x_coords, cvrp.y_coords, cvrp.dist_mtx,
                      cvrp.service_time, cvrp.demands,
                      cvrp.vehicleCapacity, cvrp.durationLimit,
                      commandline.nbVeh, cvrp.isDurationConstraint,
                      commandline.verbose, commandline.ap);
        params.generate_scenario_demands(commandline.ap.n_extra_senarios);
        params.update_max_vehi();
        params.generate_skip_penalties();

        const double solveLimit = commandline.ap.timeLim > 0
            ? commandline.ap.timeLim
            : 600.0;
        const int candidateBatchSize =
            std::max(1, commandline.ap.gpuBatchSize);
        PCTSPExactSCIP solver(params, solveLimit, commandline.verbose);
        const PCTSPResult result = solver.solveFullStochasticMILP(
            nullptr, candidateBatchSize);
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
                  << "\nRESULT SUMMARY\n"
                  << "  model: "
                  << (params.ap.optionalVisit
                          ? "optional-visit stochastic Split"
                          : "all-client stochastic Split")
                  << "\n"
                  << "  stochastic cost: " << result.objValue << '\n'
                  << "  lower bound:      " << stats.lowerBound << '\n'
                  << "  relative gap:     " << 100.0 * stats.relativeGap << "%\n"
                  << "  proven optimal:   " << (result.optimal ? "yes" : "no") << '\n'
                  << "  evaluated tours:  " << stats.evaluatedTours << '\n'
                  << "  evaluation batches: " << stats.evaluationBatches << '\n'
                  << "  max actual batch: " << stats.maxActualBatchSize << '/'
                  << stats.requestedBatchSize << '\n'
                  << "  exact-tour cuts:  " << stats.optimalityCuts << '\n'
                  << "  master time:      " << stats.masterSolveTime << " s\n"
                  << "  evaluator time:   " << stats.evaluationTime << " s\n"
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
        // Keep Tour as a complete chromosome for legacy/OOS consumers.  The
        // exact selected route comes first; skipped clients are appended in
        // deterministic index order and are ignored by SelectedTour-aware code.
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
