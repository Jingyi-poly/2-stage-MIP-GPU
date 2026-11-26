#include "GeneticHGS.h"
#include "ScenarioDemandSampler.h"
#include "commandline.h"
#include "InstanceCVRPLIB.h"
#include "SplitCUDA.h"
#include <chrono>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <algorithm>
#include <cstdint>
#include <utility>
#include <omp.h>

using namespace std;

int main(int argc, char *argv[])
{
	auto tAllStart = std::chrono::steady_clock::now();

	try
	{
		CommandLine commandline(argc, argv);
		omp_set_num_threads(commandline.ap.nthreads);

		if (commandline.verbose) print_algorithm_parameters(commandline.ap);
		if (commandline.verbose)
			std::cout << "----- READING INSTANCE: " << commandline.pathInstance << std::endl;

		InstanceCVRPLIB cvrp(commandline.pathInstance, commandline.isRoundingInteger);

		Params params(cvrp.x_coords, cvrp.y_coords, cvrp.dist_mtx, cvrp.service_time, cvrp.demands,
					  cvrp.vehicleCapacity, cvrp.durationLimit, commandline.nbVeh,
					  cvrp.isDurationConstraint, commandline.verbose, commandline.ap);

		params.generate_scenario_demands(commandline.ap.n_extra_senarios);
		params.update_max_vehi();
		params.apply_fixed_route_capacity();
		params.generate_skip_penalties();
		const int fullNbClients = params.nbClients;

		const int gpuBatchSize = commandline.ap.gpuBatchSize;
		SplitCUDA::cudaCheck(cudaSetDevice(commandline.ap.deviceId), "cudaSetDevice");
		SplitCUDA splitCuda(params, gpuBatchSize);
		std::cout << "GPU_MEMORY allocated_bytes=" << splitCuda.getAllocatedDeviceBytes()
				  << " requested_bytes=" << splitCuda.getRequestedDeviceBytes()
				  << " free_before_bytes=" << splitCuda.getFreeBeforeBytes()
				  << " free_after_bytes=" << splitCuda.getFreeAfterBytes() << std::endl;
		std::cout << "Finished generating scenarios  (n_scen=" << params.n_scenarios
				  << "  n_cli=" << params.nbClients << ")  [GPU mode]" << std::endl;
		if (commandline.ap.gpuMemoryOnly)
		{
			std::cout << "GPU memory-only mode: allocation completed; HGS was not run."
					  << std::endl;
			return 0;
		}

		int maxActualGpuBatch = 0;
		std::uint64_t gpuBatchLaunches = 0;

		// GPU batch evaluation function. Optional-visit individuals have
		// variable-length selected tours. SplitCUDA uses a rectangular buffer,
		// so evaluate together all individuals with the same selected length.
		auto gpuEval = [&splitCuda, &params, fullNbClients,
			&maxActualGpuBatch, &gpuBatchLaunches](std::vector<Individual*>& batch) {
			const int batchSize = static_cast<int>(batch.size());
			if (batchSize == 0) return;
			const bool useOptional = params.ap.optionalVisit != 0;

			std::vector<std::vector<int>> savedChromT(batchSize);
			std::vector<bool> chromTFiltered(batchSize, false);

			// params.nbClients and SplitCUDA::m are temporarily changed below.
			// Restore both, as well as every filtered chromosome, even if CUDA
			// reports an error part way through a batch.
			struct RestoreState
			{
				Params & params;
				SplitCUDA & splitCuda;
				int fullNbClients;
				std::vector<Individual*> & batch;
				std::vector<std::vector<int>> & savedChromT;
				std::vector<bool> & chromTFiltered;

				~RestoreState()
				{
					for (std::size_t b = 0; b < batch.size(); ++b)
						if (chromTFiltered[b])
							batch[b]->chromT = std::move(savedChromT[b]);
					params.nbClients = fullNbClients;
					splitCuda.setActiveClients(fullNbClients);
				}
			} restore{params, splitCuda, fullNbClients, batch, savedChromT, chromTFiltered};

			std::vector<std::vector<Individual*>> groups(fullNbClients + 1);
			for (int b = 0; b < batchSize; ++b)
			{
				Individual & indiv = *batch[b];
				if (useOptional && !indiv.clientVisited.empty())
				{
					if (static_cast<int>(indiv.clientVisited.size()) <= fullNbClients)
						throw std::invalid_argument("Optional-visit mask is shorter than the client set");
					if (static_cast<int>(indiv.chromT.size()) != fullNbClients)
						throw std::invalid_argument("Optional-visit chromosome is not a full client permutation");

					savedChromT[b] = indiv.chromT;
					chromTFiltered[b] = true;
					indiv.chromT.clear();
					indiv.chromT.reserve(fullNbClients);
					for (int client : savedChromT[b])
						if (client >= 1 && client <= fullNbClients && indiv.clientVisited[client])
							indiv.chromT.push_back(client);
					const int selectedByMask = static_cast<int>(std::count(
						indiv.clientVisited.begin() + 1,
						indiv.clientVisited.begin() + fullNbClients + 1, true));
					if (static_cast<int>(indiv.chromT.size()) != selectedByMask)
						throw std::invalid_argument("Optional-visit chromosome and mask are inconsistent");
				}

				const int selectedCount = static_cast<int>(indiv.chromT.size());
				if (selectedCount < 0 || selectedCount > fullNbClients)
					throw std::invalid_argument("Selected tour has an invalid number of clients");
				groups[selectedCount].push_back(&indiv);
			}

			// The zero-client recourse cost is exactly zero. Initialise the full
			// evaluation record here; the skip cost is added once, uniformly, below.
			for (Individual * indiv : groups[0])
			{
				indiv->eval = EvalIndivMultiScen();
				indiv->resetEval(params);
				std::fill(indiv->eval.penalizedCostScen.begin(), indiv->eval.penalizedCostScen.end(), 0.0);
				std::fill(indiv->eval.nbRoutesScen.begin(), indiv->eval.nbRoutesScen.end(), 0);
				std::fill(indiv->eval.distanceScen.begin(), indiv->eval.distanceScen.end(), 0.0);
				std::fill(indiv->eval.capacityExcessScen.begin(), indiv->eval.capacityExcessScen.end(), 0.0);
				std::fill(indiv->eval.durationExcessScen.begin(), indiv->eval.durationExcessScen.end(), 0.0);
				std::fill(indiv->eval.isFeasibleScen.begin(), indiv->eval.isFeasibleScen.end(), true);
				std::fill(indiv->successors.begin(), indiv->successors.end(), 0);
				std::fill(indiv->predecessors.begin(), indiv->predecessors.end(), 0);
				indiv->chromR_scen.clear();
			}

			const int capacity = splitCuda.getMaxBatchSize();
			for (int selectedCount = 1; selectedCount <= fullNbClients; ++selectedCount)
			{
				auto & group = groups[selectedCount];
				if (group.empty()) continue;

				params.nbClients = selectedCount;
				splitCuda.setActiveClients(selectedCount);
				for (std::size_t begin = 0; begin < group.size(); begin += capacity)
				{
					const std::size_t end = std::min(group.size(), begin + static_cast<std::size_t>(capacity));
					std::vector<Individual*> chunk(group.begin() + begin, group.begin() + end);
					const int actualBatch = static_cast<int>(chunk.size());
					maxActualGpuBatch = std::max(maxActualGpuBatch, actualBatch);
					++gpuBatchLaunches;
					splitCuda.reset_batch(actualBatch);
					splitCuda.preprocess_batch(chunk, params.nbVehicles);
					splitCuda.generate_split_batch(actualBatch);
					splitCuda.evaluateOnGPU_batch(chunk);
				}
			}

			if (useOptional)
			{
				for (Individual * indiv : batch)
				{
					if (indiv->clientVisited.empty()) continue;
					double skipCost = 0.0;
					for (int client = 1; client <= fullNbClients; ++client)
						if (!indiv->clientVisited[client])
							skipCost += params.cli[client].skipPenalty;

					// A first-stage skip cost is paid once in the expected objective,
					// and is present with the same value in each scenario record.
					indiv->eval.penalizedCost += skipCost;
					for (double & scenarioCost : indiv->eval.penalizedCostScen)
						scenarioCost += skipCost;
				}
			}
		};

		std::ofstream traceOutput;
		std::ostream * traceStream = nullptr;
		if (!commandline.traceFile.empty())
		{
			const std::filesystem::path tracePath(commandline.traceFile);
			if (!tracePath.parent_path().empty())
				std::filesystem::create_directories(tracePath.parent_path());
			traceOutput.open(tracePath);
			if (!traceOutput)
				throw std::runtime_error("Cannot open trace file: " + commandline.traceFile);
			traceOutput << "time_s,iteration,evaluated_tours,best_cost\n";
			traceStream = &traceOutput;
		}

		GeneticHGS solver(params, gpuEval, gpuBatchSize, true);
		solver.run(traceStream, commandline.ap.traceEvery);
		std::cout << "GPU_BATCH_STATS launches=" << gpuBatchLaunches
				  << " max_actual_batch=" << maxActualGpuBatch
				  << " capacity=" << gpuBatchSize << std::endl;

		const Individual * best = solver.getBestFound();
		if (best)
		{
			std::cout << "\n===== BEST SOLUTION =====" << std::endl;
			std::cout << "  penalizedCost: " << best->eval.penalizedCost << std::endl;
			std::cout << "  avg distance:  " << best->eval.distance / params.n_scenarios << std::endl;
			std::cout << "  avg capExcess: " << best->eval.capacityExcess / params.n_scenarios << std::endl;
			std::cout << "  isFeasible:    " << best->eval.isFeasible << std::endl;
			std::cout << "  nbRoutes:      " << best->eval.nbRoutes << std::endl;
			std::cout << "  evaluatedTours:" << solver.getEvaluatedTours() << std::endl;
			double tourDist = 0;
			if (!best->chromT.empty()) {
				tourDist += params.timeCost[0][best->chromT[0]];
				for (int i = 0; i + 1 < (int)best->chromT.size(); i++)
					tourDist += params.timeCost[best->chromT[i]][best->chromT[i+1]];
				tourDist += params.timeCost[best->chromT.back()][0];
			}
			std::cout << "  tourDist(TSP): " << tourDist << std::endl;
			std::cout << "  chromT:";
			for (int c : best->chromT) std::cout << " " << c;
			std::cout << std::endl;
			if (params.ap.optionalVisit && !best->clientVisited.empty())
			{
				int nVis = 0;
				for (int c = 1; c <= fullNbClients; c++)
					if (best->clientVisited[c]) nVis++;
				std::cout << "  visited/total: " << nVis << "/" << fullNbClients << std::endl;
			}

			std::ofstream solutionFile(commandline.pathSolution);
			if (!solutionFile)
				throw std::runtime_error("Cannot open solution file: " + commandline.pathSolution);
			solutionFile << std::setprecision(17) << "Tour:";
			for (int c : best->chromT) solutionFile << ' ' << c;
			solutionFile << "\nSelectedTour:";
			for (int c : best->chromT)
				if (best->clientVisited.empty() || best->clientVisited[c])
					solutionFile << ' ' << c;
			solutionFile << "\nSkipped:";
			if (!best->clientVisited.empty())
				for (int c = 1; c <= fullNbClients; ++c)
					if (!best->clientVisited[c]) solutionFile << ' ' << c;
			solutionFile << "\nCost " << best->eval.penalizedCost
						 << "\nScenarios " << params.n_scenarios
						 << "\nScenarioSeed " << params.ap.scenarioSeed
						 << "\nDemandDistribution "
						 << scenario_demand_distribution_name(params.ap.demandDistribution)
						 << "\nPenaltySeed " << params.ap.penaltySeed
						 << "\nSkipPenScale " << params.ap.skipPenScale
						 << "\nAlgoSeed " << params.ap.algoSeed
						 << "\nBatchSize " << commandline.ap.gpuBatchSize
						 << "\nEvaluatedTours " << solver.getEvaluatedTours()
						 << "\nFeasible " << (best->eval.isFeasible ? 1 : 0);
			write_scenario_demand_parameters(solutionFile, params.ap);
			solutionFile << '\n';
		}
		else
			throw std::runtime_error("HGS did not produce a solution");
	}
	catch (const string & e) { std::cerr << "EXCEPTION | " << e << std::endl; return 1; }
	catch (const std::exception & e) { std::cerr << "EXCEPTION | " << e.what() << std::endl; return 1; }

	auto tAllEnd = std::chrono::steady_clock::now();
	double totaltime = std::chrono::duration_cast<std::chrono::nanoseconds>(tAllEnd - tAllStart).count() / 1.e9;
	std::cout << "Total time: " << totaltime << "s" << std::endl;
	return 0;
}
