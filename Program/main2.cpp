#include "GeneticHGS.h"
#include "ScenarioDemandSampler.h"
#include "commandline.h"
#include "Split.h"
#include "InstanceCVRPLIB.h"
#include <chrono>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <omp.h>

using namespace std;

int main(int argc, char *argv[])
{
	auto tAllStart = std::chrono::steady_clock::now();

	try
	{
		CommandLine commandline(argc, argv);
		int n_threads = commandline.ap.nthreads;
		omp_set_num_threads(n_threads);

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
		std::cout << "Finished generating scenarios  (n_scen=" << params.n_scenarios
				  << "  n_cli=" << params.nbClients << ")  [CPU mode, T" << n_threads << "]" << std::endl;

		// CPU evaluation function — OpenMP parallel Split across scenarios
		auto cpuEval = [&params, n_threads, fullNbClients](Individual & indiv) {
			std::vector<int> fullChromT;
			bool useOptional = params.ap.optionalVisit && !indiv.clientVisited.empty();
			int nVisited = fullNbClients;

			if (useOptional)
			{
				fullChromT = indiv.chromT;
				indiv.chromT.clear();
				for (int c : fullChromT)
					if (indiv.clientVisited[c])
						indiv.chromT.push_back(c);
				nVisited = (int)indiv.chromT.size();
				if (nVisited == 0)
				{
					indiv.eval = EvalIndivMultiScen();
					indiv.resetEval(params);
					double skipCost = 0.0;
					for (int c = 1; c <= fullNbClients; c++)
						skipCost += params.cli[c].skipPenalty;
					indiv.eval.penalizedCost = skipCost;
					indiv.chromT = fullChromT;
					return;
				}
				params.nbClients = nVisited;
			}

			if (n_threads > 1)
			{
				#pragma omp parallel for
				for (int s = 0; s < params.n_scenarios; s++)
				{
					Split split(params);
					split.generalSplit(indiv, params.nbVehicles, s);
					split.generateChromR(indiv, s);
				}
			}
			else
			{
				for (int s = 0; s < params.n_scenarios; s++)
				{
					Split split(params);
					split.generalSplit(indiv, params.nbVehicles, s);
					split.generateChromR(indiv, s);
				}
			}
			indiv.evaluateCompleteCost(params);

			if (useOptional)
			{
				double skipCost = 0.0;
				for (int c = 1; c <= fullNbClients; c++)
					if (!indiv.clientVisited[c])
						skipCost += params.cli[c].skipPenalty;
				indiv.eval.penalizedCost += skipCost;
				params.nbClients = fullNbClients;
				indiv.chromT = fullChromT;
			}
		};

		auto cpuBatchEval = [&cpuEval](std::vector<Individual*>& batch) {
			for (auto* indiv : batch)
				cpuEval(*indiv);
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

		GeneticHGS solver(params, cpuBatchEval, commandline.ap.gpuBatchSize, false);
		solver.run(traceStream, commandline.ap.traceEvery);

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
				if (!params.ap.optionalVisit || best->clientVisited.empty() || best->clientVisited[c])
					solutionFile << ' ' << c;
			solutionFile << "\nSkipped:";
			if (params.ap.optionalVisit && !best->clientVisited.empty())
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
