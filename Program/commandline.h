/*MIT License

Copyright(c) 2020 Thibaut Vidal

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include <iostream>
#include <string>
#include <climits>
#include "AlgorithmParameters.h"

class CommandLine
{
public:
	AlgorithmParameters ap = default_algorithm_parameters();

	int nbVeh		 = INT_MAX;		// Number of vehicles. Default value: infinity
	std::string pathInstance;		// Instance path
	std::string pathSolution;		// Solution path
	std::string traceFile;			// Optional structured convergence trace
	bool verbose     = true;
	bool isRoundingInteger = true;

	// Reads the line of command and extracts possible options
	CommandLine(int argc, char* argv[])
	{
		if (argc % 2 != 1 || argc < 3)
		{
			std::cout << "----- NUMBER OF COMMANDLINE ARGUMENTS IS INCORRECT: " << argc << std::endl;
			display_help(); throw std::string("Incorrect line of command");
		}
		else
		{
			pathInstance = std::string(argv[1]);
			pathSolution = std::string(argv[2]);
			for (int i = 3; i < argc; i += 2)
			{
				if (std::string(argv[i]) == "-t")
					ap.timeLimit = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-it")
					ap.nbIter  = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-seed")
					ap.seed    = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-scenarioSeed")
					ap.scenarioSeed = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-demandDistribution" ||
						 std::string(argv[i]) == "-scenarioDist")
					ap.demandDistribution = parse_scenario_demand_distribution(argv[i+1]);
				else if (std::string(argv[i]) == "-demandCV")
					ap.demandCV = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-demandUniformSpan")
					ap.demandUniformSpan = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-demandStressProb")
					ap.demandStressProb = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-demandStressScale")
					ap.demandStressScale = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-demandCorrelation")
					ap.demandCorrelation = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-fixedRouteCapacity")
					ap.fixedRouteCapacity = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-penaltySeed")
					ap.penaltySeed = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-algoSeed")
					ap.algoSeed = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-veh")
					nbVeh = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-round")
					isRoundingInteger = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-log")
					verbose = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbGranular")
					ap.nbGranular = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-mu")
					ap.mu = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-lambda")
					ap.lambda = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbElite")
					ap.nbElite = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbClose")
					ap.nbClose = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbIterPenaltyManagement")
					ap.nbIterPenaltyManagement = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbIterTraces")
					ap.nbIterTraces = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-targetFeasible")
					ap.targetFeasible = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-penaltyIncrease")
					ap.penaltyIncrease = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-nthreads")
					ap.nthreads = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nextrascen")
					ap.n_extra_senarios = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-deviceId")
					ap.deviceId = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-freqPrint")
					ap.freqPrint = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-iterLim")
					ap.iterLim = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-timeLim")
					ap.timeLim = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-maxClient")
					ap.maxClient = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-optionalVisit")
					ap.optionalVisit = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-skipPenScale")
					ap.skipPenScale = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-exportScenarios")
					ap.exportScenarios = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-gpuBatchSize")
					ap.gpuBatchSize = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-traceFile")
					traceFile = argv[i+1];
				else if (std::string(argv[i]) == "-traceEvery")
					ap.traceEvery = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-gpuMemoryOnly")
					ap.gpuMemoryOnly = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-penaltyDecrease")
					ap.penaltyDecrease = atof(argv[i+1]);
				else
				{
					std::cout << "----- ARGUMENT NOT RECOGNIZED: " << std::string(argv[i]) << std::endl;
					display_help(); throw std::string("Incorrect line of command");
				}
			}
			if (ap.gpuBatchSize < 1)
				throw std::string("gpuBatchSize must be positive");
			if (ap.traceEvery < 1)
				throw std::string("traceEvery must be positive");
			validate_scenario_demand_parameters(ap);
			if (ap.scenarioSeed < 0)
				ap.scenarioSeed = ap.seed;
			if (ap.penaltySeed < 0)
				ap.penaltySeed = ap.scenarioSeed;
			if (ap.algoSeed < 0)
				ap.algoSeed = ap.seed;
		}
	}

	// Printing information about how to use the code
	void display_help()
	{
		std::cout << std::endl;
		std::cout << "-------------------------------------------------- HGS-CVRP algorithm (2020) ---------------------------------------------------" << std::endl;
		std::cout << "Call with: ./hgs instancePath solPath [-iterLim nbIter] [-timeLim seconds] [-seed mySeed] [-veh nbVehicles] [-log verbose]    " << std::endl;
		std::cout << "[-fixedRouteCapacity 0|1] Opt-in unrestricted soft-capacity route storage fixed at the number of clients; defaults to 0" << std::endl;
		std::cout << "[-it <int>] sets a maximum number of iterations without improvement. Defaults to 20,000                                         " << std::endl;
		std::cout << "[-timeLim <int>] sets the wall-clock limit used by current GeneticHGS and exact frontends                                   " << std::endl;
		std::cout << "[-t <double>] is retained for the legacy HGS-CVRP driver and is not used by current GeneticHGS frontends                       " << std::endl;
		std::cout << "[-seed <int>] sets the legacy fallback seed for scenarios and HGS. Defaults to 0                                                " << std::endl;
		std::cout << "[-scenarioSeed <int>] independently seeds generated scenarios; negative falls back to -seed                                   " << std::endl;
		std::cout << "[-demandDistribution legacy|normal|uniform|lognormal|stress] selects demand model; only legacy contains a nominal row" << std::endl;
		std::cout << "[-demandCV X] normal/lognormal CV; [-demandUniformSpan X] symmetric relative span in (0,1)" << std::endl;
		std::cout << "[-demandStressProb P] [-demandStressScale X] [-demandCorrelation C] rare stress probability/scale/common-event weight" << std::endl;
		std::cout << "[-penaltySeed <int>] independently seeds optional-visit penalties; negative falls back to -scenarioSeed/-seed                   " << std::endl;
		std::cout << "[-algoSeed <int>] independently seeds the HGS search; negative falls back to -seed                                             " << std::endl;
		std::cout << "[-traceFile <path>] writes a structured HGS or exact-solver convergence CSV when explicitly provided                             " << std::endl;
		std::cout << "[-traceEvery <int>] records every N completed HGS iterations. Exact solvers record every outer iteration. Defaults to 1          " << std::endl;
		std::cout << "[-gpuMemoryOnly <bool>] hgs_cuda reports its persistent device allocation and exits before HGS                                 " << std::endl;
		std::cout << "[-veh <int>] sets a prescribed fleet size. Otherwise a reasonable UB on the the fleet size is calculated                        " << std::endl;
		std::cout << "[-round <bool>] rounding the distance to the nearest integer or not. It can be 0 (not rounding) or 1 (rounding). Defaults to 1. " << std::endl;
		std::cout << "[-log <bool>] sets the verbose level of the algorithm log. It can be 0 or 1. Defaults to 1.                                     " << std::endl;
		std::cout << std::endl;
		std::cout << "Additional Arguments:                                                                                                           " << std::endl;
		std::cout << "[-nbIterTraces <int>] Number of iterations between traces display during HGS execution. Defaults to 500                         " << std::endl;
		std::cout << "[-nbGranular <int>] Granular search parameter, limits the number of moves in the RI local search. Defaults to 20                " << std::endl;
		std::cout << "[-mu <int>] Minimum population size. Defaults to 25                                                                             " << std::endl;
		std::cout << "[-lambda <int>] Number of solutions created before reaching the maximum population size (i.e., generation size). Defaults to 40 " << std::endl;
		std::cout << "[-nbElite <int>] Number of elite individuals. Defaults to 4                                                                     " << std::endl;
		std::cout << "[-nbClose <int>] Number of closest solutions/individuals considered when calculating diversity contribution. Defaults to 5      " << std::endl;
		std::cout << "[-nbIterPenaltyManagement <int>] Number of iterations between penalty updates. Defaults to 100                                  " << std::endl;
		std::cout << "[-targetFeasible <double>] target ratio of feasible individuals between penalty updates. Defaults to 0.2                        " << std::endl;
		std::cout << "[-penaltyIncrease <double>] penalty increase if insufficient feasible individuals between penalty updates. Defaults to 1.2      " << std::endl;
		std::cout << "[-penaltyDecrease <double>] penalty decrease if sufficient feasible individuals between penalty updates. Defaults to 0.85       " << std::endl;
		std::cout << "--------------------------------------------------------------------------------------------------------------------------------" << std::endl;
		std::cout << std::endl;
	};
};
#endif
