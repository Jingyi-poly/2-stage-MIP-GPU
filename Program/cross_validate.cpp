#include "commandline.h"
#include "InstanceCVRPLIB.h"
#include "Split.h"
#include "Individual.h"
#include "ScenarioDemandSampler.h"
#include "FixedSkipPenalties.h"
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <omp.h>

using namespace std;

struct TourCertificate
{
	vector<int> tour;
	vector<int> selectedTour;
	vector<int> skipped;
	bool hasTour = false;
	bool hasSelectedTour = false;
	bool hasSkipped = false;
	bool hasPenaltySeed = false;
	bool hasSkipPenScale = false;
	bool hasDemandDistribution = false;
	bool hasFixedRouteCapacity = false;
	int fixedRouteCapacity = 0;
	int penaltySeed = -1;
	int demandDistribution = SCENARIO_DEMAND_LEGACY_MIXTURE;
	double skipPenScale = 1.0;
	std::map<std::string, double> demandParameters;
	std::vector<double> skipPenalties;
};

static vector<int> parseClientList(const string& line, const string& marker)
{
	istringstream values(line.substr(marker.size()));
	vector<int> clients;
	int value = 0;
	while (values >> value)
		clients.push_back(value);
	return clients;
}

static void stripDepotEndpoints(vector<int>& tour)
{
	if (!tour.empty() && tour.front() == 0)
		tour.erase(tour.begin());
	if (!tour.empty() && tour.back() == 0)
		tour.pop_back();
}

static TourCertificate readTourFile(const string& path)
{
	ifstream input(path);
	if (!input)
		throw runtime_error("Cannot open chromT file: " + path);

	TourCertificate certificate;
	string line;
	while (getline(input, line))
	{
		const std::string firstKey = line.substr(0, line.find(' '));
		if (is_scenario_demand_parameter(firstKey))
		{
			istringstream values(line);
			std::string key;
			double value;
			if (!(values >> key >> value))
				throw runtime_error("Invalid demand parameter line in: " + path);
			certificate.demandParameters[key] = value;
		}
		else if (line.rfind("SelectedTour:", 0) == 0)
		{
			certificate.selectedTour = parseClientList(line, "SelectedTour:");
			stripDepotEndpoints(certificate.selectedTour);
			certificate.hasSelectedTour = true;
		}
		else if (line.rfind("FixedRouteCapacity ", 0) == 0)
		{
			istringstream values(line);
			std::string key;
			if (!(values >> key >> certificate.fixedRouteCapacity))
				throw runtime_error("Invalid FixedRouteCapacity line in: " + path);
			certificate.hasFixedRouteCapacity = true;
		}
		else if (line.rfind("Tour:", 0) == 0)
		{
			certificate.tour = parseClientList(line, "Tour:");
			stripDepotEndpoints(certificate.tour);
			certificate.hasTour = true;
		}
		else if (line.rfind("Skipped:", 0) == 0)
		{
			certificate.skipped = parseClientList(line, "Skipped:");
			certificate.hasSkipped = true;
		}
		else if (line.rfind("PenaltySeed ", 0) == 0)
		{
			istringstream values(line);
			string key;
			if (!(values >> key >> certificate.penaltySeed))
				throw runtime_error("Invalid PenaltySeed line in: " + path);
			certificate.hasPenaltySeed = true;
		}
		else if (line.rfind("DemandDistribution ", 0) == 0)
		{
			istringstream values(line);
			string key;
			string distribution;
			if (!(values >> key >> distribution))
				throw runtime_error("Invalid DemandDistribution line in: " + path);
			certificate.demandDistribution =
				parse_scenario_demand_distribution(distribution.c_str());
			certificate.hasDemandDistribution = true;
		}
		else if (line.rfind("SkipPenScale ", 0) == 0)
		{
			istringstream values(line);
			string key;
			if (!(values >> key >> certificate.skipPenScale))
				throw runtime_error("Invalid SkipPenScale line in: " + path);
			certificate.hasSkipPenScale = true;
		}
		else if (line.rfind("SkipPenalties ", 0) == 0)
		{
			istringstream values(line.substr(std::string("SkipPenalties ").size()));
			double value;
			while (values >> value) certificate.skipPenalties.push_back(value);
		}
	}

	// Backward compatibility with the original whitespace-separated integer
	// chromosome files. Tagged certificates always use Tour:/SelectedTour:.
	if (!certificate.hasTour && !certificate.hasSelectedTour)
	{
		input.clear();
		input.seekg(0);
		int value = 0;
		while (input >> value)
			certificate.tour.push_back(value);
		certificate.hasTour = !certificate.tour.empty();
		stripDepotEndpoints(certificate.tour);
	}
	if (!certificate.hasTour && !certificate.hasSelectedTour)
		throw runtime_error("No Tour: certificate or integer chromosome found: " + path);
	return certificate;
}

int main(int argc, char *argv[])
{
	if (argc < 3)
	{
		cerr << "Usage: cross_validate <instance> <tour_file> [-nextrascen N] "
				"[-seed S] [-scenarioSeed S] [-penaltySeed S] [-skipPenScale X] "
				"[-demandDistribution legacy|normal|uniform|lognormal|stress] "
				"[-demandCV X] [-demandUniformSpan X] [-demandStressProb P] "
				"[-demandStressScale X] [-demandCorrelation C] "
				"[-fixedRouteCapacity 0|1] "
				"[-nthreads T] [-maxClient N] [-skipNominal 0|1]" << endl;
		return 1;
	}

	string instancePath = argv[1];
	string chromTPath = argv[2];
	int nextra = 99999;
	int seed = 1;
	int scenarioSeed = -1;
	AlgorithmParameters ap = default_algorithm_parameters();
	std::map<std::string, bool> demandParametersProvided;
	int demandDistribution = SCENARIO_DEMAND_LEGACY_MIXTURE;
	bool demandDistributionProvided = false;
	bool fixedRouteCapacityProvided = false;
	int penaltySeed = -1;
	double skipPenScale = 1.0;
	bool penaltySeedProvided = false;
	bool skipPenScaleProvided = false;
	int nthreads = 8;
	int maxClient = -1;
	bool skipNominal = false;

	for (int i = 3; i < argc; i++)
	{
		string arg = argv[i];
		if (i + 1 >= argc)
		{
			cerr << "Missing value for " << arg << endl;
			return 1;
		}
		if (arg == "-nextrascen") nextra = atoi(argv[++i]);
		else if (arg == "-seed") seed = atoi(argv[++i]);
		else if (arg == "-scenarioSeed") scenarioSeed = atoi(argv[++i]);
		else if (arg == "-fixedRouteCapacity")
		{
			ap.fixedRouteCapacity = atoi(argv[++i]);
			fixedRouteCapacityProvided = true;
		}
		else if (is_scenario_demand_parameter(arg))
		{
			try {
				scenario_demand_parameter(ap, arg) = std::stod(argv[++i]);
				demandParametersProvided["Demand" + arg.substr(7)] = true;
			} catch (const exception& error) {
				cerr << "Invalid " << arg << ": " << error.what() << endl;
				return 1;
			}
		}
		else if (arg == "-demandDistribution" || arg == "-scenarioDist")
		{
			try
			{
				demandDistribution = parse_scenario_demand_distribution(argv[++i]);
				demandDistributionProvided = true;
			}
			catch (const exception& error)
			{
				cerr << error.what() << endl;
				return 1;
			}
		}
		else if (arg == "-penaltySeed")
		{
			penaltySeed = atoi(argv[++i]);
			penaltySeedProvided = true;
		}
		else if (arg == "-skipPenScale")
		{
			skipPenScale = atof(argv[++i]);
			skipPenScaleProvided = true;
		}
		else if (arg == "-nthreads") nthreads = atoi(argv[++i]);
		else if (arg == "-maxClient") maxClient = atoi(argv[++i]);
		else if (arg == "-skipNominal") skipNominal = atoi(argv[++i]) != 0;
		else
		{
			cerr << "Unknown option: " << arg << endl;
			return 1;
		}
	}
	if (nextra < 0 || nthreads < 1 || !isfinite(skipPenScale) || skipPenScale < 0.0)
	{
		cerr << "nextrascen/skipPenScale must be nonnegative and nthreads must be positive" << endl;
		return 1;
	}
	omp_set_num_threads(nthreads);

	TourCertificate certificate;
	try
	{
		certificate = readTourFile(chromTPath);
	}
	catch (const exception& error)
	{
		cerr << error.what() << endl;
		return 1;
	}
	if (!penaltySeedProvided && certificate.hasPenaltySeed)
		penaltySeed = certificate.penaltySeed;
	if (!fixedRouteCapacityProvided && certificate.hasFixedRouteCapacity)
		ap.fixedRouteCapacity = certificate.fixedRouteCapacity;
	else if (fixedRouteCapacityProvided && certificate.hasFixedRouteCapacity && ap.fixedRouteCapacity != certificate.fixedRouteCapacity)
	{
		cerr << "FixedRouteCapacity mismatch between certificate and evaluator" << endl;
		return 1;
	}
	if (!demandDistributionProvided && certificate.hasDemandDistribution)
		demandDistribution = certificate.demandDistribution;
	else if (demandDistributionProvided && certificate.hasDemandDistribution &&
			 demandDistribution != certificate.demandDistribution)
	{
		cerr << "Demand distribution mismatch between solution certificate and evaluator"
			 << endl;
		return 1;
	}
	if (!skipPenScaleProvided && certificate.hasSkipPenScale)
		skipPenScale = certificate.skipPenScale;
	for (const auto& parameter : certificate.demandParameters)
	{
		double& value = scenario_demand_parameter(ap, parameter.first);
		if (!demandParametersProvided[parameter.first])
			value = parameter.second;
		else if (!std::isfinite(parameter.second) || std::abs(value-parameter.second) > 1e-12)
		{
			cerr << "Demand parameter mismatch between certificate and evaluator: " << parameter.first << endl;
			return 1;
		}
	}
	if (!isfinite(skipPenScale) || skipPenScale < 0.0)
	{
		cerr << "SkipPenScale in the solution certificate must be finite and nonnegative" << endl;
		return 1;
	}
	if (skipNominal && demandDistribution != SCENARIO_DEMAND_LEGACY_MIXTURE)
	{
		cerr << "-skipNominal is only valid for demandDistribution=legacy; "
			 << "normal and uniform contain no nominal scenario" << endl;
		return 1;
	}

	ap.seed = seed;
	ap.scenarioSeed = scenarioSeed >= 0 ? scenarioSeed : seed;
	ap.demandDistribution = demandDistribution;
	ap.penaltySeed = penaltySeed;
	ap.optionalVisit = certificate.hasSelectedTour ? 1 : 0;
	ap.skipPenScale = skipPenScale;
	ap.n_extra_senarios = nextra;
	ap.maxClient = maxClient;
	try { validate_scenario_demand_parameters(ap); }
	catch (const exception& error) { cerr << error.what() << endl; return 1; }

	InstanceCVRPLIB cvrp(instancePath, true);
	Params params(cvrp.x_coords, cvrp.y_coords, cvrp.dist_mtx, cvrp.service_time, cvrp.demands,
				  cvrp.vehicleCapacity, cvrp.durationLimit, INT_MAX,
				  cvrp.isDurationConstraint, true, ap);

	params.generate_scenario_demands(nextra);
	params.update_max_vehi();
	params.apply_fixed_route_capacity();
	params.generate_skip_penalties();
	try { apply_fixed_skip_penalties(params); }
	catch (const exception& error) { cerr << error.what() << endl; return 1; }
	if (!certificate.skipPenalties.empty())
	{
		if (certificate.skipPenalties.size() != static_cast<size_t>(params.nbClients))
		{
			cerr << "SkipPenalties certificate length mismatch" << endl;
			return 1;
		}
		for (int client = 1; client <= params.nbClients; ++client)
			if (std::abs(certificate.skipPenalties[client - 1] - params.cli[client].skipPenalty) > 1e-9)
			{
				cerr << "SkipPenalties certificate value mismatch" << endl;
				return 1;
			}
	}

	cout << "Instance loaded: " << params.nbClients << " clients, "
		 << params.n_scenarios << " scenarios, "
		 << params.nbVehicles << " vehicles, demand_distribution="
		 << scenario_demand_distribution_name(params.ap.demandDistribution) << endl;

	const int fullNbClients = params.nbClients;
	auto validateUniqueClients = [fullNbClients](const vector<int>& clients, const string& label) {
		vector<bool> seen(fullNbClients + 1, false);
		for (int client : clients)
		{
			if (client <= 0 || client > fullNbClients || seen[client])
				throw runtime_error(label + " contains an out-of-range or duplicate client");
			seen[client] = true;
		}
	};

	try
	{
		if (certificate.hasTour)
		{
			validateUniqueClients(certificate.tour, "Tour");
			if ((int)certificate.tour.size() != fullNbClients)
				throw runtime_error("Tour is not a full permutation of clients 1.." +
					to_string(fullNbClients));
		}
		validateUniqueClients(certificate.hasSelectedTour
			? certificate.selectedTour : certificate.tour,
			certificate.hasSelectedTour ? "SelectedTour" : "Tour");
	}
	catch (const exception& error)
	{
		cerr << "ERROR: " << error.what() << endl;
		return 1;
	}

	const vector<int>& chromT = certificate.hasSelectedTour
		? certificate.selectedTour : certificate.tour;
	vector<bool> isSelected(fullNbClients + 1, false);
	for (int client : chromT) isSelected[client] = true;
	vector<int> computedSkipped;
	for (int client = 1; client <= fullNbClients; ++client)
		if (!isSelected[client]) computedSkipped.push_back(client);

	if (certificate.hasSelectedTour && certificate.hasTour)
	{
		size_t nextSelected = 0;
		for (int client : certificate.tour)
			if (nextSelected < chromT.size() && client == chromT[nextSelected])
				++nextSelected;
		if (nextSelected != chromT.size())
		{
			cerr << "ERROR: SelectedTour is not in the visit order recorded by Tour" << endl;
			return 1;
		}
	}
	if (certificate.hasSkipped)
	{
		try { validateUniqueClients(certificate.skipped, "Skipped"); }
		catch (const exception& error)
		{
			cerr << "ERROR: " << error.what() << endl;
			return 1;
		}
		vector<int> declaredSkipped = certificate.skipped;
		sort(declaredSkipped.begin(), declaredSkipped.end());
		if (declaredSkipped != computedSkipped)
		{
			cerr << "ERROR: Skipped is not the complement of SelectedTour" << endl;
			return 1;
		}
	}

	double skipCost = 0.0;
	for (int client : computedSkipped)
		skipCost += params.cli[client].skipPenalty;
	cout << "Solution certificate: selected=" << chromT.size()
		 << " skipped=" << computedSkipped.size()
		 << " skipCost=" << setprecision(17) << skipCost
		 << " penaltySeed=" << params.ap.penaltySeed
		 << " skipPenScale=" << params.ap.skipPenScale << endl;

	const int firstScenario = skipNominal ? 1 : 0;
	const int heldoutScenarios = params.n_scenarios - firstScenario;
	if (heldoutScenarios <= 0)
	{
		cerr << "ERROR: -skipNominal leaves no held-out scenarios" << endl;
		return 1;
	}

	Individual indiv(params, false);
	indiv.chromT = chromT;

	cout << "Evaluating with CPU Split (OpenMP " << nthreads << " threads, "
		 << params.n_scenarios << " scenarios)..." << endl;

	auto t0 = chrono::steady_clock::now();

	if (chromT.empty())
	{
		indiv.eval = EvalIndivMultiScen();
		indiv.resetEval(params);
		indiv.eval.isFeasible = true;
		fill(indiv.eval.isFeasibleScen.begin(), indiv.eval.isFeasibleScen.end(), true);
	}
	else
	{
		params.nbClients = static_cast<int>(chromT.size());
		#pragma omp parallel for
		for (int s = 0; s < params.n_scenarios; s++)
		{
			Split split(params);
			split.generalSplit(indiv, params.nbVehicles, s);
			split.generateChromR(indiv, s);
		}
		indiv.evaluateCompleteCost(params);
		params.nbClients = fullNbClients;
	}
	for (int s = 0; s < params.n_scenarios; ++s)
		indiv.eval.penalizedCostScen[s] += skipCost;
	indiv.eval.penalizedCost += skipCost;

	auto t1 = chrono::steady_clock::now();
	double evalTime = chrono::duration_cast<chrono::microseconds>(t1 - t0).count() / 1e6;

	cout << "\n===== CPU SPLIT EVALUATION =====" << endl;
	cout << "  penalizedCost: " << indiv.eval.penalizedCost << endl;
	cout << "  avg distance:  " << indiv.eval.distance / params.n_scenarios << endl;
	cout << "  avg capExcess: " << indiv.eval.capacityExcess / params.n_scenarios << endl;
	cout << "  avg durExcess: " << indiv.eval.durationExcess / params.n_scenarios << endl;
	cout << "  isFeasible:    " << indiv.eval.isFeasible << endl;
	cout << "  nbRoutes:      " << indiv.eval.nbRoutes << endl;
	cout << "  eval time:     " << evalTime << "s" << endl;
	double heldoutMean = 0.0;
	for (int s = firstScenario; s < params.n_scenarios; ++s)
		heldoutMean += indiv.eval.penalizedCostScen[s];
	heldoutMean /= heldoutScenarios;
	double heldoutSquaredDeviation = 0.0;
	for (int s = firstScenario; s < params.n_scenarios; ++s)
	{
		const double deviation = indiv.eval.penalizedCostScen[s] - heldoutMean;
		heldoutSquaredDeviation += deviation * deviation;
	}
	const double heldoutStddev = heldoutScenarios > 1
		? sqrt(heldoutSquaredDeviation / (heldoutScenarios - 1))
		: 0.0;
	const double heldoutSE = heldoutStddev / sqrt(static_cast<double>(heldoutScenarios));
	cout << setprecision(17)
		 << "OOS_RESULT mean=" << heldoutMean
		 << " stddev=" << heldoutStddev
		 << " se=" << heldoutSE
		 << " scenarios=" << heldoutScenarios
		 << " selected=" << chromT.size()
		 << " skipped=" << computedSkipped.size()
			 << " skip_cost=" << skipCost
			 << " penalty_seed=" << params.ap.penaltySeed
			 << " skip_pen_scale=" << params.ap.skipPenScale
			 << " demand_distribution="
			 << scenario_demand_distribution_name(params.ap.demandDistribution) << endl;

	// ── O(N²) exact Split DP (same as PCTSP) for comparison ──
	auto splitDP_exact = [&](const vector<int> & perm, int scenIdx) -> double {
		int sz = (int)perm.size();
		if (sz == 0) return 0.0;
		const double cap = params.vehicleCapacity;
		const double penCap = params.penaltyCapacity;
		vector<double> cost(sz + 1, 1e30);
		cost[0] = 0.0;
		for (int j = 1; j <= sz; j++) {
			double load = 0.0, routeDist = 0.0;
			for (int i = j; i >= 1; i--) {
				int cli = perm[i - 1];
				load += params.cli[cli].demands_scenarios[scenIdx];
				if (i == j) routeDist = params.timeCost[cli][0];
				else routeDist += params.timeCost[cli][perm[i]];
				double fullDist = params.timeCost[0][cli] + routeDist;
				double excess = max(0.0, load - cap);
				double segCost = fullDist + penCap * excess;
				if (cost[i - 1] + segCost < cost[j])
					cost[j] = cost[i - 1] + segCost;
			}
		}
		return cost[sz];
	};

	cout << "\nEvaluating with O(N^2) exact Split DP..." << endl;
	auto t2 = chrono::steady_clock::now();
	double totalExact = 0.0;
	#pragma omp parallel for reduction(+:totalExact)
	for (int s = firstScenario; s < params.n_scenarios; s++)
		totalExact += splitDP_exact(chromT, s) + skipCost;
	double avgExact = totalExact / heldoutScenarios;
	auto t3 = chrono::steady_clock::now();
	double exactTime = chrono::duration_cast<chrono::microseconds>(t3 - t2).count() / 1e6;

	cout << "  O(N^2) penalizedCost: " << avgExact << endl;
	cout << "  O(N)   penalizedCost: " << heldoutMean << endl;
	cout << "  Difference:           " << (heldoutMean - avgExact)
		 << " (" << (avgExact == 0.0 ? 0.0 : 100.0 * (heldoutMean - avgExact) / avgExact)
		 << "%)" << endl;
	cout << "  O(N^2) eval time:     " << exactTime << "s" << endl;

	// Per-scenario comparison for first 10
	cout << "\nPer-scenario comparison (first 10):" << endl;
	cout << "  scen |  O(N) HGS  |  O(N^2) Exact | Diff" << endl;
	int nDiff = 0;
	for (int s = firstScenario;
		 s < min(params.n_scenarios, firstScenario + 10); s++)
	{
		double hgsCost = indiv.eval.penalizedCostScen[s];
		double exactCost = splitDP_exact(chromT, s) + skipCost;
		double diff = hgsCost - exactCost;
		if (abs(diff) > 1e-6) nDiff++;
		cout << "  " << s << "    | " << hgsCost << " | " << exactCost
			 << " | " << diff << (abs(diff) > 1e-6 ? " ***" : "") << endl;
	}

	// Count total mismatches
	int totalDiff = 0;
	#pragma omp parallel for reduction(+:totalDiff)
	for (int s = firstScenario; s < params.n_scenarios; s++)
	{
		double hgsCost = indiv.eval.penalizedCostScen[s];
		double exactCost = splitDP_exact(chromT, s) + skipCost;
		if (abs(hgsCost - exactCost) > 1e-6) totalDiff++;
	}
	cout << "\nScenarios with different Split results: " << totalDiff
		 << "/" << heldoutScenarios
		 << " (" << 100.0 * totalDiff / heldoutScenarios << "%)" << endl;

	return 0;
}
