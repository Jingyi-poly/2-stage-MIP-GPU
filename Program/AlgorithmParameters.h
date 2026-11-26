//
// Created by chkwon on 3/23/22.
//

// This header file must be readable in C.

#ifndef ALGORITHMPARAMETERS_H
#define ALGORITHMPARAMETERS_H

enum ScenarioDemandDistribution {
	SCENARIO_DEMAND_LEGACY_MIXTURE = 0,
	SCENARIO_DEMAND_CENSORED_NORMAL_CV20 = 1,
	SCENARIO_DEMAND_HIGH_VARIANCE_UNIFORM = 2,
	SCENARIO_DEMAND_LOGNORMAL = 3,
	SCENARIO_DEMAND_CORRELATED_STRESS = 4
};

struct AlgorithmParameters {
	int nbGranular;			// Granular search parameter, limits the number of moves in the RI local search
	int mu;					// Minimum population size
	int lambda;				// Number of solutions created before reaching the maximum population size (i.e., generation size)
	int nbElite;			// Number of elite individuals
	int nbClose;			// Number of closest solutions/individuals considered when calculating diversity contribution

	int nbIterPenaltyManagement;  // Number of iterations between penalty updates
	double targetFeasible;	      // Reference proportion for the number of feasible individuals, used for the adaptation of the penalty parameters
	double penaltyDecrease;	      // Multiplier used to decrease penalty parameters if there are sufficient feasible individuals
	double penaltyIncrease;	      // Multiplier used to increase penalty parameters if there are insufficient feasible individuals

	int seed;				// Random seed. Default value: 0
	int scenarioSeed;		// Scenario RNG seed. A negative value falls back to seed
	int demandDistribution;	// ScenarioDemandDistribution value. Defaults to the legacy generator
	int penaltySeed;		// Skip-penalty RNG seed. A negative value falls back to scenarioSeed/seed
	int algoSeed;			// HGS RNG seed. A negative value falls back to seed
	int nbIter;				// Nb iterations without improvement until termination (or restart if a time limit is specified). Default value: 20,000 iterations
	int nbIterTraces;       // Number of iterations between traces display during HGS execution
	double timeLimit;		// CPU time limit until termination in seconds. Default value: 0 (i.e., inactive)
	int useSwapStar;		// Use SWAP* local search or not. Default value: 1. Only available when coordinates are provided.

	int deviceId;		// Device ID. Default value: 0 (cpu)
	int nthreads;		// max number of parallel for CPU
	int n_extra_senarios;
	int freqPrint;
	int iterLim;
	int timeLim;
	int maxClient;

	int optionalVisit;
	double skipPenScale;
	int exportScenarios;
	int gpuBatchSize;
	int traceEvery;		// Iteration interval for an explicitly requested trace file
	int gpuMemoryOnly;	// CUDA frontend: stop after allocating and reporting buffers
	double demandCV;		// Normal/lognormal pre-rounding coefficient of variation
	double demandUniformSpan; // Uniform endpoints d_i*(1-span), d_i*(1+span)
	double demandStressProb; // Marginal rare-stress probability
	double demandStressScale; // Stress multiplier; calm state is mean matched
	double demandCorrelation; // Convex weight on common vs independent stress event
	int fixedRouteCapacity; // Opt-in unlimited soft-capacity route storage fixed at nbClients
};

#ifdef __cplusplus
extern "C"
#endif
struct AlgorithmParameters default_algorithm_parameters();

#ifdef __cplusplus
void print_algorithm_parameters(const AlgorithmParameters & ap);
const char * scenario_demand_distribution_name(int distribution);
int parse_scenario_demand_distribution(const char * value);
void validate_scenario_demand_parameters(const AlgorithmParameters & ap);
#endif

#endif //ALGORITHMPARAMETERS_H
