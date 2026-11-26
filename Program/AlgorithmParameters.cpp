//
// Created by chkwon on 3/23/22.
//

#include "AlgorithmParameters.h"
#include <iostream>
#include <stdexcept>
#include <string>
#include <cmath>

extern "C"
struct AlgorithmParameters default_algorithm_parameters() {
	struct AlgorithmParameters ap{};

	ap.nbGranular = 20;
	ap.mu = 25;
	ap.lambda = 40;
	ap.nbElite = 4;
	ap.nbClose = 5;

	ap.nbIterPenaltyManagement = 100;
	ap.targetFeasible = 0.2;
	ap.penaltyDecrease = 0.85;
	ap.penaltyIncrease = 1.2;

	ap.seed = 0;
	ap.scenarioSeed = -1;
	ap.demandDistribution = SCENARIO_DEMAND_LEGACY_MIXTURE;
	ap.penaltySeed = -1;
	ap.algoSeed = -1;
	ap.nbIter = 20000;
	ap.nbIterTraces = 500;
	ap.timeLimit = 0;
	ap.useSwapStar = 1;
	ap.deviceId = 0;
	ap.nthreads = 1;
	ap.n_extra_senarios = 0;


	ap.freqPrint = 500;
	ap.iterLim = 1;
	ap.timeLim = -1;
	ap.maxClient = -1;

	ap.optionalVisit = 0;
	ap.skipPenScale = 1.0;
	ap.exportScenarios = 0;
	ap.gpuBatchSize = 16;
	ap.traceEvery = 1;
	ap.gpuMemoryOnly = 0;
	ap.demandCV = 0.2;
	ap.demandUniformSpan = 0.9;
	ap.demandStressProb = 0.05;
	ap.demandStressScale = 4.0;
	ap.demandCorrelation = 0.8;
	ap.fixedRouteCapacity = 0;

	return ap;
}

void print_algorithm_parameters(const AlgorithmParameters & ap)
{
	std::cout << "=========== Algorithm Parameters =================" << std::endl;
	std::cout << "---- nbGranular              is set to " << ap.nbGranular << std::endl;
	std::cout << "---- mu                      is set to " << ap.mu << std::endl;
	std::cout << "---- lambda                  is set to " << ap.lambda << std::endl;
	std::cout << "---- nbElite                 is set to " << ap.nbElite << std::endl;
	std::cout << "---- nbClose                 is set to " << ap.nbClose << std::endl;
	std::cout << "---- nbIterPenaltyManagement is set to " << ap.nbIterPenaltyManagement << std::endl;
	std::cout << "---- targetFeasible          is set to " << ap.targetFeasible << std::endl;
	std::cout << "---- penaltyDecrease         is set to " << ap.penaltyDecrease << std::endl;
	std::cout << "---- penaltyIncrease         is set to " << ap.penaltyIncrease << std::endl;
	std::cout << "---- seed                    is set to " << ap.seed << std::endl;
	std::cout << "---- scenarioSeed            is set to " << ap.scenarioSeed << std::endl;
	std::cout << "---- demandDistribution      is set to "
		<< scenario_demand_distribution_name(ap.demandDistribution) << std::endl;
	std::cout << "---- penaltySeed             is set to " << ap.penaltySeed << std::endl;
	std::cout << "---- algoSeed                is set to " << ap.algoSeed << std::endl;
	std::cout << "---- nbIter                  is set to " << ap.nbIter << std::endl;
	std::cout << "---- nbIterTraces            is set to " << ap.nbIterTraces << std::endl;
	std::cout << "---- timeLimit               is set to " << ap.timeLimit << std::endl;
	std::cout << "---- useSwapStar             is set to " << ap.useSwapStar << std::endl;
	std::cout << "==================================================" << std::endl;
}

const char * scenario_demand_distribution_name(int distribution)
{
	switch (distribution)
	{
	case SCENARIO_DEMAND_LEGACY_MIXTURE:
		return "legacy_mixture";
	case SCENARIO_DEMAND_CENSORED_NORMAL_CV20:
		return "censored_normal_cv20";
	case SCENARIO_DEMAND_HIGH_VARIANCE_UNIFORM:
		return "high_variance_uniform_10_190";
	case SCENARIO_DEMAND_LOGNORMAL:
		return "mean_matched_lognormal";
	case SCENARIO_DEMAND_CORRELATED_STRESS:
		return "mean_matched_correlated_stress";
	default:
		return "invalid";
	}
}

int parse_scenario_demand_distribution(const char * value)
{
	const std::string name(value == nullptr ? "" : value);
	if (name == "legacy" || name == "legacy_mixture")
		return SCENARIO_DEMAND_LEGACY_MIXTURE;
	if (name == "normal" || name == "censored_normal_cv20")
		return SCENARIO_DEMAND_CENSORED_NORMAL_CV20;
	if (name == "uniform" || name == "high_variance_uniform_10_190")
		return SCENARIO_DEMAND_HIGH_VARIANCE_UNIFORM;
	if (name == "lognormal" || name == "mean_matched_lognormal")
		return SCENARIO_DEMAND_LOGNORMAL;
	if (name == "stress" || name == "mean_matched_correlated_stress")
		return SCENARIO_DEMAND_CORRELATED_STRESS;
	throw std::invalid_argument(
		"Unknown demand distribution '" + name +
		"' (expected legacy, normal, uniform, lognormal, or stress)");
}

void validate_scenario_demand_parameters(const AlgorithmParameters & ap)
{
	if (ap.fixedRouteCapacity != 0 && ap.fixedRouteCapacity != 1)
		throw std::invalid_argument("fixedRouteCapacity must be 0 or 1");
	if (ap.demandDistribution < SCENARIO_DEMAND_LEGACY_MIXTURE ||
		ap.demandDistribution > SCENARIO_DEMAND_CORRELATED_STRESS)
		throw std::invalid_argument("demandDistribution is invalid");
	if (!std::isfinite(ap.demandCV) || ap.demandCV < 0.0 || ap.demandCV > 5.0)
		throw std::invalid_argument("demandCV must be finite and in [0,5]");
	if (!std::isfinite(ap.demandUniformSpan) || ap.demandUniformSpan <= 0.0 || ap.demandUniformSpan >= 1.0)
		throw std::invalid_argument("demandUniformSpan must be finite and in (0,1)");
	if (!std::isfinite(ap.demandStressProb) || ap.demandStressProb <= 0.0 || ap.demandStressProb >= 1.0 ||
		!std::isfinite(ap.demandStressScale) || ap.demandStressScale <= 1.0 ||
		ap.demandStressProb * ap.demandStressScale >= 1.0)
		throw std::invalid_argument("stress requires 0<p<1, scale>1, and p*scale<1 for positive mean-matched calm demand");
	if (!std::isfinite(ap.demandCorrelation) || ap.demandCorrelation < 0.0 || ap.demandCorrelation > 1.0)
		throw std::invalid_argument("demandCorrelation must be finite and in [0,1] (common-event weight)");
}
