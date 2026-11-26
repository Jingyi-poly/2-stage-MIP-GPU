#ifndef SCENARIO_DEMAND_SAMPLER_H
#define SCENARIO_DEMAND_SAMPLER_H

#include "AlgorithmParameters.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <ostream>
#include <random>
#include <stdexcept>
#include <string>

inline bool is_scenario_demand_parameter(const std::string& key) {
    return key == "DemandCV" || key == "-demandCV" ||
           key == "DemandUniformSpan" || key == "-demandUniformSpan" ||
           key == "DemandStressProb" || key == "-demandStressProb" ||
           key == "DemandStressScale" || key == "-demandStressScale" ||
           key == "DemandCorrelation" || key == "-demandCorrelation";
}

inline double& scenario_demand_parameter(AlgorithmParameters& ap, const std::string& key) {
    if (key == "DemandCV" || key == "-demandCV") return ap.demandCV;
    if (key == "DemandUniformSpan" || key == "-demandUniformSpan") return ap.demandUniformSpan;
    if (key == "DemandStressProb" || key == "-demandStressProb") return ap.demandStressProb;
    if (key == "DemandStressScale" || key == "-demandStressScale") return ap.demandStressScale;
    if (key == "DemandCorrelation" || key == "-demandCorrelation") return ap.demandCorrelation;
    throw std::invalid_argument("Unknown demand parameter " + key);
}

// Fixed-parameter row-major samplers ensure same-seed N runs are strict
// prefixes of larger runs. Mean matching is before integer rounding and
// the lower censor at one; no extra deterministic nominal row is introduced.
class ScenarioDemandSampler {
    const AlgorithmParameters& ap;
    std::mt19937& rng;
    std::normal_distribution<double> normal{0.0, 1.0};
    std::bernoulli_distribution stress;
    double sharedMultiplier = 1.0;
    double calmMultiplier;
public:
    ScenarioDemandSampler(const AlgorithmParameters& settings, std::mt19937& generator)
        : ap(settings), rng(generator), stress(settings.demandStressProb),
          calmMultiplier((1.0-settings.demandStressProb*settings.demandStressScale)/(1.0-settings.demandStressProb))
    { validate_scenario_demand_parameters(ap); }

    void beginScenario() {
        if (ap.demandDistribution == SCENARIO_DEMAND_CORRELATED_STRESS)
            sharedMultiplier = stress(rng) ? ap.demandStressScale : calmMultiplier;
    }

    int sample(double nominal) {
        double draw = nominal;
        switch (ap.demandDistribution) {
        case SCENARIO_DEMAND_CENSORED_NORMAL_CV20:
            draw *= 1.0 + ap.demandCV * normal(rng);
            break;
        case SCENARIO_DEMAND_HIGH_VARIANCE_UNIFORM: {
            const int low = std::max(1, static_cast<int>(std::ceil((1.0-ap.demandUniformSpan)*nominal)));
            const int high = std::max(low, static_cast<int>(std::floor((1.0+ap.demandUniformSpan)*nominal)));
            return std::uniform_int_distribution<int>(low, high)(rng);
        }
        case SCENARIO_DEMAND_LOGNORMAL: {
            const double sigma2 = std::log1p(ap.demandCV*ap.demandCV);
            draw *= std::exp(std::sqrt(sigma2)*normal(rng)-0.5*sigma2);
            break;
        }
        case SCENARIO_DEMAND_CORRELATED_STRESS: {
            const double independent = stress(rng) ? ap.demandStressScale : calmMultiplier;
            draw *= ap.demandCorrelation*sharedMultiplier + (1.0-ap.demandCorrelation)*independent;
            break;
        }
        default: throw std::invalid_argument("Nonlegacy sampler called for invalid distribution");
        }
        if (!std::isfinite(draw) || draw > std::numeric_limits<int>::max()-1.0)
            throw std::overflow_error("Generated demand exceeds supported integer range");
        return std::max(1, static_cast<int>(std::llround(draw)));
    }
};

inline void write_scenario_demand_parameters(std::ostream& stream, const AlgorithmParameters& ap) {
    stream << std::setprecision(17)
           << "\nDemandCV " << ap.demandCV
           << "\nDemandUniformSpan " << ap.demandUniformSpan
           << "\nDemandStressProb " << ap.demandStressProb
           << "\nDemandStressScale " << ap.demandStressScale
           << "\nDemandCorrelation " << ap.demandCorrelation
           << "\nFixedRouteCapacity " << ap.fixedRouteCapacity;
}

#endif
