#include "AlgorithmParameters.h"
#include "InstanceCVRPLIB.h"
#include "PCTSPExactSCIP.h"
#include "Params.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

double skipCost(const Params& params, const std::vector<int>& permutation)
{
    std::vector<bool> selected(params.nbClients + 1, false);
    for (int client : permutation)
        selected[client] = true;

    double value = 0.0;
    for (int client = 1; client <= params.nbClients; ++client) {
        if (!selected[client])
            value += params.cli[client].skipPenalty;
    }
    return value;
}

} // namespace

int main(int argc, char** argv)
{
    try {
        if (argc != 2)
            throw std::invalid_argument(
                "usage: test_pctsp_scip_optional_bruteforce <tiny.vrp>");

        AlgorithmParameters ap = default_algorithm_parameters();
        ap.optionalVisit = 1;
        ap.skipPenScale = 1.2;
        ap.scenarioSeed = 11;
        ap.penaltySeed = 17;
        ap.algoSeed = 23;
        ap.n_extra_senarios = 3;
        ap.nthreads = 1;

        InstanceCVRPLIB instance(argv[1], true);
        Params params(instance.x_coords, instance.y_coords, instance.dist_mtx,
                      instance.service_time, instance.demands,
                      instance.vehicleCapacity, instance.durationLimit,
                      std::numeric_limits<int>::max(),
                      instance.isDurationConstraint, false, ap);
        params.generate_scenario_demands(ap.n_extra_senarios);
        params.update_max_vehi();
        params.generate_skip_penalties();
        if (params.nbClients > 9)
            throw std::invalid_argument(
                "brute-force regression is intentionally limited to 9 clients");

        PCTSPExactSCIP solver(params, 30.0, false);
        double bruteObjective = std::numeric_limits<double>::infinity();
        std::vector<int> bruteTour;

        const unsigned subsetCount = 1U << params.nbClients;
        for (unsigned mask = 0; mask < subsetCount; ++mask) {
            std::vector<int> permutation;
            for (int client = 1; client <= params.nbClients; ++client) {
                if ((mask & (1U << (client - 1))) != 0U)
                    permutation.push_back(client);
            }

            do {
                const double objective = solver.evaluateTourCPU(permutation)
                    + skipCost(params, permutation);
                if (objective < bruteObjective) {
                    bruteObjective = objective;
                    bruteTour = permutation;
                }
            } while (std::next_permutation(
                permutation.begin(), permutation.end()));
        }

        const PCTSPResult exact = solver.solveFullStochasticMILP(nullptr, 4);
        const SCIPBendersStats& stats = solver.getLastStats();
        const double tolerance = 1e-7 * std::max(1.0, bruteObjective);
        if (!exact.optimal)
            throw std::runtime_error("SCIP did not close the tiny proof gap");
        if (std::abs(exact.objValue - bruteObjective) > tolerance)
            throw std::runtime_error("SCIP optimum differs from brute force");
        if (std::abs(stats.lowerBound - bruteObjective) > tolerance)
            throw std::runtime_error("SCIP certified bound differs from brute force");

        std::vector<int> exactPermutation;
        if (exact.tour.size() >= 2)
            exactPermutation.assign(exact.tour.begin() + 1,
                                    exact.tour.end() - 1);
        const double serializedObjective =
            solver.evaluateTourCPU(exactPermutation)
            + skipCost(params, exactPermutation);
        if (std::abs(serializedObjective - exact.objValue) > tolerance)
            throw std::runtime_error(
                "reported selected tour does not reproduce the objective");

        bool sawOpenGap = false;
        for (const SCIPBendersTracePoint& point : solver.getLastTrace()) {
            if (std::isfinite(point.proofGap) && point.proofGap > 1e-6)
                sawOpenGap = true;
            if (point.dualBound > bruteObjective + tolerance)
                throw std::runtime_error(
                    "a trace dual bound exceeds the brute-force optimum");
        }
        if (!sawOpenGap)
            throw std::runtime_error(
                "tiny regression did not exercise a partial proof gap");

        std::cout << "OPTIONAL_SCIP_BRUTE_FORCE_PASS objective="
                  << exact.objValue << " lower_bound=" << stats.lowerBound
                  << " selected=" << exactPermutation.size()
                  << " evaluated_tours=" << stats.evaluatedTours
                  << " trace_points=" << solver.getLastTrace().size() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "OPTIONAL_SCIP_BRUTE_FORCE_FAIL " << error.what() << '\n';
        return 1;
    }
}
