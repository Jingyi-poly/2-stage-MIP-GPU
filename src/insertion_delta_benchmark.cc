#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "domain/eval/ScenarioCapacityEvaluator.h"

namespace {

// Same stochastic structure as the reader: pickups (odd ids) positive, deliveries
// (even ids) mirrored, scenario 0 nominal, the rest perturbed.
std::vector<float> BuildDemandMatrix(int nodeNum, int numScenarios, double vehicleCapacity, uint32_t seed) {
    std::vector<float> demand(static_cast<size_t>(nodeNum) * numScenarios, 0.0f);
    std::mt19937 generator(seed);
    std::uniform_int_distribution<int> baseDemand(1, static_cast<int>(vehicleCapacity / 4) + 1);
    std::uniform_real_distribution<double> factor(0.5, 1.5);
    for (int pickup = 1; pickup + 1 < nodeNum; pickup += 2) {
        int delivery = pickup + 1;
        int nominal = baseDemand(generator);
        demand[static_cast<size_t>(pickup) * numScenarios] = static_cast<float>(nominal);
        demand[static_cast<size_t>(delivery) * numScenarios] = static_cast<float>(-nominal);
        for (int s = 1; s < numScenarios; ++s) {
            int sampled = std::max(1, static_cast<int>(std::lround(nominal * factor(generator))));
            demand[static_cast<size_t>(pickup) * numScenarios + s] = static_cast<float>(sampled);
            demand[static_cast<size_t>(delivery) * numScenarios + s] = static_cast<float>(-sampled);
        }
    }
    return demand;
}

// A set of (route, request-to-insert) probes, mimicking a greedy-insert sweep that
// tries one request against many routes.
struct InsertionProbes {
    std::vector<std::vector<int>> routes;
    std::vector<int> pickupIds;
    std::vector<int> deliveryIds;
};

InsertionProbes BuildProbes(int nodeNum, int probeCount, int routeLen, uint32_t seed) {
    InsertionProbes probes;
    std::mt19937 generator(seed);
    std::uniform_int_distribution<int> pickupPick(0, (nodeNum - 1) / 2 - 1);

    for (int p = 0; p < probeCount; ++p) {
        std::vector<int> route;
        route.reserve(routeLen);
        for (int i = 0; i < routeLen / 2; ++i) {
            int pickup = 1 + 2 * (pickupPick(generator));
            route.push_back(pickup);
            route.push_back(pickup + 1);
        }
        probes.routes.push_back(std::move(route));
        int reqPickup = 1 + 2 * pickupPick(generator);
        probes.pickupIds.push_back(reqPickup);
        probes.deliveryIds.push_back(reqPickup + 1);
    }
    return probes;
}

double MaxAbsFiniteDiff(const std::vector<double>& a, const std::vector<double>& b) {
    double maxDiff = 0.0;
    const size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i) {
        if (std::isinf(a[i]) && std::isinf(b[i])) {
            continue;
        }
        maxDiff = std::max(maxDiff, std::fabs(a[i] - b[i]));
    }
    return maxDiff;
}

InsertionProbeBatch ToBatch(const InsertionProbes& probes) {
    InsertionProbeBatch batch;
    batch.routeOffsets.push_back(0);
    for (size_t i = 0; i < probes.routes.size(); ++i) {
        batch.nodeIds.insert(batch.nodeIds.end(), probes.routes[i].begin(), probes.routes[i].end());
        batch.routeOffsets.push_back(static_cast<int>(batch.nodeIds.size()));
        batch.pickupIds.push_back(probes.pickupIds[i]);
        batch.deliveryIds.push_back(probes.deliveryIds[i]);
    }
    return batch;
}

double TimeBatch(ScenarioCapacityEvaluator& evaluator, const InsertionProbeBatch& batch, int iterations,
                 std::vector<std::vector<double>>& lastOut) {
    evaluator.EvaluateInsertionDeltasBatch(batch, lastOut);  // warm-up
    auto start = std::chrono::steady_clock::now();
    for (int it = 0; it < iterations; ++it) {
        evaluator.EvaluateInsertionDeltasBatch(batch, lastOut);
    }
    auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count() / iterations;
}

double MaxAbsFiniteDiffNested(const std::vector<std::vector<double>>& a, const std::vector<std::vector<double>>& b) {
    double maxDiff = 0.0;
    for (size_t p = 0; p < std::min(a.size(), b.size()); ++p) {
        maxDiff = std::max(maxDiff, MaxAbsFiniteDiff(a[p], b[p]));
    }
    return maxDiff;
}

}  // namespace

int main(int argc, char* argv[]) {
    int nodeNum = 2001;
    int routeLen = 40;        // a typical PDPTW route length
    int probeCount = 50;      // requests tried against ~50 routes per sweep
    double vehicleCapacity = 200.0;
    int iterations = 50;
    std::vector<int> scenarioList = {5, 100, 1000, 10000, 100000};

    if (argc >= 2) routeLen = std::atoi(argv[1]);
    if (argc >= 3) probeCount = std::atoi(argv[2]);
    if (argc >= 4) {
        scenarioList.clear();
        scenarioList.push_back(std::atoi(argv[3]));
    }

    std::printf("Insertion-delta evaluator benchmark (device-resident, scalar-only output)\n");
    std::printf("routeLen=%d  probesPerSweep=%d  vehicleCapacity=%.1f  iterations=%d\n\n", routeLen, probeCount,
                vehicleCapacity, iterations);
    std::printf("%-10s %-14s %-14s %-12s %-16s\n", "scenarios", "cpu_ms", "gpu_ms", "speedup", "max_abs_diff");

    InsertionProbes probes = BuildProbes(nodeNum, probeCount, routeLen, 11);
    InsertionProbeBatch batch = ToBatch(probes);
    for (int numScenarios : scenarioList) {
        std::vector<float> demand = BuildDemandMatrix(nodeNum, numScenarios, vehicleCapacity, 7);

        auto cpu = MakeScenarioCapacityEvaluator(demand.data(), nodeNum, numScenarios, vehicleCapacity,
                                                 EvaluatorBackend::kCpu);
        auto gpu = MakeScenarioCapacityEvaluator(demand.data(), nodeNum, numScenarios, vehicleCapacity,
                                                 EvaluatorBackend::kCuda);

        std::vector<std::vector<double>> cpuOut, gpuOut;
        double cpuMs = TimeBatch(*cpu, batch, iterations, cpuOut);
        if (std::string(gpu->Backend()) == "cuda") {
            double gpuMs = TimeBatch(*gpu, batch, iterations, gpuOut);
            double diff = MaxAbsFiniteDiffNested(cpuOut, gpuOut);
            std::printf("%-10d %-14.4f %-14.4f %-12.2f %-16.6g\n", numScenarios, cpuMs, gpuMs, cpuMs / gpuMs, diff);
        } else {
            std::printf("%-10d %-14.4f %-14s %-12s %-16s\n", numScenarios, cpuMs, "n/a (cpu)", "-", "-");
        }
    }
    return 0;
}
