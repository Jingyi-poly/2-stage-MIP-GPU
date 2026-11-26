#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "domain/eval/ScenarioCapacityEvaluator.h"

namespace {

struct SyntheticProblem {
    int nodeNum;
    int numScenarios;
    double vehicleCapacity;
    std::vector<float> demandMatrix;  // node-major [nodeNum * numScenarios]
};

// Builds a synthetic stochastic PDPTW-like instance: pickups (odd ids) carry a
// positive demand, deliveries (even ids) mirror it, scenario 0 is nominal and
// the rest are perturbations - the same structure the real reader now produces.
SyntheticProblem BuildSyntheticProblem(int nodeNum, int numScenarios, double vehicleCapacity, uint32_t seed) {
    SyntheticProblem problem;
    problem.nodeNum = nodeNum;
    problem.numScenarios = numScenarios;
    problem.vehicleCapacity = vehicleCapacity;
    problem.demandMatrix.assign(static_cast<size_t>(nodeNum) * numScenarios, 0.0f);

    std::mt19937 generator(seed);
    std::uniform_int_distribution<int> base_demand(1, static_cast<int>(vehicleCapacity / 4) + 1);
    std::uniform_real_distribution<double> factor(0.5, 1.5);

    for (int pickup = 1; pickup + 1 < nodeNum; pickup += 2) {
        int delivery = pickup + 1;
        int nominal = base_demand(generator);
        problem.demandMatrix[static_cast<size_t>(pickup) * numScenarios] = static_cast<float>(nominal);
        problem.demandMatrix[static_cast<size_t>(delivery) * numScenarios] = static_cast<float>(-nominal);
        for (int s = 1; s < numScenarios; ++s) {
            int sampled = std::max(1, static_cast<int>(std::lround(nominal * factor(generator))));
            problem.demandMatrix[static_cast<size_t>(pickup) * numScenarios + s] = static_cast<float>(sampled);
            problem.demandMatrix[static_cast<size_t>(delivery) * numScenarios + s] = static_cast<float>(-sampled);
        }
    }
    return problem;
}

// Splits the pickup/delivery requests over `routeNum` routes; within a route a
// request's pickup precedes its delivery, mimicking a feasible route shape.
RouteBatch BuildRouteBatch(int nodeNum, int routeNum, uint32_t seed) {
    std::vector<int> requestPickups;
    for (int pickup = 1; pickup + 1 < nodeNum; pickup += 2) {
        requestPickups.push_back(pickup);
    }
    std::mt19937 generator(seed);
    std::shuffle(requestPickups.begin(), requestPickups.end(), generator);

    RouteBatch batch;
    batch.routeOffsets.push_back(0);
    const int requestsPerRoute = std::max<int>(1, static_cast<int>(requestPickups.size()) / std::max(1, routeNum));
    int cursor = 0;
    for (int route = 0; route < routeNum && cursor < static_cast<int>(requestPickups.size()); ++route) {
        int count = (route == routeNum - 1) ? static_cast<int>(requestPickups.size()) - cursor : requestsPerRoute;
        count = std::min(count, static_cast<int>(requestPickups.size()) - cursor);
        for (int i = 0; i < count; ++i) {
            int pickup = requestPickups[cursor + i];
            batch.nodeIds.push_back(pickup);
            batch.nodeIds.push_back(pickup + 1);
        }
        cursor += count;
        batch.routeOffsets.push_back(static_cast<int>(batch.nodeIds.size()));
    }
    return batch;
}

double MaxAbsDiff(const std::vector<double>& a, const std::vector<double>& b) {
    double maxDiff = 0.0;
    const size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i) {
        maxDiff = std::max(maxDiff, std::fabs(a[i] - b[i]));
    }
    if (a.size() != b.size()) {
        maxDiff = std::numeric_limits<double>::infinity();
    }
    return maxDiff;
}

double TimeFullEvaluate(ScenarioCapacityEvaluator& evaluator, const RouteBatch& batch, int iterations,
                        std::vector<double>& outLoads, std::vector<double>& outNodeAccViol) {
    evaluator.Evaluate(batch, outLoads, outNodeAccViol);  // warm-up, excluded
    auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < iterations; ++i) {
        evaluator.Evaluate(batch, outLoads, outNodeAccViol);
    }
    auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count() / iterations;
}

double TimeRouteViolations(ScenarioCapacityEvaluator& evaluator, const RouteBatch& batch, int iterations,
                           std::vector<double>& outRouteViol) {
    evaluator.EvaluateRouteViolations(batch, outRouteViol);  // warm-up, excluded
    auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < iterations; ++i) {
        evaluator.EvaluateRouteViolations(batch, outRouteViol);
    }
    auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count() / iterations;
}

}  // namespace

int main(int argc, char* argv[]) {
    int nodeNum = 2001;  // 1 depot + 1000 pickup/delivery pairs
    // Batch the routes of a whole population (~100 solutions) at once, which is
    // the intended GPU usage; with 1000 requests this gives ~1000 routes.
    int routeNum = 1000;
    double vehicleCapacity = 200.0;
    int iterations = 20;
    std::vector<int> scenarioList = {5, 100, 1000, 10000, 100000};
    bool gpuFirst = false;

    if (argc >= 2) nodeNum = std::atoi(argv[1]);
    if (argc >= 3) routeNum = std::atoi(argv[2]);
    if (argc >= 4) {
        scenarioList.clear();
        scenarioList.push_back(std::atoi(argv[3]));
    }
    if (argc >= 5) {
        const std::string order(argv[4]);
        if (order == "gpu-first") {
            gpuFirst = true;
        } else if (order != "cpu-first") {
            std::fprintf(stderr, "execution order must be cpu-first or gpu-first\n");
            return 2;
        }
    }

    std::printf("Scenario capacity evaluator benchmark\n");
    std::printf("nodeNum=%d  routeNum=%d  vehicleCapacity=%.1f  iterations=%d  executionOrder=%s\n\n", nodeNum,
                routeNum, vehicleCapacity, iterations, gpuFirst ? "GPU-first" : "CPU-first");

    std::printf("== Per-route total violation (lean path, fitness use-case) ==\n");
    std::printf("%-10s %-14s %-14s %-12s %-16s\n", "scenarios", "cpu_ms", "gpu_ms", "speedup", "max_abs_diff");

    bool gpuIsCuda = false;
    for (int numScenarios : scenarioList) {
        SyntheticProblem problem = BuildSyntheticProblem(nodeNum, numScenarios, vehicleCapacity, 7);
        RouteBatch batch = BuildRouteBatch(nodeNum, routeNum, 11);

        auto cpu = MakeScenarioCapacityEvaluator(problem.demandMatrix.data(), problem.nodeNum, problem.numScenarios,
                                                 problem.vehicleCapacity, EvaluatorBackend::kCpu);
        auto gpu = MakeScenarioCapacityEvaluator(problem.demandMatrix.data(), problem.nodeNum, problem.numScenarios,
                                                 problem.vehicleCapacity, EvaluatorBackend::kCuda);
        gpuIsCuda = std::string(gpu->Backend()) == "cuda";

        std::vector<double> cpuViol, gpuViol;
        double cpuMs = 0.0;
        double gpuMs = 0.0;
        if (gpuIsCuda) {
            if (gpuFirst) {
                gpuMs = TimeRouteViolations(*gpu, batch, iterations, gpuViol);
                cpuMs = TimeRouteViolations(*cpu, batch, iterations, cpuViol);
            } else {
                cpuMs = TimeRouteViolations(*cpu, batch, iterations, cpuViol);
                gpuMs = TimeRouteViolations(*gpu, batch, iterations, gpuViol);
            }
            double diff = MaxAbsDiff(cpuViol, gpuViol);
            std::printf("%-10d %-14.4f %-14.4f %-12.2f %-16.6g\n", numScenarios, cpuMs, gpuMs, cpuMs / gpuMs, diff);
        } else {
            cpuMs = TimeRouteViolations(*cpu, batch, iterations, cpuViol);
            std::printf("%-10d %-14.4f %-14s %-12s %-16s\n", numScenarios, cpuMs, "n/a (cpu)", "-", "-");
        }
    }

    std::printf("\n== Full per-node loads + accumulated violation (host-integration path) ==\n");
    std::printf("%-10s %-14s %-14s %-12s %-16s\n", "scenarios", "cpu_ms", "gpu_ms", "speedup", "max_abs_diff");
    for (int numScenarios : {5, 1000, 10000}) {
        SyntheticProblem problem = BuildSyntheticProblem(nodeNum, numScenarios, vehicleCapacity, 7);
        RouteBatch batch = BuildRouteBatch(nodeNum, routeNum, 11);

        auto cpu = MakeScenarioCapacityEvaluator(problem.demandMatrix.data(), problem.nodeNum, problem.numScenarios,
                                                 problem.vehicleCapacity, EvaluatorBackend::kCpu);
        auto gpu = MakeScenarioCapacityEvaluator(problem.demandMatrix.data(), problem.nodeNum, problem.numScenarios,
                                                 problem.vehicleCapacity, EvaluatorBackend::kCuda);

        std::vector<double> cpuLoads, cpuViol, gpuLoads, gpuViol;
        double cpuMs = 0.0;
        double gpuMs = 0.0;
        if (std::string(gpu->Backend()) == "cuda") {
            if (gpuFirst) {
                gpuMs = TimeFullEvaluate(*gpu, batch, iterations, gpuLoads, gpuViol);
                cpuMs = TimeFullEvaluate(*cpu, batch, iterations, cpuLoads, cpuViol);
            } else {
                cpuMs = TimeFullEvaluate(*cpu, batch, iterations, cpuLoads, cpuViol);
                gpuMs = TimeFullEvaluate(*gpu, batch, iterations, gpuLoads, gpuViol);
            }
            double diff = std::max(MaxAbsDiff(cpuLoads, gpuLoads), MaxAbsDiff(cpuViol, gpuViol));
            std::printf("%-10d %-14.4f %-14.4f %-12.2f %-16.6g\n", numScenarios, cpuMs, gpuMs, cpuMs / gpuMs, diff);
        } else {
            cpuMs = TimeFullEvaluate(*cpu, batch, iterations, cpuLoads, cpuViol);
            std::printf("%-10d %-14.4f %-14s %-12s %-16s\n", numScenarios, cpuMs, "n/a (cpu)", "-", "-");
        }
    }
    return 0;
}
