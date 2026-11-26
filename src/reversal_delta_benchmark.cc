// Benchmark + correctness harness for the 2-opt segment-reversal capacity
// primitive (EvaluateReversalViolationsBatch), the heavy multi-scenario step
// inside local search's get_reversed_cost.
//
// For a sweep of routes it compares:
//   * CPU backend  (sequential per-scenario walk)
//   * GPU backend  (one block per (route,a,b) candidate x scenario chunk)
// and verifies they agree against an independent, deliberately simple reference
// that reproduces get_reversed_cost's reversed-segment accumulation exactly.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "domain/eval/ScenarioCapacityEvaluator.h"

namespace {

// Same stochastic structure as the reader / insertion benchmark: pickups (odd
// ids) positive, deliveries (even ids) mirrored, scenario 0 nominal.
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

// A 2-opt sweep shape: many routes, each scanned for all reversal segments. To
// induce non-trivial capacity violations we bias routes towards consecutive
// pickups (load builds up before deliveries arrive).
RouteBatch BuildRouteBatch(int nodeNum, int routeCount, int routeLen, uint32_t seed) {
    RouteBatch batch;
    batch.routeOffsets.push_back(0);
    std::mt19937 generator(seed);
    std::uniform_int_distribution<int> pickupPick(0, (nodeNum - 1) / 2 - 1);
    for (int r = 0; r < routeCount; ++r) {
        std::vector<int> pickups;
        for (int i = 0; i < routeLen / 2; ++i) {
            pickups.push_back(1 + 2 * pickupPick(generator));
        }
        // Front-load the pickups, then the deliveries, so the prefix load peaks.
        for (int p : pickups) {
            batch.nodeIds.push_back(p);
        }
        for (int p : pickups) {
            batch.nodeIds.push_back(p + 1);
        }
        batch.routeOffsets.push_back(static_cast<int>(batch.nodeIds.size()));
    }
    return batch;
}

// Independent reference: for every route and every reversal segment [a,b], walk
// the reversed order (b,b-1,...,a) from the prefix load before a, exactly like
// get_reversed_cost. Intentionally written differently from the CPU backend's
// inner loop so it is a real cross-check, not a copy.
std::vector<std::vector<double>> ReferenceReversal(const std::vector<float>& demand, int numScenarios,
                                                   double vehicleCapacity, const RouteBatch& batch) {
    const int routeNum = batch.RouteNum();
    std::vector<std::vector<double>> out(routeNum);
    for (int r = 0; r < routeNum; ++r) {
        const int begin = batch.routeOffsets[r];
        const int len = batch.routeOffsets[r + 1] - begin;
        out[r].assign(static_cast<size_t>(len) * len, 0.0);
        for (int a = 1; a < len; ++a) {
            for (int b = a; b < len; ++b) {
                double viol = 0.0;
                for (int s = 0; s < numScenarios; ++s) {
                    double load = 0.0;
                    for (int m = 0; m < a; ++m) {  // prefix up to a-1
                        load += demand[static_cast<size_t>(batch.nodeIds[begin + m]) * numScenarios + s];
                    }
                    for (int j = b; j >= a; --j) {  // reversed visit order
                        load += demand[static_cast<size_t>(batch.nodeIds[begin + j]) * numScenarios + s];
                        if (load - vehicleCapacity > 0.0) {
                            viol += load - vehicleCapacity;
                        }
                    }
                }
                out[r][static_cast<size_t>(a) * len + b] = viol;
            }
        }
    }
    return out;
}

double MaxAbsDiffNested(const std::vector<std::vector<double>>& a, const std::vector<std::vector<double>>& b) {
    double maxDiff = 0.0;
    for (size_t r = 0; r < std::min(a.size(), b.size()); ++r) {
        const size_t n = std::min(a[r].size(), b[r].size());
        for (size_t i = 0; i < n; ++i) {
            maxDiff = std::max(maxDiff, std::fabs(a[r][i] - b[r][i]));
        }
    }
    return maxDiff;
}

double MaxAbs(const std::vector<std::vector<double>>& a) {
    double m = 0.0;
    for (const auto& row : a) {
        for (double v : row) {
            m = std::max(m, std::fabs(v));
        }
    }
    return m;
}

double TimeBatch(ScenarioCapacityEvaluator& evaluator, const RouteBatch& batch, int iterations,
                 std::vector<std::vector<double>>& lastOut) {
    evaluator.EvaluateReversalViolationsBatch(batch, lastOut);  // warm-up
    auto start = std::chrono::steady_clock::now();
    for (int it = 0; it < iterations; ++it) {
        evaluator.EvaluateReversalViolationsBatch(batch, lastOut);
    }
    auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count() / iterations;
}

}  // namespace

int main(int argc, char* argv[]) {
    int nodeNum = 2001;
    int routeLen = 20;    // nodes per route scanned by 2-opt
    int routeCount = 30;  // routes per sweep
    double vehicleCapacity = 200.0;
    int iterations = 30;
    std::vector<int> scenarioList = {5, 100, 1000, 10000, 100000};

    if (argc >= 2) routeLen = std::atoi(argv[1]);
    if (argc >= 3) routeCount = std::atoi(argv[2]);
    if (argc >= 4) {
        scenarioList.clear();
        scenarioList.push_back(std::atoi(argv[3]));
    }

    std::printf("2-opt reversal-violation evaluator benchmark (device-resident, scalar-only output)\n");
    std::printf("routeLen=%d  routesPerSweep=%d  vehicleCapacity=%.1f  iterations=%d\n", routeLen, routeCount,
                vehicleCapacity, iterations);
    std::printf("reversal candidates/sweep ~= routes * (L-1)L/2 = %d\n\n",
                routeCount * (routeLen - 1) * routeLen / 2);
    std::printf("%-10s %-14s %-14s %-12s %-16s %-16s\n", "scenarios", "cpu_ms", "gpu_ms", "speedup", "cpu_vs_ref",
                "gpu_vs_ref");

    RouteBatch batch = BuildRouteBatch(nodeNum, routeCount, routeLen, 11);

    for (int numScenarios : scenarioList) {
        std::vector<float> demand = BuildDemandMatrix(nodeNum, numScenarios, vehicleCapacity, 7);

        auto cpu = MakeScenarioCapacityEvaluator(demand.data(), nodeNum, numScenarios, vehicleCapacity,
                                                 EvaluatorBackend::kCpu);
        auto gpu = MakeScenarioCapacityEvaluator(demand.data(), nodeNum, numScenarios, vehicleCapacity,
                                                 EvaluatorBackend::kCuda);

        // Reference only at modest scenario counts (it is O(L^3 * S) and slow).
        std::vector<std::vector<double>> reference;
        const bool haveRef = numScenarios <= 1000;
        if (haveRef) {
            reference = ReferenceReversal(demand, numScenarios, vehicleCapacity, batch);
        }

        std::vector<std::vector<double>> cpuOut, gpuOut;
        double cpuMs = TimeBatch(*cpu, batch, iterations, cpuOut);

        char refBuf[32];
        if (haveRef) {
            std::snprintf(refBuf, sizeof(refBuf), "%.6g", MaxAbsDiffNested(cpuOut, reference));
        } else {
            std::snprintf(refBuf, sizeof(refBuf), "skip");
        }

        if (std::string(gpu->Backend()) == "cuda") {
            double gpuMs = TimeBatch(*gpu, batch, iterations, gpuOut);
            // CPU is the trusted double-precision oracle once verified against the reference.
            double gpuVsCpu = MaxAbsDiffNested(cpuOut, gpuOut);
            double scale = std::max(1.0, MaxAbs(cpuOut));
            char gpuBuf[40];
            std::snprintf(gpuBuf, sizeof(gpuBuf), "%.4g (rel %.1e)", gpuVsCpu, gpuVsCpu / scale);
            std::printf("%-10d %-14.4f %-14.4f %-12.2f %-16s %-16s\n", numScenarios, cpuMs, gpuMs, cpuMs / gpuMs,
                        refBuf, gpuBuf);
        } else {
            std::printf("%-10d %-14.4f %-14s %-12s %-16s %-16s\n", numScenarios, cpuMs, "n/a (cpu)", "-", refBuf, "-");
        }
    }
    return 0;
}
