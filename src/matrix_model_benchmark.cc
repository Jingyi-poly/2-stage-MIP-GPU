#include <dirent.h>
#include <omp.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include "config.h"
#include "domain/eval/MatrixModelEvaluator.h"
#include "domain/eval/ScenarioCapacityEvaluator.h"
#include "io/pdptw_reader.h"
#include "io/sol_reader.h"
#include "model/pdptw.h"
#include "model/solution.h"
#include "util/convert_utils.h"
#include "util/string_utils.h"

namespace {

std::string FindBks(const std::string& instanceName) {
    std::string lowerName = ToLower(instanceName);
    std::string bksFolder = AMDAHL_DIR;
    bksFolder += "/data/bks/";
    DIR* dir = opendir(bksFolder.c_str());
    if (dir == nullptr) return "";
    std::string bksFilePath;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string fileName = entry->d_name;
        if (fileName.size() < lowerName.size()) continue;
        if (0 != fileName.compare(0, lowerName.size(), lowerName, 0, lowerName.size())) continue;
        if (fileName.size() > lowerName.size()) {
            char c = fileName[lowerName.size()];
            if ('0' <= c && '9' >= c) continue;
        }
        bksFilePath = bksFolder + fileName;
    }
    closedir(dir);
    return bksFilePath;
}

// One full-solution cost evaluation via the solver's incremental path: re-run
// updateRouteData over every route, then aggregate. Returns the objective sink.
double IncrementalFullEval(VrpSolution& sol) {
    for (VrpRoute& route : sol.routes) {
        if (route.nodeNum <= 0) continue;
        sol.updateRouteData(route.routeId, route.startNode, route.endNode);
    }
    sol.calTotalValue();
    return sol.getTotalDistance() + sol.getTotalCapacityViolation() + sol.getTotalTwViolation();
}

double MatrixFullEvalCpu(const VrpSolution& sol) {
    MatrixModelEvaluator eval(sol);
    MatrixModelResult r = eval.Evaluate();
    return r.distance + r.capacityViolation + r.twViolationCapped;
}

double MatrixFullEvalGpu(const VrpSolution& sol, ScenarioCapacityEvaluator& cap) {
    MatrixModelEvaluator eval(sol);
    MatrixModelResult r = eval.Evaluate(cap);
    return r.distance + r.capacityViolation + r.twViolationCapped;
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string benchmarkName = "LiLim";
    std::string instanceName = "LC1_2_1";
    std::vector<int> scenarioList = {1000, 10000, 100000};
    if (argc >= 2) instanceName = argv[1];
    if (argc >= 3) {
        scenarioList.clear();
        for (int i = 2; i < argc; ++i) scenarioList.push_back(std::atoi(argv[i]));
    }

    const int kThreadCounts[] = {1, 8};
    const int kMaxCopies = 8;

    enum Method { kBaseline = 0, kMatrixCpu = 1, kMatrixGpu = 2 };

    std::printf("FULL-SOLUTION evaluation throughput: CPU baseline vs matrix model (CPU & GPU)\n");
    std::printf("instance=%s  copies(per-thread)=%d\n\n", instanceName.c_str(), kMaxCopies);
    std::printf("%-9s %-7s %-13s %-13s %-13s %-11s %-11s\n", "scenarios", "threads", "baseline_us", "cpuMat_us",
                "gpuMat_us", "gpu/base", "gpu/cpuMat");

    for (int numScenarios : scenarioList) {
        std::string instanceFilePath = AMDAHL_DIR;
        instanceFilePath += "/data/instance/" + instanceName + ".txt";
        PDPTW problem;
        ReadPDPTW(instanceFilePath, problem, ProblemFormat::kLiLim, numScenarios);
        std::string bksFilePath = FindBks(instanceName);
        Solution solution(problem);
        ReadSol(bksFilePath, solution);

        VrpProblem vrpProblem;
        ConvertVrpProblem(problem, vrpProblem);  // useGpuCapacityEval defaults false -> CPU baseline

        VrpSolution base(&vrpProblem);
        ConvertVrpSolution(solution, base);

        // One CUDA evaluator, shared across threads (per-thread streams/buffers
        // serve concurrent calls). Falls back to CPU if no device is present.
        auto gpuEval = MakeScenarioCapacityEvaluator(vrpProblem.vrpData.demandMatrix.data(),
                                                     vrpProblem.vrpData.nodeNum, vrpProblem.vrpData.numScenarios,
                                                     vrpProblem.vrpData.vehicleCapacity, EvaluatorBackend::kCuda);
        const bool gpuOk = std::string(gpuEval->Backend()) == "cuda";

        // One independent solution copy per worker thread (no false sharing).
        std::vector<VrpSolution> copies;
        copies.reserve(kMaxCopies);
        for (int i = 0; i < kMaxCopies; ++i) {
            copies.emplace_back(&vrpProblem);
            copies.back().copySolution(base);
        }

        // One-off correctness check: GPU matrix capacity vs CPU matrix capacity.
        if (numScenarios == scenarioList.front()) {
            MatrixModelEvaluator mm(base);
            const double capCpu = mm.Evaluate().capacityViolation;
            const double capGpu = gpuOk ? mm.Evaluate(*gpuEval).capacityViolation : capCpu;
            std::printf("# correctness @%d: cap_cpu=%.3f cap_gpu=%.3f |rel|=%.2e\n\n", numScenarios, capCpu, capGpu,
                        capCpu != 0.0 ? std::fabs(capCpu - capGpu) / std::fabs(capCpu) : 0.0);
        }

        const int iterations = std::min(2000, std::max(8, 40000000 / numScenarios));

        for (int threads : kThreadCounts) {
            double sink = 0.0;

            auto runBatch = [&](Method method) {
                double localSink = 0.0;
#pragma omp parallel for num_threads(threads) reduction(+ : localSink) schedule(static)
                for (int it = 0; it < iterations; ++it) {
                    VrpSolution& s = copies[omp_get_thread_num()];
                    if (method == kBaseline) {
                        localSink += IncrementalFullEval(s);
                    } else if (method == kMatrixCpu) {
                        localSink += MatrixFullEvalCpu(s);
                    } else {
                        localSink += MatrixFullEvalGpu(s, *gpuEval);
                    }
                }
                return localSink;
            };

            auto timeMethod = [&](Method method) {
                sink += runBatch(method);  // warm-up
                auto a = std::chrono::steady_clock::now();
                sink += runBatch(method);
                auto b = std::chrono::steady_clock::now();
                return std::chrono::duration<double, std::micro>(b - a).count() / iterations;
            };

            const double baseUs = timeMethod(kBaseline);
            const double cpuMatUs = timeMethod(kMatrixCpu);
            const double gpuMatUs = gpuOk ? timeMethod(kMatrixGpu) : 0.0;

            if (gpuOk) {
                std::printf("%-9d %-7d %-13.3f %-13.3f %-13.3f %-11.2f %-11.2f%s\n", numScenarios, threads, baseUs,
                            cpuMatUs, gpuMatUs, baseUs / gpuMatUs, cpuMatUs / gpuMatUs, sink > 0 ? "" : "");
            } else {
                std::printf("%-9d %-7d %-13.3f %-13.3f %-13s %-11s %-11s\n", numScenarios, threads, baseUs, cpuMatUs,
                            "n/a", "-", "-");
            }
        }
        std::printf("\n");
    }
    return 0;
}
