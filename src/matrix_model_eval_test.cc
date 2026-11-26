#include <dirent.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

#include "config.h"
#include "domain/eval/MatrixModelEvaluator.h"
#include "domain/eval/ScenarioCapacityEvaluator.h"
#include "io/pdptw_reader.h"
#include "io/sol_reader.h"
#include "model/pdptw.h"
#include "model/solution.h"
#include "util/convert_utils.h"
#include "util/string_utils.h"

// Validates the NPU-style MatrixModelEvaluator against the solver's incremental
// evaluation (updateRouteData + calTotalValue) on a real BKS solution.
//
// Usage: matrix_model_eval_test [benchmark] [instanceName] [scenarioNum]
//   defaults: LiLim LC1_2_1 100
int main(int argc, char* argv[]) {
    std::string benchmarkName = argc >= 2 ? argv[1] : "LiLim";
    std::string instanceName = argc >= 3 ? argv[2] : "LC1_2_1";
    int scenarioNum = argc >= 4 ? std::atoi(argv[3]) : 100;

    std::string instanceFilePath = AMDAHL_DIR;
    instanceFilePath += "/data/instance/" + instanceName + ".txt";

    PDPTW problem;
    if (benchmarkName == "LiLim") {
        ReadPDPTW(instanceFilePath, problem, ProblemFormat::kLiLim, scenarioNum);
    } else if (benchmarkName == "SartoriBuriol") {
        ReadPDPTW(instanceFilePath, problem, ProblemFormat::kSartoriBuriol, scenarioNum);
    } else {
        std::cerr << "unknown benchmark: " << benchmarkName << std::endl;
        return 1;
    }

    // Locate the BKS file the same way main.cc does.
    std::string lowerName = ToLower(instanceName);
    std::string bksFolder = AMDAHL_DIR;
    bksFolder += "/data/bks/";
    DIR* dir = opendir(bksFolder.c_str());
    if (dir == nullptr) {
        std::cerr << "Unable to open bks directory: " << bksFolder << std::endl;
        return 1;
    }
    std::string bksFilePath;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string fileName = entry->d_name;
        if (fileName.size() < lowerName.size()) continue;
        if (0 != fileName.compare(0, lowerName.size(), lowerName, 0, lowerName.size())) continue;
        if (fileName.size() > lowerName.size()) {
            char c = fileName[lowerName.size()];
            if ('0' <= c && '9' >= c) continue;  // avoid matching LC1_2_10 for LC1_2_1
        }
        bksFilePath = bksFolder + fileName;
    }
    closedir(dir);
    if (bksFilePath.empty()) {
        std::cerr << "No BKS file found for " << instanceName << std::endl;
        return 1;
    }

    Solution solution(problem);
    ReadSol(bksFilePath, solution);

    VrpProblem vrpProblem;
    ConvertVrpProblem(problem, vrpProblem);

    // Optional GPU backend for the matrix model's capacity step (uses float demand).
    auto gpuEval = MakeScenarioCapacityEvaluator(vrpProblem.vrpData.demandMatrix.data(), vrpProblem.vrpData.nodeNum,
                                                 vrpProblem.vrpData.numScenarios, vrpProblem.vrpData.vehicleCapacity,
                                                 EvaluatorBackend::kCuda);
    const bool gpuOk = std::string(gpuEval->Backend()) == "cuda";

    std::cout.setf(std::ios::fixed);
    std::cout.precision(6);
    std::cout << "Instance        : " << instanceName << "  (scenarios=" << scenarioNum << ")\n\n";

    auto line = [](const char* label, double solver, double matrix) {
        const double diff = std::fabs(solver - matrix);
        const double tol = 1e-4 + 1e-6 * std::fabs(solver);
        const bool good = diff <= tol;
        std::cout << "  " << std::left;
        std::cout.width(22);
        std::cout << label << std::right << " solver=" << solver << "  matrix=" << matrix << "  |diff|=" << diff
                  << (good ? "  [OK]" : "  [MISMATCH]") << "\n";
        return good;
    };

    // Compare the matrix model against the solver's incremental totals for one
    // solution, and report the slide-8 (uncapped) TW variant for context.
    auto compare = [&](const char* title, VrpSolution& sol) {
        MatrixModelEvaluator matrixEval(sol);
        auto t0 = std::chrono::steady_clock::now();
        MatrixModelResult mm = matrixEval.Evaluate();
        auto t1 = std::chrono::steady_clock::now();
        const double matrixMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

        std::cout << title << "  (M=" << mm.routeCount << " routes x N<=" << mm.maxRouteLen << ", matrix eval "
                  << matrixMs << " ms)\n";
        bool ok = true;
        ok &= line("distance", sol.getTotalDistance(), mm.distance);
        ok &= line("capacity violation (cpu)", sol.getTotalCapacityViolation(), mm.capacityViolation);
        ok &= line("tw violation (capped)", sol.getTotalTwViolation(), mm.twViolationCapped);
        if (gpuOk) {
            // GPU path uses float demand, so compare with a relative tolerance.
            const double capGpu = matrixEval.Evaluate(*gpuEval).capacityViolation;
            const double ref = sol.getTotalCapacityViolation();
            const double rel = ref != 0.0 ? std::fabs(capGpu - ref) / std::fabs(ref) : std::fabs(capGpu);
            std::cout << "  " << std::left;
            std::cout.width(22);
            std::cout << "capacity (gpu/matrix)" << std::right << " solver=" << ref << "  matrix=" << capGpu
                      << "  |rel|=" << rel << (rel <= 1e-5 ? "  [OK]" : "  [check]") << "\n";
        }
        std::cout << "  slide-8 tw (uncapped) = " << mm.twViolationSlides
                  << "   delta vs capped = " << (mm.twViolationSlides - mm.twViolationCapped) << "\n\n";
        return ok;
    };

    bool ok = true;

    VrpSolution bks(&vrpProblem);
    ConvertVrpSolution(solution, bks);  // runs updateRouteData per route (incremental CPU path)
    ok &= compare("[1] BKS (feasible)", bks);

    // Stress case: relocate every request into route 0 so the single overloaded
    // route produces large, nonzero capacity *and* time-window violations -- this
    // is what actually exercises the matrix model's cumsum/relu reductions.
    VrpSolution stress(&vrpProblem);
    stress.copySolution(bks);
    for (int pickup : vrpProblem.vrpData.pickNodeIds) {
        const int delivery = vrpProblem.vrpData.nodeList[pickup].corresNodeId;
        const int srcRoute = stress.routeNodes[pickup].routeId;
        if (srcRoute <= 0) {
            continue;  // unassigned or already in route 0
        }
        stress.removeNode(srcRoute, pickup, delivery, true);
        RouteNode* tail = stress.routes[0].endNode;  // append the pair at the very end
        stress.insertNode(0, pickup, delivery, tail, tail, true);
    }
    ok &= compare("[2] Stressed (all requests merged into route 0)", stress);

    // Capacity stress: cluster pickups near the front and deliveries at the tail
    // so loads stack up well beyond vehicle capacity, forcing a large nonzero
    // capacity violation across every scenario.
    VrpSolution capStress(&vrpProblem);
    capStress.copySolution(bks);
    for (int pickup : vrpProblem.vrpData.pickNodeIds) {
        const int delivery = vrpProblem.vrpData.nodeList[pickup].corresNodeId;
        const int srcRoute = capStress.routeNodes[pickup].routeId;
        if (srcRoute <= 0) {
            continue;
        }
        capStress.removeNode(srcRoute, pickup, delivery, true);
        RouteNode* head = capStress.routes[0].startNode;
        RouteNode* tail = capStress.routes[0].endNode;
        if (head == nullptr || head->nexNode == nullptr) {
            capStress.insertNode(0, pickup, delivery, tail, tail, true);  // route too short to split
        } else {
            capStress.insertNode(0, pickup, delivery, head, tail, true);  // pickup at front, delivery at tail
        }
    }
    ok &= compare("[3] Capacity-stressed (pickups front / deliveries tail)", capStress);

    std::cout << "Notes: distance & capacity reproduce the solver exactly (capacity is the same\n"
                 "       multi-scenario cumsum/relu/reduce already offloaded to the GPU). The slide-8\n"
                 "       cumsum/cummax TW formula matches only with no lateness; otherwise it propagates\n"
                 "       uncapped lateness, so this solver's capped TW penalty needs a row scan.\n";
    std::cout << "\nResult: " << (ok ? "PASS" : "FAIL") << std::endl;
    return ok ? 0 : 1;
}
