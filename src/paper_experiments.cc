// Reproduces, mapped onto our PDPTW + GPU scenario-evaluation framework, the
// three experiments of "GPU-based Split algorithm for Large-Scale CVRPSD"
// (arXiv:2511.18022):
//
//   4.1  Scalability of scenario evaluation vs #scenarios (CPU-1 / CPU-8 / GPU).
//        -> covered by `matrix_model_benchmark` (full-solution eval throughput).
//   4.2  Impact of the training scenario-set size on out-of-sample decision
//        quality. We optimise with N in {1,100,1000,10000} scenarios, then score
//        the obtained solution on a fixed, independently-seeded out-of-sample set.
//   4.3  Decision quality under fixed wall-clock budgets for CPU-1 / CPU-8 / GPU.
//
// In this PDPTW model only demand is stochastic (time windows are deterministic),
// so the out-of-sample objective is
//     distance + twPenalty * twViolation + capPenalty * E_oos[capacity violation],
// where E_oos[.] is the mean per-scenario total capacity violation evaluated on
// the held-out scenario set via the ScenarioCapacityEvaluator.

#include <dirent.h>
#include <sys/stat.h>

#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "config.h"
#include "domain/eval/ScenarioCapacityEvaluator.h"
#include "domain/solver/memetic/Memetic.h"
#include "io/pdptw_reader.h"
#include "io/sol_reader.h"
#include "model/pdptw.h"
#include "model/solution.h"
#include "util/convert_utils.h"
#include "util/random_utils.h"
#include "util/string_utils.h"

namespace {

constexpr double kCapPenalty = 20000.0;
constexpr double kTwPenalty = 20000.0;
constexpr uint32_t kDefaultTrainSeed = 42;
constexpr uint32_t kDefaultOutOfSampleSeed = 7777777;

uint32_t g_trainSeed = kDefaultTrainSeed;
uint32_t g_outOfSampleSeed = kDefaultOutOfSampleSeed;
uint32_t g_algoSeed = kRandomSeed;

// Natural vehicle capacity by default: we run OUR PDPTW method exactly as-is and
// observe whether it exhibits the paper's properties, without altering the
// instance. (Kept as an optional override only.)
double g_capacityScale = 1.0;

// When true, the solver does NOT seed the population with BKS; it searches from
// scratch so there is headroom for extra generations (i.e. faster evaluation) to
// translate into better solutions.
bool g_fromScratch = false;

uint32_t SeedFromEnv(const char* name, uint32_t fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr) return fallback;

    errno = 0;
    char* end = nullptr;
    const unsigned long long parsed = std::strtoull(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0' || parsed > std::numeric_limits<uint32_t>::max()) {
        std::fprintf(stderr, "WARNING invalid %s=%s; using %u\n", name, value, fallback);
        return fallback;
    }
    return static_cast<uint32_t>(parsed);
}

int PositiveIntFromEnv(const char* name, int fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr) return fallback;

    errno = 0;
    char* end = nullptr;
    const long long parsed = std::strtoll(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0' || parsed <= 0 || parsed > std::numeric_limits<int>::max()) {
        std::fprintf(stderr, "WARNING invalid %s=%s; using %d\n", name, value, fallback);
        return fallback;
    }
    return static_cast<int>(parsed);
}

// Extra vehicles beyond the (deterministic) BKS fleet. Under tight stochastic
// capacity the robust response is to deploy more vehicles so per-route concurrent
// load drops, restoring capacity feasibility WITHOUT breaking time windows.
int ExtraRoutes() {
    if (const char* e = std::getenv("PAPER_EXTRA_ROUTES")) return std::atoi(e);
    return 0;
}

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

std::string InstancePath(const std::string& instanceName) {
    std::string p = AMDAHL_DIR;
    p += "/data/instance/" + instanceName + ".txt";
    return p;
}

// Concatenate every route's node-id chain into a CSR RouteBatch.
RouteBatch ToBatch(VrpSolution& sol) {
    RouteBatch batch;
    batch.routeOffsets.push_back(0);
    for (VrpRoute& route : sol.routes) {
        if (route.nodeNum <= 0) continue;
        for (RouteNode* n = route.startNode; n != nullptr; n = n->nexNode) {
            batch.nodeIds.push_back(n->nodeId);
        }
        batch.routeOffsets.push_back(static_cast<int>(batch.nodeIds.size()));
    }
    return batch;
}

struct OosScore {
    double distance = 0.0;
    double twViolation = 0.0;
    double expCapViolation = 0.0;  // mean per-scenario total capacity violation
    double penalized = 0.0;
};

// Scores a (training) solution on the held-out scenario set. Distance and TW are
// deterministic; only the capacity term is re-evaluated against oosEvaluator,
// which is bound to the out-of-sample demand matrix.
OosScore ScoreOutOfSample(VrpSolution& sol, ScenarioCapacityEvaluator& oosEvaluator, int oosScenarios) {
    RouteBatch batch = ToBatch(sol);
    std::vector<double> routeViol;
    oosEvaluator.EvaluateRouteViolations(batch, routeViol);
    double total = 0.0;
    for (double v : routeViol) total += v;

    OosScore s;
    s.distance = sol.getTotalDistance();
    s.twViolation = sol.getTotalTwViolation();
    s.expCapViolation = oosScenarios > 0 ? total / oosScenarios : 0.0;
    s.penalized = s.distance + kTwPenalty * s.twViolation + kCapPenalty * s.expCapViolation;
    return s;
}

long LastGeneration(const std::string& log) {
    long gen = 0;
    std::istringstream iss(log);
    std::string line;
    while (std::getline(iss, line)) {
        if (!line.empty() && (std::isdigit(line[0]) != 0)) {
            gen = std::strtol(line.c_str(), nullptr, 10);
        }
    }
    return gen;
}

// Builds a VrpProblem with `scenarios` realisations sampled directly with `seed`.
// The reader's scenario-capacity arc screen is a robust feasibility filter: it
// removes an arc when any sampled scenario overloads the vehicle.  These paper
// experiments instead model overload as a soft expected penalty, so applying
// that filter would change the feasible first-stage space as N grows.  Keep it
// disabled for both training and held-out evaluation; deterministic precedence
// and time-window arc screens are still constructed by the reader.
void BuildProblem(const std::string& instanceName, int scenarios, uint32_t seed, PDPTW& problem,
                  VrpProblem& vrpProblem, bool screenScenarioCapacityArcs = false) {
    ReadPDPTW(InstancePath(instanceName), problem, ProblemFormat::kLiLim, scenarios, seed,
              screenScenarioCapacityArcs);
    ConvertVrpProblem(problem, vrpProblem);
    vrpProblem.vrpData.vehicleCapacity *= g_capacityScale;
}

// Runs the memetic solver to `timeLimit` and returns the best solution found,
// with stdout suppressed (the solver is very chatty). When useGpu is set, the
// per-scenario capacity work is delegated to a CUDA evaluator.
void RunSolve(VrpProblem& vrpProblem, const VrpSolution& seed, int routeNum, double timeLimit, int threads,
               bool useGpu, std::unique_ptr<ScenarioCapacityEvaluator>& evalHolder, VrpSolution& outBest,
               long& generationsOut, double& solveSecondsOut, const char*& backendOut,
               std::string* capturedOut = nullptr) {
    vrpProblem.useGpuCapacityEval = false;
    vrpProblem.useGpuInsertionEval = false;
    vrpProblem.capacityEvaluator = nullptr;
    backendOut = "cpu";
    if (useGpu) {
        evalHolder = MakeScenarioCapacityEvaluator(vrpProblem.vrpData.demandMatrix.data(), vrpProblem.vrpData.nodeNum,
                                                   vrpProblem.vrpData.numScenarios, vrpProblem.vrpData.vehicleCapacity,
                                                   EvaluatorBackend::kCuda);
        vrpProblem.capacityEvaluator = evalHolder.get();
        vrpProblem.useGpuCapacityEval = true;
        vrpProblem.useGpuInsertionEval = true;
        backendOut = evalHolder->Backend();
    }

    VrpConfig vrpConfig;
    vrpConfig.vehicleCost = 0.0;
    // With hard time windows, pin the TW penalty far above the (adaptive) capacity
    // penalty so the search never sacrifices TW feasibility for capacity.
    vrpConfig.twPenaltyCoeff = gMemeticHardTw ? 1.0e7 : kTwPenalty;
    vrpConfig.capacityPenaltyCoeff = kCapPenalty;
    vrpConfig.useMeanScenarioCapacityViolation = true;

    std::vector<VrpSolution> initialSols;
    if (!g_fromScratch) {
        initialSols.push_back(seed);
    }

    std::ostringstream captured;
    std::streambuf* old = std::cout.rdbuf(captured.rdbuf());
    ResetRandomGenerator(g_algoSeed);
    const auto solveStart = std::chrono::steady_clock::now();
    memetic(vrpConfig, routeNum, timeLimit, seed, initialSols, vrpProblem, outBest, "/tmp/paper_exp_results", threads);
    solveSecondsOut =
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - solveStart)
            .count();
    std::cout.rdbuf(old);
    generationsOut = LastGeneration(captured.str());
    if (capturedOut != nullptr) *capturedOut = captured.str();
}

// Single-run anytime trace: run each backend ONCE to `budget` and dump the
// solver's full per-generation log (Generation/Distance/TW/Capacity/Time) so the
// monotone best-so-far convergence curve can be plotted from one trajectory.
// This is the correct way to show objective-vs-time for a stochastic, threaded
// search (independent re-runs per budget are NOT comparable, hence noisy starts).
void ExperimentTrace(const std::string& instanceName, int trainScenarios, double budget) {
    PDPTW problem;
    VrpProblem vrpProblem;
    BuildProblem(instanceName, trainScenarios, g_trainSeed, problem, vrpProblem);
    std::string bksPath = FindBks(instanceName);
    Solution bksSolution(problem);
    ReadSol(bksPath, bksSolution);
    VrpSolution seed(&vrpProblem);
    ConvertVrpSolution(bksSolution, seed);
    int routeNum = seed.vehicleNum + ExtraRoutes();

    struct Config {
        const char* name;
        int threads;
        bool useGpu;
    };
    std::vector<Config> configs = {{"CPU-8thread", 8, false}, {"GPU-8thread", 8, true}};
    if (const char* f = std::getenv("PAPER_CONFIGS")) {
        std::string filter(f);
        std::vector<Config> kept;
        for (const Config& c : configs)
            if (filter.find(c.name) != std::string::npos) kept.push_back(c);
        if (!kept.empty()) configs = kept;
    }
    for (Config& c : configs) {
        c.threads = PositiveIntFromEnv("PAPER_SOLVER_THREADS", c.threads);
    }

    for (const Config& cfg : configs) {
        std::unique_ptr<ScenarioCapacityEvaluator> evalHolder;
        VrpSolution best(&vrpProblem);
        long gens = 0;
        double solveSeconds = 0.0;
        const char* backend = "cpu";
        std::string log;
        RunSolve(vrpProblem, seed, routeNum, budget, cfg.threads, cfg.useGpu, evalHolder, best, gens, solveSeconds,
                 backend, &log);
        std::string path = std::string("/tmp/fig_trace_") + cfg.name + ".log";
        FILE* fp = std::fopen(path.c_str(), "w");
        if (fp != nullptr) {
            std::fwrite(log.data(), 1, log.size(), fp);
            std::fclose(fp);
        }
        std::printf("wrote %s  (backend=%s gens=%ld trainScen=%d budget=%.0fs)\n", path.c_str(), backend, gens,
                    trainScenarios, budget);
        const double inSampleMean = best.getTotalCapacityViolation() / static_cast<double>(trainScenarios);
        std::printf(
            "PAPER_RESULT experiment=trace3 config=%s backend=%s budget_seconds=%.17g train_scenarios=%d "
            "generations=%ld solve_wall_seconds=%.17g distance=%.17g tw_violation=%.17g "
            "in_mean_capacity_violation=%.17g algo_seed=%u\n",
            cfg.name, backend, budget, trainScenarios, gens, solveSeconds, best.getTotalDistance(),
            best.getTotalTwViolation(), inSampleMean, g_algoSeed);
        std::fflush(stdout);
    }
}

void Experiment42(const std::string& instanceName, double timeLimit, int oosScenarios) {
    std::printf("\n========== Experiment 4.2: training scenario-set size -> out-of-sample quality ==========\n");
    std::printf("instance=%s  timeLimit=%.0fs/run  out-of-sample=%d scenarios (seed %u)  backend=GPU,8threads\n",
                instanceName.c_str(), timeLimit, oosScenarios, g_outOfSampleSeed);
    std::printf("%-10s %-8s %-12s %-14s %-16s %-16s %-16s\n", "trainScen", "gens", "distance", "tw_viol",
                "in_E[capViol]", "oos_E[capViol]", "oos_penalized");

    // Held-out scenario set + evaluator, shared across all training sizes.
    PDPTW oosProblem;
    VrpProblem oosVrp;
    BuildProblem(instanceName, oosScenarios, g_outOfSampleSeed, oosProblem, oosVrp,
                 /*screenScenarioCapacityArcs=*/false);
    auto oosEval = MakeScenarioCapacityEvaluator(oosVrp.vrpData.demandMatrix.data(), oosVrp.vrpData.nodeNum,
                                                 oosVrp.vrpData.numScenarios, oosVrp.vrpData.vehicleCapacity,
                                                 EvaluatorBackend::kCuda);

    std::string bksPath = FindBks(instanceName);

    // Fast calibration: print only the BKS out-of-sample capacity violation under
    // the current (capacityScale, demand spread) and return, without solving.
    if (std::getenv("PAPER_BKS_ONLY") != nullptr) {
        PDPTW p0;
        VrpProblem v0;
        BuildProblem(instanceName, 1, g_trainSeed, p0, v0);
        Solution bs(p0);
        ReadSol(bksPath, bs);
        VrpSolution s0(&v0);
        ConvertVrpSolution(bs, s0);
        OosScore b = ScoreOutOfSample(s0, *oosEval, oosScenarios);
        std::printf("BKS calib: capScale=%.2f  vehCap=%.1f  dist=%.2f  oos_E[capViol]=%.4f  oos_penalized=%.2f\n",
                    g_capacityScale, v0.vrpData.vehicleCapacity, b.distance, b.expCapViolation, b.penalized);
        std::printf(
            "PAPER_RESULT experiment=exp2 kind=bks distance=%.17g tw_violation=%.17g "
            "oos_mean_capacity_violation=%.17g oos_penalized=%.17g oos_scenarios=%d oos_seed=%u\n",
            b.distance, b.twViolation, b.expCapViolation, b.penalized, oosScenarios, g_outOfSampleSeed);
        return;
    }

    std::vector<int> trainList = {1, 100, 1000, 10000};
    if (const char* tl = std::getenv("PAPER_TRAIN_LIST")) {
        trainList.clear();
        std::istringstream ts(tl);
        std::string tok;
        while (std::getline(ts, tok, ',')) trainList.push_back(std::atoi(tok.c_str()));
    }
    for (int trainN : trainList) {
        PDPTW problem;
        VrpProblem vrpProblem;
        BuildProblem(instanceName, trainN, g_trainSeed, problem, vrpProblem);

        Solution bksSolution(problem);
        ReadSol(bksPath, bksSolution);
        VrpSolution seed(&vrpProblem);
        ConvertVrpSolution(bksSolution, seed);
        int routeNum = seed.vehicleNum + ExtraRoutes();

        if (trainN == trainList.front()) {
            OosScore b = ScoreOutOfSample(seed, *oosEval, oosScenarios);
            std::printf("%-10s %-8s %-12.2f %-14.2f %-16s %-16.4f %-16.2f\n", "BKS(seed)", "-", b.distance,
                        b.twViolation, "-", b.expCapViolation, b.penalized);
        }

        std::unique_ptr<ScenarioCapacityEvaluator> evalHolder;
        VrpSolution best(&vrpProblem);
        long gens = 0;
        double solveSeconds = 0.0;
        const char* backend = "cpu";
        RunSolve(vrpProblem, seed, routeNum, timeLimit, /*threads=*/8, /*useGpu=*/true, evalHolder, best, gens,
                 solveSeconds, backend);

        OosScore s = ScoreOutOfSample(best, *oosEval, oosScenarios);
        double inSampleCapViol = best.getTotalCapacityViolation() / static_cast<double>(trainN);
        std::printf("%-10d %-8ld %-12.2f %-14.2f %-16.4f %-16.4f %-16.2f\n", trainN, gens, s.distance, s.twViolation,
                    inSampleCapViol, s.expCapViolation, s.penalized);
        std::printf(
            "PAPER_RESULT experiment=exp2 kind=solution backend=%s train_scenarios=%d generations=%ld "
            "solve_wall_seconds=%.17g "
            "distance=%.17g tw_violation=%.17g in_mean_capacity_violation=%.17g "
            "oos_mean_capacity_violation=%.17g oos_penalized=%.17g oos_scenarios=%d train_seed=%u oos_seed=%u "
            "algo_seed=%u capacity_aggregation=mean\n",
            backend, trainN, gens, solveSeconds, s.distance, s.twViolation, inSampleCapViol, s.expCapViolation,
            s.penalized, oosScenarios, g_trainSeed, g_outOfSampleSeed, g_algoSeed);
        std::fflush(stdout);
    }
}

void Experiment43(const std::string& instanceName, int trainScenarios, int oosScenarios,
                  const std::vector<double>& budgets) {
    std::printf("\n========== Experiment 4.3: decision quality under fixed time budgets ==========\n");
    std::printf("instance=%s  trainScenarios=%d  out-of-sample=%d scenarios  fromScratch=%d\n", instanceName.c_str(),
                trainScenarios, oosScenarios, g_fromScratch ? 1 : 0);
    std::printf("%-14s %-8s %-8s %-12s %-18s %-16s\n", "config", "budget", "gens", "distance", "oos_E[capViol]",
                "oos_penalized");

    PDPTW oosProblem;
    VrpProblem oosVrp;
    BuildProblem(instanceName, oosScenarios, g_outOfSampleSeed, oosProblem, oosVrp,
                 /*screenScenarioCapacityArcs=*/false);
    auto oosEval = MakeScenarioCapacityEvaluator(oosVrp.vrpData.demandMatrix.data(), oosVrp.vrpData.nodeNum,
                                                 oosVrp.vrpData.numScenarios, oosVrp.vrpData.vehicleCapacity,
                                                 EvaluatorBackend::kCuda);

    PDPTW problem;
    VrpProblem vrpProblem;
    BuildProblem(instanceName, trainScenarios, g_trainSeed, problem, vrpProblem);
    std::string bksPath = FindBks(instanceName);
    Solution bksSolution(problem);
    ReadSol(bksPath, bksSolution);
    VrpSolution seed(&vrpProblem);
    ConvertVrpSolution(bksSolution, seed);
    int routeNum = seed.vehicleNum + ExtraRoutes();
    std::printf("(reference: BKS distance = %.2f)\n", seed.getTotalDistance());

    struct Config {
        const char* name;
        int threads;
        bool useGpu;
    };
    std::vector<Config> configs = {{"CPU-1thread", 1, false}, {"CPU-8thread", 8, false}, {"GPU-8thread", 8, true}};
    // Optional filter via env PAPER_CONFIGS (comma list of substrings) so heavy
    // runs can skip the impractical single-thread init wall.
    if (const char* f = std::getenv("PAPER_CONFIGS")) {
        std::string filter(f);
        std::vector<Config> kept;
        for (const Config& c : configs)
            if (filter.find(c.name) != std::string::npos) kept.push_back(c);
        if (!kept.empty()) configs = kept;
    }

    for (const Config& cfg : configs) {
        for (double budget : budgets) {
            std::unique_ptr<ScenarioCapacityEvaluator> evalHolder;
            VrpSolution best(&vrpProblem);
            long gens = 0;
            double solveSeconds = 0.0;
            const char* backend = "cpu";
            RunSolve(vrpProblem, seed, routeNum, budget, cfg.threads, cfg.useGpu, evalHolder, best, gens, solveSeconds,
                     backend);
            OosScore s = ScoreOutOfSample(best, *oosEval, oosScenarios);
            char label[24];
            std::snprintf(label, sizeof(label), "%s", cfg.name);
            std::printf("%-14s %-8.0f %-8ld %-12.2f %-18.4f %-16.2f\n", label, budget, gens, s.distance,
                        s.expCapViolation, s.penalized);
            const double inSampleMean = best.getTotalCapacityViolation() / static_cast<double>(trainScenarios);
            std::printf(
                "PAPER_RESULT experiment=exp3 config=%s backend=%s budget_seconds=%.17g train_scenarios=%d "
                "generations=%ld solve_wall_seconds=%.17g distance=%.17g tw_violation=%.17g "
                "in_mean_capacity_violation=%.17g "
                "oos_mean_capacity_violation=%.17g oos_penalized=%.17g oos_scenarios=%d train_seed=%u "
                "oos_seed=%u algo_seed=%u capacity_aggregation=mean\n",
                cfg.name, backend, budget, trainScenarios, gens, solveSeconds, s.distance, s.twViolation,
                inSampleMean, s.expCapViolation, s.penalized, oosScenarios, g_trainSeed, g_outOfSampleSeed,
                g_algoSeed);
            std::fflush(stdout);
        }
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string which = "all";
    std::string instanceName = "lc101";
    double exp42TimeLimit = 45.0;
    int exp42Oos = 100000;
    int exp43Train = 10000;
    int exp43Oos = 100000;
    std::vector<double> exp43Budgets = {15.0, 45.0};

    if (argc >= 2) which = argv[1];
    if (argc >= 3) instanceName = argv[2];
    if (argc >= 4) g_capacityScale = std::atof(argv[3]);
    if (argc >= 5) exp42TimeLimit = std::atof(argv[4]);
    if (argc >= 6) exp43Train = std::atoi(argv[5]);
    if (argc >= 7) g_fromScratch = std::atoi(argv[6]) != 0;
    if (argc >= 8) {
        exp43Budgets.clear();
        std::istringstream bs(argv[7]);
        std::string tok;
        while (std::getline(bs, tok, ',')) exp43Budgets.push_back(std::atof(tok.c_str()));
    }
    if (const char* f = std::getenv("PAPER_FROM_SCRATCH")) g_fromScratch = std::atoi(f) != 0;

    g_trainSeed = SeedFromEnv("PAPER_TRAIN_SEED", kDefaultTrainSeed);
    g_outOfSampleSeed = SeedFromEnv("PAPER_OOS_SEED", kDefaultOutOfSampleSeed);
    g_algoSeed = SeedFromEnv("PAPER_ALGO_SEED", kRandomSeed);
    exp42Oos = PositiveIntFromEnv("PAPER_OOS_SCENARIOS", exp42Oos);
    exp43Oos = exp42Oos;

    mkdir("/tmp/paper_exp_results", 0755);
    // Optimise the (stochastic) capacity penalty during search. This is REQUIRED for
    // the scenario set to influence the decision: with it off the solver zeroes the
    // capacity penalty and ignores demand realisations entirely. Override with
    // PAPER_OPT_CAP=0 to fall back to the distance/TW-only behaviour.
    gMemeticOptimizeCapacity = true;
    if (const char* o = std::getenv("PAPER_OPT_CAP")) gMemeticOptimizeCapacity = std::atoi(o) != 0;
    // Treat time windows as hard so robustness is bought with distance, not TW.
    gMemeticHardTw = true;
    if (const char* h = std::getenv("PAPER_HARD_TW")) gMemeticHardTw = std::atoi(h) != 0;
    // SREX is not spare-vehicle safe; restrict to (hardened) EAX when running with
    // extra vehicles for the stochastic experiments.  Small route sets can also
    // opt out explicitly: SREX's route-subset sampler requires more routes than
    // the three-route exact-comparison instance provides.
    gMemeticDisableSrex = std::getenv("PAPER_EXTRA_ROUTES") != nullptr && ExtraRoutes() > 0;
    if (const char* s = std::getenv("PAPER_DISABLE_SREX")) gMemeticDisableSrex = std::atoi(s) != 0;

    std::printf("Paper experiments (arXiv:2511.18022) mapped onto PDPTW + GPU scenario evaluation\n");
    std::printf("NOTE: Exp 4.1 is produced by `matrix_model_benchmark` (CPU-1/CPU-8/GPU eval vs scenarios).\n");
    std::printf("instance=%s  capacityScale=%.2f\n", instanceName.c_str(), g_capacityScale);
    std::printf(
        "PAPER_CONFIG mode=%s instance=%s capacity_scale=%.17g budget_seconds=%.17g train_scenarios=%d "
        "oos_scenarios=%d train_seed=%u oos_seed=%u algo_seed=%u capacity_aggregation=mean\n",
        which.c_str(), instanceName.c_str(), g_capacityScale, exp42TimeLimit, exp43Train, exp42Oos, g_trainSeed,
        g_outOfSampleSeed, g_algoSeed);

    if (which == "exp2" || which == "all") {
        Experiment42(instanceName, exp42TimeLimit, exp42Oos);
    }
    if (which == "exp3" || which == "all") {
        Experiment43(instanceName, exp43Train, exp43Oos, exp43Budgets);
    }
    if (which == "trace3") {
        // Single-run anytime convergence trace. budget = argv[4] (exp42TimeLimit),
        // trainScenarios = argv[5] (exp43Train).
        ExperimentTrace(instanceName, exp43Train, exp42TimeLimit);
    }
    return 0;
}
