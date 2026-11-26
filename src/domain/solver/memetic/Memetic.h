#ifndef AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_MEMETIC_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_MEMETIC_H_

#include "domain/model/VrpSolution.h"
#include "domain/model/entity/VrpConfig.h"

constexpr int kPopulationSize = 100;
constexpr int kMaxGeneration = 1000000;
constexpr int kThreadsNum = 8;
constexpr int kMaxNonImprovement = 2000;
constexpr double kPenaltyCoeffUpper = 20000.0;
constexpr double kPenaltyCoeffLower = 1.0;
constexpr double kSrexProb = 0.1;
constexpr double kPopulationReplaceRatio = 0.5;

// When true, the generation loop keeps optimizing the (stochastic) capacity
// objective: instead of resetting capacityPenaltyCoeff to 0, it preserves the
// caller's coefficient and lets the adaptive scheme operate around it. Default
// false preserves the original behaviour (capacity assumed feasible, search
// drives TW/distance/fleet). Used by the CVRPSD-style stochastic experiments.
extern bool gMemeticOptimizeCapacity;

// When true, time windows are treated as a HARD constraint: the TW penalty
// coefficient is kept fixed (and high) instead of being adaptively reduced, so the
// search never trades TW feasibility for capacity feasibility. Used together with
// gMemeticOptimizeCapacity for the stochastic-capacity experiments where TW is hard
// and only capacity is the (soft, stochastic) objective.
extern bool gMemeticHardTw;

// When true, the SREX crossover is disabled (only EAX is used). SREX's route-subset
// selection is not robust to spare/empty vehicles; EAX has been hardened against the
// degenerate case. Used by the spare-fleet stochastic experiments.
extern bool gMemeticDisableSrex;

void buildInitialSolution(int routeNum, VrpSolution& vrpSolution, int type);

void memetic(VrpConfig& vrpConfig, int routeNum, double timeLimit, const VrpSolution& bks,
             const std::vector<VrpSolution>& initialSols, VrpProblem& vrpProblem, VrpSolution& bestSolution,
             const std::string& resultFolder, int numThreads = kThreadsNum);

#endif