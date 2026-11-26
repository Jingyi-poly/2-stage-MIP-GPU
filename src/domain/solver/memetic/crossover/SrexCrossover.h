#ifndef AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_CROSSOVER_SREX_CROSSOVER_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_CROSSOVER_SREX_CROSSOVER_H_

#include <chrono>

#include "domain/model/VrpProblem.h"
#include "domain/model/VrpSolution.h"
#include "domain/model/entity/VrpConfig.h"

constexpr int kSrexRouteSubsetNum = 20;
constexpr int kCrossoverChildSolNum = 10;

void srexCrossover(const VrpConfig& vrpConfig, VrpSolution& firstSol, VrpSolution& secondSol,
                   std::chrono::steady_clock::time_point startTime, double timeLimit, VrpSolution& childSol);

#endif