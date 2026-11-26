#ifndef AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_LARGE_NEIGHBORHOOD_SEARCH_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_LARGE_NEIGHBORHOOD_SEARCH_H_

#include "domain/model/VrpSolution.h"
#include "domain/model/entity/VrpConfig.h"

constexpr double kRandomRemovalProb = 0.1;
constexpr double kWorstRemovalProb = 0.2;
constexpr double kShawRemovalProb = 0.2;

void insertEmptyRoute(VrpSolution& vrpSolution);

void largeNeighborhoodSearch(const VrpConfig& vrpConfig, VrpSolution& vrpSolution);

#endif