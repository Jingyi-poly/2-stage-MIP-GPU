#ifndef AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_REMOVAL_REMOVAL_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_REMOVAL_REMOVAL_H_

#include "domain/model/VrpSolution.h"
#include "util/random_utils.h"

constexpr int kRemovedStringCardinalityLimit = 10;
constexpr int kAverageRemovedTasks = 10;
constexpr int kRemovalRandomDegree = 3;
constexpr double kRequestDistanceWeight = 9.0;
constexpr double kRequestDemandDiffWeight = 2.0;
constexpr double kRequestTemporalWeight = 3.0;

static inline double GetMaxStringCardinality(double avgNodeNum) {
    return std::min((double)kRemovedStringCardinalityLimit, avgNodeNum);
}

static inline double GetMaxStringNum(double maxStringCardinality) {
    return 4.0 * kAverageRemovedTasks / (1 + maxStringCardinality) - 1.0;
}

static inline int GetStringNum(double maxStringNum) { return std::floor(GetRandDouble(1.0, maxStringNum + 1.0)); }

static inline double GetMaxRouteStringCardinality(double maxStringCardinality, int routeCardinality) {
    return std::min(maxStringCardinality, (double)routeCardinality);
}

static inline int GetRemovedStringCardinality(double maxRouteStringCardinality) {
    return std::floor(GetRandDouble(1.0, maxRouteStringCardinality + 1.0));
}

void stringRemoval(const VrpConfig& vrpConfig, VrpSolution& vrpSolution);

void worstRemoval(const VrpConfig& vrpConfig, VrpSolution& vrpSolution);

double calculateRemovalCost(const VrpConfig& vrpConfig, const VrpSolution& vrpSolution, int routeId,
                            RouteNode* pickupNode, RouteNode* deliveryNode);

void randomRemoval(const VrpConfig& vrpConfig, VrpSolution& vrpSolution);

void shawRemoval(const VrpConfig& vrpConfig, VrpSolution& vrpSolution);

#endif
