#ifndef AMDAHL_SRC_DOMAIN_MODEL_VRP_PROBLEM_H_
#define AMDAHL_SRC_DOMAIN_MODEL_VRP_PROBLEM_H_

#include <unordered_map>

#include "domain/model/entity/VrpData.h"

class ScenarioCapacityEvaluator;

class VrpProblem {
   public:
    VrpData vrpData;
    std::vector<std::vector<int>> adjacentList;
    std::unordered_map<int, std::unordered_map<int, double>> requestDistanceMap;
    std::unordered_map<int, std::unordered_map<int, double>> requestDemandDiffMap;

    // When set, updateRouteData delegates the per-scenario capacity computation
    // to this (GPU) evaluator instead of the inline CPU loop. The CPU path is
    // fully preserved and used whenever this is disabled, so both versions can be
    // compared from the same binary via the `useGpu` run-config switch.
    bool useGpuCapacityEval{false};

    // When set, greedyInsertWithBlinks evaluates the per-scenario capacity-violation
    // delta of all candidate insertions on the (GPU) evaluator: one batched launch
    // per request covers every route, and only O(L^2) scalars per route come back.
    // The inline CPU enumeration is preserved and used whenever this is disabled.
    bool useGpuInsertionEval{false};

    ScenarioCapacityEvaluator* capacityEvaluator{nullptr};
};
#endif