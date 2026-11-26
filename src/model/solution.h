#ifndef AMDAHL_SRC_MODEL_SOLUTION_H_
#define AMDAHL_SRC_MODEL_SOLUTION_H_

#include "model/pdptw.h"
class Solution {
   public:
    explicit Solution(const PDPTW& problem) : problem_(problem) {}

    const PDPTW& problem_;
    std::vector<std::vector<idx_t>> routes_;
    std::vector<idx_t> unserved_tasks_;
    idx_t unserved_task_num_{0};

    std::vector<real_t> route_distance_;
    real_t total_distance_{0.0};

    std::vector<real_t> tw_violations_;
    real_t total_tw_violations_{0.0};

    std::vector<demand_t> capacity_violations_;
    demand_t total_capacity_violations_{0};

    int total_pickup_delivery_precedence_violations_{0};
    int total_pickup_delivery_pairing_violations_{0};
    int total_infeasible_arcs_violations_{0};

    void Evaluate();
};
#endif
