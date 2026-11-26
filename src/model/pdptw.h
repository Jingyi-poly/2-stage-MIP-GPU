#ifndef AMDAHL_SRC_MODEL_PDPTW_H_
#define AMDAHL_SRC_MODEL_PDPTW_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "common/constants.h"

struct PDPTW {
    std::string instance_name;
    int scenario_num;
    int vehicle_num;
    demand_t vehicle_capacity;
    int task_num;
    std::vector<pos_t> task_positions;
    std::vector<std::vector<demand_t>> task_demands;
    std::vector<real_t> task_time_windows;
    std::vector<real_t> task_service_times;
    std::vector<idx_t> new_original_task_index_map;
    std::unordered_map<idx_t, idx_t> original_new_task_index_map;
    std::vector<real_t> distance_matrix;
    std::vector<uint8_t> feasible_arcs;
    std::vector<std::vector<idx_t>> adjacency_list;
};

inline real_t GetDistance(const PDPTW& problem, idx_t start_index, idx_t to_index) {
    return problem.distance_matrix[start_index * problem.task_num + to_index];
}

inline real_t GetEarliestStartTime(const PDPTW& problem, idx_t task_index) {
    return problem.task_time_windows[2 * task_index];
}

inline real_t GetLatestStartTime(const PDPTW& problem, idx_t task_index) {
    return problem.task_time_windows[2 * task_index + 1];
}

inline uint8_t IsArcFeasible(const PDPTW& problem, idx_t arc_head, idx_t arc_tail) {
    return problem.feasible_arcs[arc_head * problem.task_num + arc_tail];
}

#endif
