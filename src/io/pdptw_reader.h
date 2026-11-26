#ifndef AMDAHL_SRC_IO_PDPTW_READER_H_
#define AMDAHL_SRC_IO_PDPTW_READER_H_

#include <cstdint>
#include <string>

#include "model/pdptw.h"

struct TaskData {
    idx_t task_index{-1};
    pos_t pos_x{0};
    pos_t pos_y{0};
    demand_t demand{0};
    double tw_earliest{0.0};
    double tw_latest{0.0};
    double service_time{0.0};
    idx_t pickup_sibling_index{-1};
    idx_t delivery_sibling_index{-1};
};

enum class ProblemFormat : uint8_t { kLiLim = 0, kSartoriBuriol = 1 };

constexpr int kDefaultScenarioNum = 5;

void ParseVehicleData(char* line_data, int line_data_len, int& vehicle_num, demand_t& vehicle_capacity);

void ParseTaskData(char* line_data, int line_data_len, idx_t& task_index, pos_t& pos_x, pos_t& pos_y, demand_t& demand,
                   double& time_window_earliest, double& time_window_latest, double& service_time,
                   idx_t& pickup_sibling_index, idx_t& delivery_sibling_index);

// Generates `scenario_num` stochastic demand realisations for every request.
// Scenario 0 keeps the nominal (file) demand; scenarios 1.. are sampled around
// it while preserving the pickup/delivery sign symmetry (pickup[s] == -delivery[s]),
// so that the vehicle load returns to zero at the end of every request pair.
void GenerateScenarioDemands(PDPTW& problem, int scenario_num, uint32_t seed = 42);

// `screen_scenario_capacity_arcs=false` skips only the O(nodes^2 * scenarios)
// demand-capacity screening. Deterministic precedence/time-window arcs are still
// built so the converted problem remains structurally valid for evaluation-only
// held-out scenario sets.
bool ReadPDPTW(const std::string& problem_file_path, PDPTW& problem,
               ProblemFormat problem_format = ProblemFormat::kLiLim, int scenario_num = kDefaultScenarioNum,
               uint32_t demand_seed = 42, bool screen_scenario_capacity_arcs = true);

#endif
