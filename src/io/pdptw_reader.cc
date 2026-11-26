#include "io/pdptw_reader.h"

// #include <filesystem>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <unordered_set>

#include "util/distance_utils.h"
#include "util/string_utils.h"

void ParseVehicleData(char* line_data, int line_data_len, int& vehicle_num, demand_t& vehicle_capacity) {
    int start_pos = 0;
    int end_pos = 0;

    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char char_bak = *(line_data + end_pos);
    char* str_end_ptr = nullptr;

    if (kNullChar == char_bak) {
        vehicle_num = (int)strtol(line_data + start_pos, &str_end_ptr, 10);
    } else {
        *(line_data + end_pos) = kNullChar;
        vehicle_num = (int)strtol(line_data + start_pos, &str_end_ptr, 10);
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    if (char_bak == kNullChar) {
        vehicle_capacity = static_cast<demand_t>(strtod(line_data + start_pos, &str_end_ptr));
    } else {
        *(line_data + end_pos) = kNullChar;
        vehicle_capacity = static_cast<demand_t>(strtod(line_data + start_pos, &str_end_ptr));
        *(line_data + end_pos) = char_bak;
    }
}

void ParseTaskData(char* line_data, int line_data_len, idx_t& task_index, pos_t& pos_x, pos_t& pos_y, demand_t& demand,
                   double& time_window_earliest, double& time_window_latest, double& service_time,
                   idx_t& pickup_sibling_index, idx_t& delivery_sibling_index) {
    int start_pos = 0;
    int end_pos = 0;

    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char char_bak = *(line_data + end_pos);
    char* str_end_ptr = nullptr;

    if (kNullChar == char_bak) {
        task_index = (idx_t)strtol(line_data + start_pos, &str_end_ptr, 10);
    } else {
        *(line_data + end_pos) = kNullChar;
        task_index = (idx_t)strtol(line_data + start_pos, &str_end_ptr, 10);
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        pos_x = static_cast<pos_t>(strtod(line_data + start_pos, &str_end_ptr));
    } else {
        *(line_data + end_pos) = kNullChar;
        pos_x = static_cast<pos_t>(strtod(line_data + start_pos, &str_end_ptr));
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        pos_y = static_cast<pos_t>(strtod(line_data + start_pos, &str_end_ptr));
    } else {
        *(line_data + end_pos) = kNullChar;
        pos_y = static_cast<pos_t>(strtod(line_data + start_pos, &str_end_ptr));
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        demand = static_cast<demand_t>(strtod(line_data + start_pos, &str_end_ptr));
    } else {
        *(line_data + end_pos) = kNullChar;
        demand = static_cast<demand_t>(strtod(line_data + start_pos, &str_end_ptr));
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        time_window_earliest = strtod(line_data + start_pos, &str_end_ptr);
    } else {
        *(line_data + end_pos) = kNullChar;
        time_window_earliest = strtod(line_data + start_pos, &str_end_ptr);
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        time_window_latest = strtod(line_data + start_pos, &str_end_ptr);
    } else {
        *(line_data + end_pos) = kNullChar;
        time_window_latest = strtod(line_data + start_pos, &str_end_ptr);
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        service_time = strtod(line_data + start_pos, &str_end_ptr);
    } else {
        *(line_data + end_pos) = kNullChar;
        service_time = strtod(line_data + start_pos, &str_end_ptr);
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        pickup_sibling_index = (idx_t)strtol(line_data + start_pos, &str_end_ptr, 10);
    } else {
        *(line_data + end_pos) = kNullChar;
        pickup_sibling_index = (idx_t)strtol(line_data + start_pos, &str_end_ptr, 10);
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        delivery_sibling_index = (idx_t)strtol(line_data + start_pos, &str_end_ptr, 10);
    } else {
        *(line_data + end_pos) = kNullChar;
        delivery_sibling_index = (idx_t)strtol(line_data + start_pos, &str_end_ptr, 10);
        *(line_data + end_pos) = char_bak;
    }
}

void GenerateScenarioDemands(PDPTW& problem, int scenario_num, uint32_t seed) {
    // Pickups occupy odd new-indices, their delivery sibling is the next (even) index.
    // Scenario 0 is the nominal demand read from the instance file; the remaining
    // scenarios multiply the nominal pickup demand by a random factor and mirror it
    // onto the delivery so that the per-request load stays balanced.
    std::mt19937 generator(seed);
    // Demand stochasticity: nominal demand is multiplied by Uniform(1-spread, 1+spread).
    // Default spread 0.5 reproduces the original Uniform(0.5, 1.5). Widen via the
    // PDPTW_DEMAND_SPREAD env var to make the stochastic capacity constraint bind harder.
    double spread = 0.5;
    if (const char* s = std::getenv("PDPTW_DEMAND_SPREAD")) {
        double v = std::atof(s);
        if (v > 0.0) spread = v;
    }
    double lo = 1.0 - spread;
    if (lo < 0.0) lo = 0.0;
    std::uniform_real_distribution<double> factor_distribution(lo, 1.0 + spread);

    for (idx_t pickup_index = 1; pickup_index < problem.task_num; pickup_index += 2) {
        idx_t delivery_index = pickup_index + 1;
        if (delivery_index >= problem.task_num) {
            break;
        }

        demand_t nominal_pickup_demand = problem.task_demands[pickup_index][0];
        for (int scenario = 1; scenario < scenario_num; ++scenario) {
            double factor = factor_distribution(generator);
            demand_t sampled_demand = static_cast<demand_t>(std::lround(nominal_pickup_demand * factor));
            problem.task_demands[pickup_index][scenario] = sampled_demand;
            problem.task_demands[delivery_index][scenario] = -sampled_demand;
        }
    }
}

bool ReadPDPTW(const std::string& problem_file_path, PDPTW& problem, ProblemFormat problem_format, int scenario_num,
               uint32_t demand_seed, bool screen_scenario_capacity_arcs) {
    const int kScenarioNum = scenario_num > 0 ? scenario_num : kDefaultScenarioNum;
    problem.scenario_num = kScenarioNum;

    std::ifstream ifs;
    ifs.open(problem_file_path, std::ios::in);

    if (!ifs.is_open()) {
        std::cout << "Failed to open " << problem_file_path << std::endl;
        ifs.close();
        return false;
    }

    // std::filesystem::path path = problem_file_path;
    // problem.instance_name = path.stem();

    // 找到最后一个 '/' 和最后一个 '.' 的位置
    size_t last_slash_idx = problem_file_path.rfind('/');
    size_t last_dot_idx = problem_file_path.rfind('.');

    // 检查是否找到 '/' 和 '.'
    if (last_slash_idx != std::string::npos && last_dot_idx != std::string::npos) {
        // 确保 '.' 在 '/' 之后
        if (last_dot_idx > last_slash_idx) {
            // 提取文件名（不包含扩展名）
            problem.instance_name = problem_file_path.substr(last_slash_idx + 1, last_dot_idx - last_slash_idx - 1);
        }
    }

    if (ProblemFormat::kLiLim == problem_format) {
        std::string line_data;
        bool first_line = true;

        idx_t depot_task_index;
        std::unordered_map<idx_t, TaskData> task_data_map;
        while (getline(ifs, line_data)) {
            line_data.erase(line_data.find_last_not_of(kWhiteSpaceChars) + 1);
            if (first_line) {
                ParseVehicleData(&line_data[0], (int)line_data.length(), problem.vehicle_num, problem.vehicle_capacity);
                first_line = false;
            } else {
                TaskData tmp_task_data;
                ParseTaskData(&line_data[0], (int)line_data.length(), tmp_task_data.task_index, tmp_task_data.pos_x,
                              tmp_task_data.pos_y, tmp_task_data.demand, tmp_task_data.tw_earliest,
                              tmp_task_data.tw_latest, tmp_task_data.service_time, tmp_task_data.pickup_sibling_index,
                              tmp_task_data.delivery_sibling_index);

                if (0 == tmp_task_data.pickup_sibling_index && 0 == tmp_task_data.delivery_sibling_index) {
                    depot_task_index = tmp_task_data.task_index;
                }
                task_data_map[tmp_task_data.task_index] = tmp_task_data;
            }
        }

        problem.task_num = (int)task_data_map.size();
        problem.new_original_task_index_map.resize(problem.task_num);
        problem.task_demands.resize(problem.task_num);
        problem.task_positions.resize(problem.task_num * 2);
        problem.task_time_windows.resize(problem.task_num * 2);
        problem.task_service_times.resize(problem.task_num);

        problem.task_demands[0].resize(kScenarioNum, 0.0);
        problem.original_new_task_index_map[depot_task_index] = 0;
        problem.new_original_task_index_map[0] = depot_task_index;
        problem.task_positions[0] = task_data_map[depot_task_index].pos_x;
        problem.task_positions[1] = task_data_map[depot_task_index].pos_y;
        problem.task_service_times[0] = task_data_map[depot_task_index].service_time;
        problem.task_time_windows[0] = task_data_map[depot_task_index].tw_earliest;
        problem.task_time_windows[1] = task_data_map[depot_task_index].tw_latest;

        idx_t task_counter = 1;
        for (const auto& [index, task_data] : task_data_map) {
            if (index == depot_task_index) {
                continue;
            }

            idx_t delivery_sibling_index = task_data.delivery_sibling_index;
            if (0 == delivery_sibling_index) {
                continue;
            }

            problem.original_new_task_index_map[index] = task_counter;
            problem.original_new_task_index_map[delivery_sibling_index] = (idx_t)(task_counter + 1);
            problem.new_original_task_index_map[task_counter] = index;
            problem.new_original_task_index_map[task_counter + 1] = delivery_sibling_index;

            problem.task_positions[2 * task_counter] = task_data_map[index].pos_x;
            problem.task_positions[2 * task_counter + 1] = task_data_map[index].pos_y;
            problem.task_demands[task_counter].resize(kScenarioNum, 0.0);
            problem.task_demands[task_counter][0] = task_data_map[index].demand;
            problem.task_service_times[task_counter] = task_data_map[index].service_time;
            problem.task_time_windows[2 * task_counter] = task_data_map[index].tw_earliest;
            problem.task_time_windows[2 * task_counter + 1] = task_data_map[index].tw_latest;

            problem.task_positions[2 * task_counter + 2] = task_data_map[delivery_sibling_index].pos_x;
            problem.task_positions[2 * task_counter + 3] = task_data_map[delivery_sibling_index].pos_y;
            problem.task_demands[task_counter + 1].resize(problem.scenario_num, 0.0);
            problem.task_demands[task_counter + 1][0] = task_data_map[delivery_sibling_index].demand;
            problem.task_service_times[task_counter + 1] = task_data_map[delivery_sibling_index].service_time;
            problem.task_time_windows[2 * task_counter + 2] = task_data_map[delivery_sibling_index].tw_earliest;
            problem.task_time_windows[2 * task_counter + 3] = task_data_map[delivery_sibling_index].tw_latest;

            task_counter += 2;
        }

        ifs.close();

        problem.distance_matrix.resize(problem.task_num * problem.task_num);
        for (idx_t i = 0; i < problem.task_num; ++i) {
            idx_t offset = i * problem.task_num;
            pos_t first_x = problem.task_positions[2 * i];
            pos_t first_y = problem.task_positions[2 * i + 1];
            for (idx_t j = 0; j < problem.task_num; ++j) {
                pos_t second_x = problem.task_positions[2 * j];
                pos_t second_y = problem.task_positions[2 * j + 1];
                real_t distance = GetEuclideanDis(first_x, first_y, second_x, second_y);
                problem.distance_matrix[offset + j] = distance;
            }
        }

    } else {
        std::string line_data;

        std::unordered_map<idx_t, TaskData> task_data_map;
        std::vector<std::vector<real_t>> distance_matrix;

        bool nodes_section = false;
        bool edges_section = false;
        char* str_end_ptr = nullptr;
        while (getline(ifs, line_data)) {
            line_data.erase(line_data.find_last_not_of(kWhiteSpaceChars) + 1);

            if (5 == line_data.size() && "NODES" == line_data) {
                nodes_section = true;
                edges_section = false;
                continue;
            } else if (5 == line_data.size() && "EDGES" == line_data) {
                edges_section = true;
                nodes_section = false;
                continue;
            } else if (nodes_section) {
                TaskData tmp_task_data;
                ParseTaskData(&line_data[0], (int)line_data.length(), tmp_task_data.task_index, tmp_task_data.pos_x,
                              tmp_task_data.pos_y, tmp_task_data.demand, tmp_task_data.tw_earliest,
                              tmp_task_data.tw_latest, tmp_task_data.service_time, tmp_task_data.pickup_sibling_index,
                              tmp_task_data.delivery_sibling_index);
                task_data_map[tmp_task_data.task_index] = tmp_task_data;
                continue;
            } else if (edges_section) {
                std::istringstream string_stream(line_data);
                char white_space = ' ';
                std::string token;
                std::vector<real_t> distance;
                distance.reserve(std::max(100, problem.task_num));
                while (getline(string_stream, token, white_space)) {
                    distance.push_back(static_cast<real_t>(strtod(token.c_str(), &str_end_ptr)));
                }

                distance_matrix.push_back(std::move(distance));
                continue;
            } else if ("SIZE" == line_data.substr(0, 4)) {
                problem.task_num = static_cast<int>(strtol(line_data.c_str() + 6, &str_end_ptr, 10));
                continue;
            } else if ("CAPACITY" == line_data.substr(0, 8)) {
                problem.vehicle_capacity = static_cast<demand_t>(strtod(line_data.c_str() + 10, &str_end_ptr));
                continue;
            } else {
                continue;
            }
        }

        problem.vehicle_num = (problem.task_num - 1) / 2;
        problem.new_original_task_index_map.resize(problem.task_num);
        problem.task_demands.resize(problem.task_num);
        problem.task_positions.resize(problem.task_num * 2);
        problem.task_time_windows.resize(problem.task_num * 2);
        problem.task_service_times.resize(problem.task_num);

        int depot_task_index = 0;
        problem.original_new_task_index_map[depot_task_index] = 0;
        problem.new_original_task_index_map[0] = depot_task_index;
        problem.task_positions[0] = task_data_map[depot_task_index].pos_x;
        problem.task_positions[1] = task_data_map[depot_task_index].pos_y;
        problem.task_service_times[0] = task_data_map[depot_task_index].service_time;
        problem.task_time_windows[0] = task_data_map[depot_task_index].tw_earliest;
        problem.task_time_windows[1] = task_data_map[depot_task_index].tw_latest;

        idx_t task_counter = 1;
        for (const auto& [index, task_data] : task_data_map) {
            if (index == depot_task_index) {
                continue;
            }

            idx_t delivery_sibling_index = task_data.delivery_sibling_index;
            if (0 == delivery_sibling_index) {
                continue;
            }

            problem.original_new_task_index_map[index] = task_counter;
            problem.original_new_task_index_map[delivery_sibling_index] = (idx_t)(task_counter + 1);
            problem.new_original_task_index_map[task_counter] = index;
            problem.new_original_task_index_map[task_counter + 1] = delivery_sibling_index;

            problem.task_positions[2 * task_counter] = task_data_map[index].pos_x;
            problem.task_positions[2 * task_counter + 1] = task_data_map[index].pos_y;
            problem.task_demands[task_counter].resize(problem.scenario_num, 0.0);
            problem.task_demands[task_counter][0] = task_data_map[index].demand;
            problem.task_service_times[task_counter] = task_data_map[index].service_time;
            problem.task_time_windows[2 * task_counter] = task_data_map[index].tw_earliest;
            problem.task_time_windows[2 * task_counter + 1] = task_data_map[index].tw_latest;

            problem.task_positions[2 * task_counter + 2] = task_data_map[delivery_sibling_index].pos_x;
            problem.task_positions[2 * task_counter + 3] = task_data_map[delivery_sibling_index].pos_y;
            problem.task_demands[task_counter + 1].resize(problem.scenario_num, 0.0);
            problem.task_demands[task_counter + 1][0] = task_data_map[delivery_sibling_index].demand;
            problem.task_service_times[task_counter + 1] = task_data_map[delivery_sibling_index].service_time;
            problem.task_time_windows[2 * task_counter + 2] = task_data_map[delivery_sibling_index].tw_earliest;
            problem.task_time_windows[2 * task_counter + 3] = task_data_map[delivery_sibling_index].tw_latest;

            task_counter += 2;
        }

        ifs.close();

        problem.distance_matrix.resize(problem.task_num * problem.task_num);
        for (idx_t i = 0; i < problem.task_num; ++i) {
            idx_t offset = i * problem.task_num;
            idx_t original_index = problem.new_original_task_index_map[i];
            for (idx_t j = 0; j < problem.task_num; ++j) {
                idx_t original_index_2 = problem.new_original_task_index_map[j];
                problem.distance_matrix[offset + j] = distance_matrix[original_index][original_index_2];
            }
        }
    }

    GenerateScenarioDemands(problem, problem.scenario_num, demand_seed);

    std::unordered_set<idx_t> time_window_tightened;
    for (idx_t i = 1; i < problem.task_num; ++i) {
        if (i % 2 == 0) {
            continue;
        }
        real_t earliest_pickup_time = problem.task_time_windows[2 * i];
        real_t latest_pickup_time = problem.task_time_windows[2 * i + 1];
        real_t earliest_delivery_time = problem.task_time_windows[2 * (i + 1)];
        real_t latest_delivery_time = problem.task_time_windows[2 * (i + 1) + 1];

        real_t actual_earliest_delivery_time =
            earliest_pickup_time + problem.task_service_times[i] + GetDistance(problem, i, i + 1);

        if (earliest_delivery_time < actual_earliest_delivery_time) {
            time_window_tightened.insert(i);
            problem.task_time_windows[2 * (i + 1)] = actual_earliest_delivery_time;
        }

        real_t actual_latest_pickup_time =
            latest_delivery_time - problem.task_service_times[i] - GetDistance(problem, i, i + 1);
        if (actual_latest_pickup_time < latest_pickup_time) {
            time_window_tightened.insert(i);
            problem.task_time_windows[2 * i + 1] = actual_latest_pickup_time;
        }
    }
    std::cout << "Time windows tightened: " << time_window_tightened.size() << "/"
              << std::to_string((problem.task_num - 1) / 2) << std::endl;

    problem.feasible_arcs.assign(problem.task_num * problem.task_num, 1);
    problem.feasible_arcs[0] = 0;
    real_t first_earliest_arrival_time;
    real_t first_earliest_start_service_time;
    real_t second_earliest_arrival_time;
    real_t second_earliest_start_service_time;
    real_t third_earliest_arrival_time;
    real_t third_earliest_start_service_time;
    real_t fourth_earliest_arrival_time;
    real_t fourth_earliest_start_service_time;
    for (idx_t i = 1; i < problem.task_num; ++i) {
        idx_t offset = i * problem.task_num;

        if (0 == i % 2) {
            problem.feasible_arcs[i] = 0;
        } else {
            problem.feasible_arcs[offset] = 0;
        }

        for (idx_t j = 1; j < problem.task_num; ++j) {
            if (i == j) {
                problem.feasible_arcs[offset + j] = 0;
                continue;
            }

            if (0 == i % 2 && j == i - 1) {
                problem.feasible_arcs[offset + j] = 0;
                continue;
            }

            if (screen_scenario_capacity_arcs) {
                if (1 == i % 2) {

                    if (j != i + 1) {
                        bool capacity_feasible = true;
                        for (int k = 0; k < problem.scenario_num; ++k) {
                            if (problem.task_demands[i][k] + std::abs(problem.task_demands[j][k]) >
                                problem.vehicle_capacity) {
                                capacity_feasible = false;
                                break;
                            }
                        }

                        if (!capacity_feasible) {
                            problem.feasible_arcs[offset + j] = 0;
                            continue;
                        }
                    }
                } else {
                    if (0 == j % 2) {
                        bool capacity_feasible = true;

                        for (int k = 0; k < problem.scenario_num; ++k) {
                            if (std::abs(problem.task_demands[i][k]) + std::abs(problem.task_demands[j][k]) >
                                problem.vehicle_capacity) {
                                capacity_feasible = false;
                                break;
                            }
                        }

                        if (!capacity_feasible) {
                            problem.feasible_arcs[offset + j] = 0;
                            continue;
                        }
                    }
                }
            }

            if (1 == i % 2 && j == i + 1) {
                continue;
            }

            if (1 == i % 2 && 1 == j % 2) {
                first_earliest_start_service_time =
                    std::max(GetEarliestStartTime(problem, i),
                             problem.task_time_windows[0] + problem.task_service_times[0] + GetDistance(problem, 0, i));

                second_earliest_arrival_time =
                    first_earliest_start_service_time + problem.task_service_times[i] + GetDistance(problem, i, j);
                if (second_earliest_arrival_time > GetLatestStartTime(problem, j)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
                second_earliest_start_service_time =
                    std::max(second_earliest_arrival_time, GetEarliestStartTime(problem, j));
                bool feasible_arc = false;

                third_earliest_arrival_time =
                    second_earliest_start_service_time + problem.task_service_times[j] + GetDistance(problem, j, i + 1);
                if (third_earliest_arrival_time <= GetLatestStartTime(problem, i + 1)) {
                    third_earliest_start_service_time =
                        std::max(third_earliest_arrival_time, GetEarliestStartTime(problem, i + 1));
                    fourth_earliest_arrival_time = third_earliest_start_service_time +
                                                   problem.task_service_times[i + 1] +
                                                   GetDistance(problem, i + 1, j + 1);
                    if (fourth_earliest_arrival_time <= GetLatestStartTime(problem, j + 1)) {
                        fourth_earliest_start_service_time =
                            std::max(fourth_earliest_arrival_time, GetEarliestStartTime(problem, j + 1));
                        if (fourth_earliest_start_service_time + problem.task_service_times[j + 1] +
                                GetDistance(problem, j + 1, 0) <=
                            GetLatestStartTime(problem, 0)) {
                            feasible_arc = true;
                        }
                    }
                }

                if (!feasible_arc) {
                    third_earliest_arrival_time = second_earliest_start_service_time + problem.task_service_times[j] +
                                                  GetDistance(problem, j, j + 1);
                    if (third_earliest_arrival_time <= GetLatestStartTime(problem, j + 1)) {
                        third_earliest_start_service_time =
                            std::max(third_earliest_arrival_time, GetEarliestStartTime(problem, j + 1));
                        fourth_earliest_arrival_time = third_earliest_start_service_time +
                                                       problem.task_service_times[j + 1] +
                                                       GetDistance(problem, j + 1, i + 1);
                        if (fourth_earliest_arrival_time <= GetLatestStartTime(problem, i + 1)) {
                            fourth_earliest_start_service_time =
                                std::max(fourth_earliest_arrival_time, GetEarliestStartTime(problem, i + 1));
                            if (fourth_earliest_start_service_time + problem.task_service_times[i + 1] +
                                    GetDistance(problem, i + 1, 0) <=
                                GetLatestStartTime(problem, 0)) {
                                feasible_arc = true;
                            }
                        }
                    }
                }

                if (!feasible_arc) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
            }

            if (1 == i % 2 && 0 == j % 2) {
                first_earliest_arrival_time =
                    GetEarliestStartTime(problem, 0) + problem.task_service_times[0] + GetDistance(problem, 0, j - 1);
                first_earliest_start_service_time =
                    std::max(first_earliest_arrival_time, GetEarliestStartTime(problem, j - 1));
                second_earliest_arrival_time = first_earliest_start_service_time + problem.task_service_times[j - 1] +
                                               GetDistance(problem, j - 1, i);
                if (second_earliest_arrival_time > GetLatestStartTime(problem, i)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }

                second_earliest_start_service_time =
                    std::max(second_earliest_arrival_time, GetEarliestStartTime(problem, i));
                third_earliest_arrival_time =
                    second_earliest_start_service_time + problem.task_service_times[i] + GetDistance(problem, i, j);
                if (third_earliest_arrival_time > GetLatestStartTime(problem, j)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }

                third_earliest_start_service_time =
                    std::max(third_earliest_arrival_time, GetEarliestStartTime(problem, j));
                fourth_earliest_arrival_time =
                    third_earliest_start_service_time + problem.task_service_times[j] + GetDistance(problem, j, i + 1);
                if (fourth_earliest_arrival_time > GetLatestStartTime(problem, i + 1)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }

                fourth_earliest_start_service_time =
                    std::max(fourth_earliest_arrival_time, GetEarliestStartTime(problem, i + 1));
                if (fourth_earliest_start_service_time + problem.task_service_times[i + 1] +
                        GetDistance(problem, i + 1, 0) >
                    GetLatestStartTime(problem, 0)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
            }

            if (0 == i % 2 && 1 == j % 2) {
                first_earliest_arrival_time =
                    GetEarliestStartTime(problem, 0) + problem.task_service_times[0] + GetDistance(problem, 0, i - 1);
                first_earliest_start_service_time =
                    std::max(first_earliest_arrival_time, GetEarliestStartTime(problem, i - 1));
                second_earliest_arrival_time = first_earliest_start_service_time + problem.task_service_times[i - 1] +
                                               GetDistance(problem, i - 1, i);

                second_earliest_start_service_time =
                    std::max(second_earliest_arrival_time, GetEarliestStartTime(problem, i));
                third_earliest_arrival_time =
                    second_earliest_start_service_time + problem.task_service_times[i] + GetDistance(problem, i, j);

                if (third_earliest_arrival_time > GetLatestStartTime(problem, j)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
                third_earliest_start_service_time =
                    std::max(third_earliest_arrival_time, GetEarliestStartTime(problem, j));
                fourth_earliest_arrival_time =
                    third_earliest_start_service_time + problem.task_service_times[j] + GetDistance(problem, j, j + 1);
                if (fourth_earliest_arrival_time > GetLatestStartTime(problem, j + 1)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
                fourth_earliest_start_service_time =
                    std::max(fourth_earliest_arrival_time, GetEarliestStartTime(problem, j + 1));
                if (fourth_earliest_start_service_time + problem.task_service_times[j + 1] +
                        GetDistance(problem, j + 1, 0) >
                    GetLatestStartTime(problem, 0)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
            }

            if (0 == i % 2 && 0 == j % 2) {
                bool feasible_arc = false;
                first_earliest_arrival_time =
                    GetEarliestStartTime(problem, 0) + problem.task_service_times[0] + GetDistance(problem, 0, i - 1);
                first_earliest_start_service_time =
                    std::max(first_earliest_arrival_time, GetEarliestStartTime(problem, i - 1));
                second_earliest_arrival_time = first_earliest_start_service_time + problem.task_service_times[i - 1] +
                                               GetDistance(problem, i - 1, j - 1);
                if (second_earliest_arrival_time <= GetLatestStartTime(problem, j - 1)) {
                    second_earliest_start_service_time =
                        std::max(second_earliest_arrival_time, GetEarliestStartTime(problem, j - 1));
                    third_earliest_arrival_time = second_earliest_start_service_time +
                                                  problem.task_service_times[j - 1] + GetDistance(problem, j - 1, i);
                    if (third_earliest_arrival_time <= GetLatestStartTime(problem, i)) {
                        third_earliest_start_service_time =
                            std::max(third_earliest_arrival_time, GetEarliestStartTime(problem, i));
                        fourth_earliest_arrival_time = third_earliest_start_service_time +
                                                       problem.task_service_times[i] + GetDistance(problem, i, j);
                        if (fourth_earliest_arrival_time <= GetLatestStartTime(problem, j)) {
                            fourth_earliest_start_service_time =
                                std::max(fourth_earliest_arrival_time, GetEarliestStartTime(problem, j));
                            if (fourth_earliest_start_service_time + problem.task_service_times[j] +
                                    GetDistance(problem, j, 0) <=
                                GetLatestStartTime(problem, 0)) {
                                feasible_arc = true;
                            }
                        }
                    }
                }

                if (!feasible_arc) {
                    first_earliest_arrival_time = GetEarliestStartTime(problem, 0) + problem.task_service_times[0] +
                                                  GetDistance(problem, 0, j - 1);
                    first_earliest_start_service_time =
                        std::max(first_earliest_arrival_time, GetEarliestStartTime(problem, j - 1));
                    second_earliest_arrival_time = first_earliest_start_service_time +
                                                   problem.task_service_times[j - 1] +
                                                   GetDistance(problem, j - 1, i - 1);
                    if (second_earliest_arrival_time <= GetLatestStartTime(problem, i - 1)) {
                        second_earliest_start_service_time =
                            std::max(second_earliest_arrival_time, GetEarliestStartTime(problem, i - 1));
                        third_earliest_arrival_time = second_earliest_start_service_time +
                                                      problem.task_service_times[i - 1] +
                                                      GetDistance(problem, i - 1, i);
                        if (third_earliest_arrival_time <= GetLatestStartTime(problem, i)) {
                            third_earliest_start_service_time =
                                std::max(third_earliest_arrival_time, GetEarliestStartTime(problem, i));
                            fourth_earliest_arrival_time = third_earliest_start_service_time +
                                                           problem.task_service_times[i] + GetDistance(problem, i, j);
                            if (fourth_earliest_arrival_time <= GetLatestStartTime(problem, j)) {
                                fourth_earliest_start_service_time =
                                    std::max(fourth_earliest_arrival_time, GetEarliestStartTime(problem, j));
                                if (fourth_earliest_start_service_time + problem.task_service_times[j] +
                                        GetDistance(problem, j, 0) <=
                                    GetLatestStartTime(problem, 0)) {
                                    feasible_arc = true;
                                }
                            }
                        }
                    }
                }

                if (!feasible_arc) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
            }
        }
    }

    problem.adjacency_list.resize(problem.task_num);
    for (idx_t i = 0; i < problem.task_num; ++i) {
        problem.adjacency_list[i].resize(problem.task_num);
        std::iota(problem.adjacency_list[i].begin(), problem.adjacency_list[i].end(), 0);
        int tmp_offset = i * problem.task_num;
        std::sort(problem.adjacency_list[i].begin(), problem.adjacency_list[i].begin() + problem.task_num,
                  [&](idx_t a, idx_t b) {
                      return problem.distance_matrix[tmp_offset + a] < problem.distance_matrix[tmp_offset + b];
                  });
    }
    return true;
}
