#include "io/sol_reader.h"

#include <fstream>
#include <iostream>

#include "util/string_utils.h"

bool ReadSol(const std::string& sol_file_path, Solution& solution) {
    std::ifstream ifs;
    ifs.open(sol_file_path, std::ios::in);

    if (!ifs.is_open()) {
        std::cout << "Failed to open " << sol_file_path << std::endl;
        ifs.close();
        return false;
    }

    std::string line_data;
    bool solution_section = false;
    bool success = true;

    while (getline(ifs, line_data)) {
        line_data.erase(line_data.find_last_not_of(kWhiteSpaceChars) + 1);
        if (solution_section || 0 == line_data.compare(0, 5, kRouteTag, 5)) {
            std::vector<idx_t> route;
            route.push_back(0);
            size_t start_pos = line_data.find_first_not_of(' ', line_data.find_first_of(kColonChar) + 1);

            char* str_end_ptr = nullptr;
            idx_t task_index;

            while (true) {
                size_t next_pos = line_data.find_first_of(' ', start_pos);

                if (std::string::npos == next_pos) {
                    next_pos = line_data.length();
                }

                char char_bak = *(line_data.data() + next_pos);

                if (kNullChar == char_bak) {
                    task_index = (idx_t)strtol(line_data.data() + start_pos, &str_end_ptr, 10);
                } else {
                    *(line_data.data() + next_pos) = kNullChar;
                    task_index = (idx_t)strtol(line_data.data() + start_pos, &str_end_ptr, 10);
                    *(line_data.data() + next_pos) = char_bak;
                }

                task_index = solution.problem_.original_new_task_index_map.find(task_index)->second;
                route.push_back(task_index);

                if (next_pos == line_data.length()) {
                    break;
                } else {
                    start_pos = next_pos + 1;
                }
            }

            route.push_back(0);

            solution.routes_.push_back(std::move(route));
        } else {
            if (0 == line_data.compare(0, 8, kInstanceNameTag, 8)) {
                size_t name_start_pos = line_data.find_first_of(':', 13);
                name_start_pos = line_data.find_first_not_of(" \t", name_start_pos + 1);
                std::string instance_name = line_data.substr(name_start_pos);
                if (!CaseInsensitiveStrEqual(instance_name, solution.problem_.instance_name)) {
                    std::string tmp_instance_name = solution.problem_.instance_name;
                    tmp_instance_name.erase(std::remove_if(tmp_instance_name.begin(), tmp_instance_name.end(),
                                                           [](char a) { return '_' == a; }),
                                            tmp_instance_name.end());
                    if (!CaseInsensitiveStrEqual(instance_name, tmp_instance_name)) {
                        std::cout << "The instance name in solution file is inconsistent with the name in problem."
                                  << std::endl;
                        success = false;
                        break;
                    }
                }
            } else if (0 == line_data.compare(0, 8, kSolutionTag, 8)) {
                solution_section = true;
            }
        }
    }

    solution.Evaluate();
    return success;
}