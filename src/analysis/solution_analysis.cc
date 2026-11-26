#include "analysis/solution_analysis.h"

// #include <filesystem>
#include <dirent.h>
#include <iomanip>
#include <iostream>
#include <vector>

#include "config.h"
#include "io/pdptw_reader.h"
#include "io/sol_reader.h"
#include "model/pdptw.h"
#include "model/solution.h"
#include "util/string_utils.h"

void AnalysisSolution() {
    std::string instance_folder = AMDAHL_DIR;
    instance_folder += "/data";
    instance_folder += "/instance/";

    DIR* dir = opendir(instance_folder.c_str());
    if (dir == nullptr) {
        std::cerr << "Unable to open directory: " << instance_folder << std::endl;
        return;
    }
    std::vector<PDPTW> problems;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        // 忽略目录项，只处理文件
        if (entry->d_type == DT_REG) {
            std::string file_path = instance_folder + "/" + entry->d_name;
            problems.emplace_back();
            ReadPDPTW(file_path, problems.back());
        }
    }

    // 关闭目录
    closedir(dir);

    std::string bks_folder = AMDAHL_DIR;
    bks_folder += "/data";
    bks_folder += "/bks/";

    std::cout << std::left << std::setw(16) << "Instance" << std::setw(16) << "Vehicle Num" << std::setw(16)
              << "Total Distance" << std::setw(20) << "TW Violations" << std::setw(20) << "Capacity Violations"
              << std::endl;
    for (const PDPTW& problem : problems) {
        std::string instance_name = ToLower(problem.instance_name);

        DIR* dir = opendir(bks_folder.c_str());
        if (dir == nullptr) {
            std::cerr << "Unable to open directory: " << bks_folder << std::endl;
            exit(0);
        }
        
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string file_path;
            std::string file_name;
            // 忽略目录项，只处理文件
            if (entry->d_type == DT_REG) {
                file_path = bks_folder + "/" + entry->d_name;
                file_name = entry->d_name;

                // 提取文件名（不包含扩展名）
                size_t last_dot_pos = file_name.find_last_of('.');
                if (last_dot_pos != std::string::npos) {
                    file_name = file_name.substr(0, last_dot_pos);
                }
            } else {
                continue;
            }

            if (file_name.size() < instance_name.size()) {
                continue;
            }

            if (0 != file_name.compare(0, instance_name.size(), instance_name, 0, instance_name.size())) {
                continue;
            }

            if (file_name.size() > instance_name.size()) {
                char tmp_char = file_name[instance_name.size()];
                if ('0' <= tmp_char && '9' >= tmp_char) {
                    continue;
                }
            }

            Solution solution(problem);
            ReadSol(file_path, solution);

            std::cout.precision(2);
            std::cout.flags(std::ostream::fixed);
            std::cout << std::left << std::setw(16) << instance_name << std::setw(16) << solution.routes_.size()
                      << std::setw(16) << solution.total_distance_ << std::setw(20) << solution.total_tw_violations_
                      << std::setw(20) << solution.total_capacity_violations_ << std::endl;
        }
    }
}