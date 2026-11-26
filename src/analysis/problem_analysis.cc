#include "analysis/problem_analysis.h"

// #include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <dirent.h>

#include "config.h"
#include "io/pdptw_reader.h"
#include "model/pdptw.h"
void AnalysisProblem() {
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

    idx_t largest_task_num = 0;
    std::cout << std::left << std::setw(20) << "Instance name" << std::setw(20) << "Task Num" << std::setw(20)
              << "Vehicle Num" << std::endl;
    for (const PDPTW& problem : problems) {
        std::cout << std::left << std::setw(20) << problem.instance_name << std::setw(20) << problem.task_num
                  << std::setw(20) << problem.vehicle_num << std::endl;
        largest_task_num = std::max(largest_task_num, problem.task_num);
    }

    std::cout << "Largest task num: " << largest_task_num << std::endl;
}