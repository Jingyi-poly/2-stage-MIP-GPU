#include "io/sol_writer.h"

#include <algorithm>
// #include <filesystem>
#include <fstream>
#include <limits>
#include <iostream>
#include <numeric>
#include <dirent.h>
#include <sys/stat.h>

#include "io/sol_reader.h"
#include "util/date_utils.h"
#include "util/string_utils.h"

bool WriteSol(const Solution& solution, const std::string& sol_file_path) {
    struct stat st;
    if (stat(sol_file_path.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
        std::cout << "Sol file path " << sol_file_path << " is invalid" << std::endl;
        return false;
    }

    // 获取父目录路径
    size_t last_slash_pos = sol_file_path.find_last_of("/\\");
    std::string folder = (last_slash_pos != std::string::npos) ? sol_file_path.substr(0, last_slash_pos) : "";

    // 创建目录
    mkdir(folder.c_str(), 0700);

    std::ofstream ofs;
    ofs.open(sol_file_path, std::ios::out);

    if (!ofs.is_open()) {
        std::cout << "Failed to open " << sol_file_path << std::endl;
        ofs.close();
        return false;
    }

    ofs << std::left << std::setw(kTagWidth) << kInstanceNameTag << ": " << solution.problem_.instance_name
        << std::endl;
    ofs << std::left << std::setw(kTagWidth) << kAuthorsTag << ": " << kAuthors << std::endl;
    ofs << std::left << std::setw(kTagWidth) << kDateTag << ": " << GetCurrentDateStr() << std::endl;
    ofs << std::left << std::setw(kTagWidth) << kReferenceTag << ": " << kReference << std::endl;
    ofs << kSolutionTag << std::endl;

    // First task index served by a route (depot markers == 0 are skipped). Empty
    // routes (which appear when the fleet has spare vehicles) sort last instead of
    // dereferencing a non-existent route[1].
    auto first_task = [&solution](int r) -> int {
        const std::vector<idx_t>& route = solution.routes_[r];
        for (idx_t task_index : route) {
            if (task_index != 0) return solution.problem_.new_original_task_index_map[task_index];
        }
        return std::numeric_limits<int>::max();
    };

    std::vector<int> route_written_seq(solution.routes_.size(), 0);
    std::iota(route_written_seq.begin(), route_written_seq.end(), 0);
    std::sort(route_written_seq.begin(), route_written_seq.end(),
              [&first_task](int a, int b) { return first_task(a) < first_task(b); });
    int route_index = 1;

    for (const int route_seq : route_written_seq) {
        const std::vector<idx_t>& route = solution.routes_[route_seq];
        if (first_task(route_seq) == std::numeric_limits<int>::max()) continue;  // skip empty routes
        ofs << kRouteTag << " " << std::to_string(route_index) << " :";
        for (idx_t task_index : route) {
            if (0 == task_index) {
                continue;
            }

            task_index = solution.problem_.new_original_task_index_map[task_index];
            ofs << " " << std::to_string(task_index);
        }
        ofs << std::endl;
        route_index += 1;
    }

    ofs.close();
    return true;
}