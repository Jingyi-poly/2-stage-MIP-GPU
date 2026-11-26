#include "io/vrp_sol_writer.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <dirent.h>
#include <sys/stat.h>

#include "io/sol_reader.h"
#include "io/sol_writer.h"
#include "util/date_utils.h"

bool IsDirectory(const std::string& path) {
    struct stat path_stat;
    if (stat(path.c_str(), &path_stat) != 0) {
        // 路径不存在或无法访问
        return false;
    }
    return S_ISDIR(path_stat.st_mode);
}

void CreateDirectories(const std::string& path) {
    std::string::size_type pos = 0;
    std::string current_path = "";
    while ((pos = path.find_first_of("/\\", pos)) != std::string::npos) {
        current_path += path.substr(0, pos + 1);
        // 注意：这里使用0777作为权限设置，可以根据需要调整
        mkdir(current_path.c_str(), 0777);
        pos++;
    }
    // 创建最后的目录
    mkdir(path.c_str(), 0740);
}

bool WriteSol(const VrpSolution& solution, const std::string& sol_file_path) {
    if (IsDirectory(sol_file_path)) {
        std::cout << "Sol file path " << sol_file_path << " is invalid" << std::endl;
        return false;
    }

    std::string folder = sol_file_path.substr(0, sol_file_path.find_last_of("/\\"));
    CreateDirectories(folder);

    std::ofstream ofs;
    ofs.open(sol_file_path, std::ios::out);

    if (!ofs.is_open()) {
        std::cout << "Failed to open " << sol_file_path << std::endl;
        ofs.close();
        return false;
    }

    ofs << std::left << std::setw(kTagWidth) << kInstanceNameTag << ": " << solution.vrpProblem->vrpData.name
        << std::endl;
    ofs << std::left << std::setw(kTagWidth) << kAuthorsTag << ": " << kAuthors << std::endl;
    ofs << std::left << std::setw(kTagWidth) << kDateTag << ": " << GetCurrentDateStr() << std::endl;
    ofs << std::left << std::setw(kTagWidth) << kReferenceTag << ": " << kReference << std::endl;
    ofs << kSolutionTag << std::endl;

    // Empty routes (spare vehicles) have a null startNode; sort them last and skip
    // them when writing instead of dereferencing a null pointer.
    auto first_node_id = [&solution](int r) {
        const RouteNode* s = solution.routes[r].startNode;
        return s != nullptr ? s->nodeId : std::numeric_limits<int>::max();
    };
    std::vector<int> route_written_seq(solution.routes.size(), 0);
    std::iota(route_written_seq.begin(), route_written_seq.end(), 0);
    std::sort(route_written_seq.begin(), route_written_seq.end(),
              [&first_node_id](int a, int b) { return first_node_id(a) < first_node_id(b); });

    int route_index = 1;
    for (int route_seq : route_written_seq) {
        const VrpRoute& route = solution.routes[route_seq];
        if (route.startNode == nullptr) continue;  // skip empty routes
        ofs << kRouteTag << " " << std::to_string(route_index) << " :";

        RouteNode* currentNode = route.startNode;
        while (nullptr != currentNode) {
            ofs << " " << std::to_string(currentNode->nodeId);
            currentNode = currentNode->nexNode;
        }
        ofs << std::endl;
        route_index += 1;
    }

    ofs.close();
    return true;
}