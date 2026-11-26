#include <chrono>
// #include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <iostream>
// #include <nlohmann/json.hpp>
#include <string>
#include <dirent.h>

#include "config.h"

#include "domain/eval/ScenarioCapacityEvaluator.h"
#include "domain/solver/memetic/LargeNeighborhoodSearch.h"
#include "domain/solver/memetic/Memetic.h"
#include "domain/solver/memetic/insert/Insert.h"
#include "domain/solver/memetic/removal/Removal.h"
#include "io/pdptw_reader.h"
#include "io/request_pairing_reader.h"
#include "io/sol_reader.h"
#include "io/valid_arcs_reader.h"
#include "model/pdptw.h"
#include "model/solution.h"
#include "util/convert_utils.h"
#include "util/string_utils.h"

using namespace std;

void sisr(VrpProblem& vrpProblem, int vehicleNum) {
    VrpSolution vrpSolution(&vrpProblem);
    buildInitialSolution(vehicleNum, vrpSolution, 0);

    VrpConfig vrpConfig;
    double initialTemperature = 100.0;
    double finalTemperature = 1.0;
    int iterationNum = 100000;
    double coolingRate = std::pow(finalTemperature / initialTemperature, 1.0 / iterationNum);
    VrpSolution currentSol(vrpSolution.vrpProblem);
    currentSol.copySolution(vrpSolution);
    VrpSolution newSol(vrpSolution.vrpProblem);
    double currentTemperature = initialTemperature;

    std::cout << std::left << std::setw(12) << "Iteration" << std::setw(12) << "Distance" << std::setw(20)
              << "TW Violation" << std::setw(20) << "Capacity Violation" << std::setw(20) << "Time" << std::endl;

    std::cout.precision(2);
    std::cout.flags(std::ostream::fixed);

    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point currentTime;
    double duration;
    for (int i = 0; i < iterationNum; ++i) {
        newSol.copySolution(currentSol);

        currentTemperature *= coolingRate;

        stringRemoval(vrpConfig, newSol);
        greedyInsertWithBlinks(vrpConfig, newSol, true);
        insertEmptyRoute(newSol);

        double currentObj = currentSol.getTotalObjectiveValue(vrpConfig);
        double newObj = newSol.getTotalObjectiveValue(vrpConfig);

        if (newObj < currentObj) {
            currentSol.copySolution(newSol);
            if (newObj < vrpSolution.getTotalObjectiveValue(vrpConfig)) {
                vrpSolution.copySolution(newSol);
            }
        } else {
            double randNum = GetRandDouble(0.0, 1.0);
            double delta = newObj - currentObj;
            double prob = std::exp(-delta / currentTemperature);
            if (randNum < prob) {
                currentSol.copySolution(newSol);
            }
        }

        if (0 == i % 1000 || i == iterationNum - 1) {
            currentTime = std::chrono::steady_clock::now();
            duration = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();

            std::cout << std::left << std::setw(12) << i << std::setw(12) << vrpSolution.getTotalDistance()
                      << std::setw(20) << vrpSolution.getTotalTwViolation() << std::setw(20)
                      << vrpSolution.getTotalCapacityViolation() << std::setw(20) << duration << std::endl;
        }
    }
}

void writeFeasibleArcs(const VrpSolution& vrpSolution) {
    std::ofstream ofs;
    ofs.open(vrpSolution.vrpProblem->vrpData.name + "_feasible_arcs.txt", std::ios::out);
    for (int i = 0; i < vrpSolution.vrpProblem->vrpData.nodeNum; ++i) {
        for (int j : vrpSolution.vrpProblem->vrpData.nodeList[i].candiNextNodes) {
            ofs << i << " " << j << std::endl;
        }
    }

    ofs.close();
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printf("Invalid argument number.\n");
        exit(0);
    }
    std::string configFileName = argv[1];
    cout<<"File name = "<<configFileName<<endl;
    std::ifstream ifs(configFileName);
    if (!ifs.is_open()) {
        std::cout << "Failed to open " << configFileName << std::endl;
        ifs.close();
        return 1;
    }
    
    std::string line;
    std::unordered_map<std::string, std::string> configs;
    while (std::getline(ifs, line)) {
        std::istringstream iss(line);
        std::string key, value;

        if (std::getline(iss, key, '=')) {
            std::getline(iss, value);
            configs[key] = value;
        }
    }

    ifs.close();

    
    std::string benchmarkName = configs["benchmark"];
    std::string instanceName = configs["instanceName"];
    int bksRouteNum = std::stoi(configs["bksRouteNum"]);
    int additionalVehicleNum = std::stoi(configs["additionalVehicleNum"]);
    int scenarioNum = configs.count("scenarioNum") ? std::stoi(configs["scenarioNum"]) : kDefaultScenarioNum;
    bool useGpu = configs.count("useGpu") ? std::stoi(configs["useGpu"]) != 0 : false;
    bool useGpuInsertion = configs.count("useGpuInsertion") ? std::stoi(configs["useGpuInsertion"]) != 0 : false;
    double timeLimit = std::stoi(configs["timeLimit"]);
    std::string resultFolder = configs["resultFolder"];
    std::vector<std::string> initialSolFilePaths;
    std::string initialSolFileString = configs["initialSolFiles"];
    std::istringstream iss(initialSolFileString);
    std::string token;
    while (std::getline(iss, token, ';')) {
        initialSolFilePaths.push_back(token);
    }

    std::cout << std::left << std::setw(20) << "Instance Name: " << instanceName << std::endl;
    std::cout << std::left << std::setw(20) << "Time Limit: " << timeLimit << std::endl;
    std::cout << std::left << std::setw(20) << "Result Folder: " << resultFolder << std::endl;
    std::cout << std::left << std::setw(20) << "Initial Sol Files: " << std::endl;
    for (int i = 0; i < initialSolFilePaths.size(); ++i) {
        std::cout << i << ": " << initialSolFilePaths[i] << std::endl;
    }

    std::string instanceFilePath = AMDAHL_DIR;
    instanceFilePath += "/data";
    instanceFilePath += "/instance/";
    instanceFilePath += instanceName + ".txt";
    std::cout << std::left << std::setw(20) << "Scenario Num: " << scenarioNum << std::endl;
    PDPTW problem;
    if (benchmarkName == "LiLim") {
        ReadPDPTW(instanceFilePath, problem, ProblemFormat::kLiLim, scenarioNum);
    } else if (benchmarkName == "SartoriBuriol") {
        ReadPDPTW(instanceFilePath, problem, ProblemFormat::kSartoriBuriol, scenarioNum);
    } else {
        std::cout << "unknown benchmark!" << std::endl;
    }

    instanceName = ToLower(instanceName);

    std::string bks_folder = AMDAHL_DIR;
    bks_folder += "/data";
    bks_folder += "/bks/";

    DIR* dir = opendir(bks_folder.c_str());
    if (dir == nullptr) {
        std::cerr << "Unable to open directory: " << bks_folder << std::endl;
        exit(0);
    }

    std::string bksFilePath;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string file_name = entry->d_name;
        if (file_name.size() < instanceName.size()) {
            continue;
        }

        if (0 != file_name.compare(0, instanceName.size(), instanceName, 0, instanceName.size())) {
            continue;
        }

        if (file_name.size() > instanceName.size()) {
            char tmp_char = file_name[instanceName.size()];
            if ('0' <= tmp_char && '9' >= tmp_char) {
                continue;
            }
        }

        bksFilePath = bks_folder + "/" + file_name;
    }

    Solution solution(problem);
    ReadSol(bksFilePath, solution);

    VrpProblem vrpProblem;
    ConvertVrpProblem(problem, vrpProblem);

    std::unique_ptr<ScenarioCapacityEvaluator> capacityEvaluator;
    if (useGpu || useGpuInsertion) {
        capacityEvaluator = MakeScenarioCapacityEvaluator(
            vrpProblem.vrpData.demandMatrix.data(), vrpProblem.vrpData.nodeNum, vrpProblem.vrpData.numScenarios,
            vrpProblem.vrpData.vehicleCapacity, EvaluatorBackend::kCuda);
        vrpProblem.capacityEvaluator = capacityEvaluator.get();
        vrpProblem.useGpuCapacityEval = useGpu;
        vrpProblem.useGpuInsertionEval = useGpuInsertion;
        std::cout << std::left << std::setw(20) << "Capacity eval: " << capacityEvaluator->Backend()
                  << " (updateRouteData=" << (useGpu ? "gpu" : "cpu")
                  << ", insertion=" << (useGpuInsertion ? "gpu" : "cpu") << ")" << std::endl;
    } else {
        std::cout << std::left << std::setw(20) << "Capacity eval: " << "cpu (inline)" << std::endl;
    }

    VrpSolution bks(&vrpProblem);
    ConvertVrpSolution(solution, bks);

    std::cout.precision(2);
    std::cout.flags(std::ostream::fixed);
    std::cout << "BKS: " << bks.getTotalDistance() << ", " << bks.getTotalTwViolation() << ", "
              << bks.getTotalCapacityViolation() << std::endl;
    std::vector<VrpSolution> initialSols;
    for (int i = 0; i < initialSolFilePaths.size(); ++i) {
        Solution initSol(problem);
        ReadSol(initialSolFilePaths[i], initSol);

        initialSols.emplace_back(&vrpProblem);
        ConvertVrpSolution(initSol, initialSols[i]);
    }

    // sisr(vrpProblem, bks.vehicleNum + additionalVehicleNum);
    VrpConfig vrpConfig;
    vrpConfig.vehicleCost = 0.0;
    vrpConfig.twPenaltyCoeff = 20000.0;
    vrpConfig.capacityPenaltyCoeff = 20000.0;
    VrpSolution memeticSol(&vrpProblem);
    if (bksRouteNum <= 0) {
        bksRouteNum = bks.vehicleNum;
    }
    memetic(vrpConfig, bksRouteNum + additionalVehicleNum, timeLimit, bks, initialSols, vrpProblem, memeticSol,
            resultFolder);
}
