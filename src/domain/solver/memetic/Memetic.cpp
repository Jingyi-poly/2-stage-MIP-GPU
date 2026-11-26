#include "domain/solver/memetic/Memetic.h"

#include <omp.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
// #include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "domain/solver/memetic/LargeNeighborhoodSearch.h"
#include "domain/solver/memetic/LocalSearch.h"
#include "domain/solver/memetic/crossover/CrossoverStrategy.h"
#include "domain/solver/memetic/crossover/EdgeAssemblyCrossover.h"
#include "domain/solver/memetic/crossover/SrexCrossover.h"
#include "domain/solver/memetic/insert/Insert.h"
#include "domain/solver/memetic/removal/Removal.h"
#include "io/vrp_sol_writer.h"
#include "util/random_utils.h"
#include "util/sol_similarity.h"

// #include "aclnn/aclnn.h"

using namespace std;


bool gMemeticOptimizeCapacity = false;
bool gMemeticHardTw = false;
bool gMemeticDisableSrex = false;

std::vector<double> tmp1;
std::vector<double> tmp2;
bool cmp1(int p1, int p2){ return tmp1[p1] < tmp1[p2];}
bool cmp2(int p1, int p2){ return tmp2[p1] < tmp2[p2];}


void buildInitialSolution(int routeNum, VrpSolution& vrpSolution, int type) {
    vrpSolution.routes.clear();
    vrpSolution.vehicleNum = routeNum;

    int avgRouteNodeNum = (int)vrpSolution.vrpProblem->vrpData.pickNodeIds.size() / vrpSolution.vehicleNum;

    std::vector<int> pickupNodeIds = vrpSolution.vrpProblem->vrpData.pickNodeIds;
    if(type == 1) 
    {
        tmp1.resize(vrpSolution.vrpProblem->vrpData.nodeNum);
        for(int i=1; i<vrpSolution.vrpProblem->vrpData.nodeNum; i++) 
        {
            RouteNode* currentNode = &vrpSolution.routeNodes[i];
            RouteNode* currentCorresRouteNode = currentNode->corresRouteNode;
            int k = currentCorresRouteNode->nodeId;
            double t1 = vrpSolution.vrpProblem->vrpData.nodeList[i].timeWindow.second;
            double t2 = vrpSolution.vrpProblem->vrpData.nodeList[k].timeWindow.second;
            tmp1[i] = max(t1, t2);
        }
        sort(pickupNodeIds.begin(), pickupNodeIds.end(), cmp1);
    }
    else if(type == 2)
    {
        tmp2.resize(vrpSolution.vrpProblem->vrpData.nodeNum);
        for(int i=1; i<vrpSolution.vrpProblem->vrpData.nodeNum; i++) 
        {
            RouteNode* currentNode = &vrpSolution.routeNodes[i];
            RouteNode* currentCorresRouteNode = currentNode->corresRouteNode;
            int k = currentCorresRouteNode->nodeId;
            double t1 = vrpSolution.vrpProblem->vrpData.nodeList[i].timeWindow.first;
            double t2 = vrpSolution.vrpProblem->vrpData.nodeList[k].timeWindow.first;
            tmp2[i] = max(t1, t2);
        }
        sort(pickupNodeIds.begin(), pickupNodeIds.end(), cmp2);
    }
    else std::shuffle(pickupNodeIds.begin(), pickupNodeIds.end(), GetRandomGenerator());

    for (int i = 0; i < vrpSolution.vehicleNum; ++i) {
        vrpSolution.routes.emplace_back(i);
        RouteNode* prevNode = nullptr;
        int nodeNumInRoute = avgRouteNodeNum;
        if (i == vrpSolution.vehicleNum - 1) {
            nodeNumInRoute = (int)pickupNodeIds.size() - i * avgRouteNodeNum;
        }
        for (int j = i * avgRouteNodeNum; j < i * avgRouteNodeNum + nodeNumInRoute; ++j) {
            int pickupNodeId = pickupNodeIds[j];

            RouteNode* currentNode = &vrpSolution.routeNodes[pickupNodeId];
            RouteNode* currentCorresRouteNode = currentNode->corresRouteNode;

            vrpSolution.insertNode(i, pickupNodeId, currentCorresRouteNode->nodeId, prevNode, prevNode, false);

            prevNode = currentCorresRouteNode;
        }
        vrpSolution.updateRouteData(i, vrpSolution.routes[i].startNode, vrpSolution.routes[i].endNode);
    }
}

void writeBestSol(VrpSolution& bestSolution, const std::string& resultFolder) {
    std::string resultFolderPath = resultFolder;
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << bestSolution.getTotalDistance() << "_"
           << bestSolution.getTotalTwViolation() << "_" << bestSolution.getTotalCapacityViolation();
    std::string solQualityStr = stream.str();
    std::string solFileName = bestSolution.vrpProblem->vrpData.name + "." + std::to_string(bestSolution.routes.size()) +
                              "_" + solQualityStr + ".txt";

    resultFolderPath += '/' + solFileName;

    std::string solFilePath = resultFolderPath;
    WriteSol(bestSolution, solFilePath);
}

// vector<double> tmp_value;
// bool cmp_solIndices(int x,int y){ return tmp_value[x]>tmp_value[y];}
void memetic(VrpConfig& inputConfig, int routeNum, double timeLimit, const VrpSolution& bks,
             const std::vector<VrpSolution>& initialSols, VrpProblem& vrpProblem, VrpSolution& bestSolution,
             const std::string& resultFolder, int numThreads) {
    // Penalties adapt within a run. Keep the caller's semantic coefficients
    // unchanged so reusing a config cannot apply scenario normalization twice.
    VrpConfig vrpConfig = inputConfig;

    // Capacity state/deltas are deliberately kept as raw sums so the CPU and
    // CUDA evaluators remain bit-for-bit comparable. Scale the coefficient once
    // here to optimize the scenario mean, and scale the adaptive bounds by the
    // same factor so changing N does not silently change the effective penalty.
    double capacityPenaltyLower = kPenaltyCoeffLower;
    double capacityPenaltyUpper = kPenaltyCoeffUpper;
    if (vrpConfig.useMeanScenarioCapacityViolation) {
        const int scenarioCount = std::max(1, vrpProblem.vrpData.numScenarios);
        const double inverseScenarioCount = 1.0 / static_cast<double>(scenarioCount);
        vrpConfig.capacityPenaltyCoeff *= inverseScenarioCount;
        capacityPenaltyLower *= inverseScenarioCount;
        capacityPenaltyUpper *= inverseScenarioCount;
    }

    std::cout << std::left << std::setw(12) << "Generation" << std::setw(12) << "Distance" << std::setw(20)
              << "TW Violation" << std::setw(20) << "Capacity Violation" << std::setw(20) << "Time" << std::endl;
    omp_set_num_threads(numThreads);
    
    // Aclnn aclnn = Aclnn();
    // aclnn.aclnn_test();

    std::cout.precision(2);
    std::cout.flags(std::ostream::fixed);

    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point currentTime;

    std::vector<VrpSolution> population;
    population.reserve(kPopulationSize);
    for (int i = 0; i < kPopulationSize; ++i) {
        population.emplace_back(&vrpProblem);
    }

    // bool flag1 = true, flag2 = true;
    // for (int i = 0; i < kPopulationSize; ++i) {
    //     if (i < initialSols.size()) {
    //         population[i].copySolution(initialSols[i]);
    //     } else {
    //         if(!flag1) buildInitialSolution(routeNum, population[i], 1), flag1 = true;
    //         else if(!flag2) buildInitialSolution(routeNum, population[i], 2), flag2 = true;
    //         else buildInitialSolution(routeNum, population[i], 0);
    //     }
    // }
    for (int i = 0; i < kPopulationSize; ++i) {
        if (i < initialSols.size()) {
            population[i].copySolution(initialSols[i]);
        } else {
            buildInitialSolution(routeNum, population[i], 0);
        }
    }
    const char* traceInitializationEnv = std::getenv("PAPER_TRACE_INITIALIZATION");
    const bool traceInitialization =
        numThreads == 1 && traceInitializationEnv != nullptr && std::atoi(traceInitializationEnv) != 0;
    if (traceInitialization) {
        // A small exact-comparison instance can reach its optimum during population
        // initialisation.  In a single-thread trace, expose those genuine
        // best-so-far improvements instead of hiding the entire phase in the
        // generation-0 aggregate.  Keeping this opt-in and single-threaded avoids
        // changing the parallel search path or introducing logging races.
        bestSolution.copySolution(population[0]);
        for (int i = 1; i < kPopulationSize; ++i) {
            double initElapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
                                     std::chrono::steady_clock::now() - startTime)
                                     .count();
            if (initElapsed > timeLimit) break;
            localSearch(vrpConfig, population[i]);
            if (population[i].getTotalObjectiveValue(vrpConfig) < bestSolution.getTotalObjectiveValue(vrpConfig)) {
                bestSolution.copySolution(population[i]);
            }
            initElapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
                              std::chrono::steady_clock::now() - startTime)
                              .count();
            std::cout << std::left << std::setw(12) << 0 << std::setw(12) << bestSolution.getTotalDistance()
                      << std::setw(20) << bestSolution.getTotalTwViolation() << std::setw(20)
                      << bestSolution.getTotalCapacityViolation() << std::setw(20) << initElapsed << std::endl;
        }
    } else {
#pragma omp parallel for shared(vrpConfig, population, startTime, timeLimit) default(none)
        for (int i = 1; i < kPopulationSize; ++i) {
            // Respect the wall-clock budget during population initialisation too, so that
            // fixed-time comparisons are not dominated by an unbounded init phase.
            double initElapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
                                     std::chrono::steady_clock::now() - startTime)
                                     .count();
            if (initElapsed > timeLimit) continue;
            localSearch(vrpConfig, population[i]);
        }
    }

    for (int i = 0; i < kPopulationSize; ++i) {
        if (0 == i) {
            bestSolution.copySolution(population[i]);
        } else {
            if (bestSolution.getTotalObjectiveValue(vrpConfig) > population[i].getTotalObjectiveValue(vrpConfig)) {
                bestSolution.copySolution(population[i]);
            }
        }

        if (population[i].getTotalTwViolation() <= 0.0 && population[i].getTotalCapacityViolation() <= 0.0) {
            writeBestSol(population[i], resultFolder);
        }
    }

    writeBestSol(bestSolution, resultFolder);

    currentTime = std::chrono::steady_clock::now();
    double duration = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();

    std::cout << std::left << std::setw(12) << 0 << std::setw(12) << bestSolution.getTotalDistance() << std::setw(20)
              << bestSolution.getTotalTwViolation() << std::setw(20) << bestSolution.getTotalCapacityViolation()
              << std::setw(20) << duration << std::endl;

    CrossoverStrategy crossoverStrategy = CrossoverStrategy::kEax;

    std::vector<int> solIndices(kPopulationSize, 0);
    std::iota(solIndices.begin(), solIndices.end(), 0);

    std::vector<VrpSolution> children;
    children.reserve(kPopulationSize);
    for (int i = 0; i < kPopulationSize; ++i) {
        children.emplace_back(&vrpProblem);
    }

    if (!gMemeticOptimizeCapacity) {
        vrpConfig.twPenaltyCoeff = 1.0;
        vrpConfig.capacityPenaltyCoeff = 0.0;
    }
    int nonImprovement = 0;
    double lastBestObj = 0.0;
    for (int i = 1; i < kMaxGeneration; ++i) {
        // tmp_value.clear();
        // for(int j = 0; j < kPopulationSize; j++)
        //     tmp_value.push_back(population[j].getTotalObjectiveValue(vrpConfig));
        // sort(tmp_value.begin(),tmp_value.end());
        // double value_demand = tmp_value[(int)kPopulationSize*0.95];
        // for(int j = 0; j < kPopulationSize; j++)
        // {
        //     if(tmp_value[j] < value_demand)
        //     {
        //         buildInitialSolution(routeNum, population[j], 0);
        //         localSearch(vrpConfig, population[j]);
        //         tmp_value[j] = population[j].getTotalObjectiveValue(vrpConfig);
        //     }
        // }
        std::shuffle(solIndices.begin(), solIndices.end(), GetRandomGenerator());
        // std::sort(solIndices.begin(), solIndices.end(),cmp_solIndices);
#pragma omp parallel for shared(vrpConfig, population, children, solIndices, startTime, timeLimit, \
                                    gMemeticDisableSrex) default(none)
        for (int j = 0; j < kPopulationSize; ++j) {
            int firstSolIndex = solIndices[j];
            int secondSolIndex = solIndices[(j + 1) % kPopulationSize];

            double tmpRandValue = GetRandDouble(0.0, 1.0);
            if (!gMemeticDisableSrex && tmpRandValue < kSrexProb) {
                srexCrossover(vrpConfig, population[firstSolIndex], population[secondSolIndex], startTime, timeLimit,
                              children[j]);
            } else {
                eax(vrpConfig, population[firstSolIndex], population[secondSolIndex], children[j]);
            }

            bool emptyVehicle = std::any_of(children[j].routes.begin(), children[j].routes.end(),
                                            [](const VrpRoute& route) { return route.nodeNum == 0; });
            if (emptyVehicle &&
                (children[j].getTotalTwViolation() > 0.0 || children[j].getTotalCapacityViolation() > 0.0)) {
                children[j].copySolution(population[firstSolIndex]);
            }
        }

        for (int j = 0; j < kPopulationSize; ++j) {
            if (children[j].getTotalObjectiveValue(vrpConfig) <
                population[solIndices[j]].getTotalObjectiveValue(vrpConfig)) {
                population[solIndices[j]].copySolution(children[j]);
                ++nonImprovement;

                if (children[j].getTotalObjectiveValue(vrpConfig) < bestSolution.getTotalObjectiveValue(vrpConfig)) {
                    lastBestObj = bestSolution.getTotalObjectiveValue(vrpConfig);
                    bestSolution.copySolution(children[j]);
                    writeBestSol(bestSolution, resultFolder);
                    nonImprovement = 0;
                }
            }

            if (children[j].getTotalTwViolation() <= 0.0 && children[j].getTotalCapacityViolation() <= 0.0) {
                writeBestSol(children[j], resultFolder);
            }
        }

        if (nonImprovement >= kMaxNonImprovement) {
            double totalDiversity = 0.0;
            std::vector<std::pair<int, double>> fitnessRank;
            std::vector<std::pair<int, double>> totalObjRank;
            std::vector<std::pair<int, double>> diversityRank;

            for (int j = 0; j < kPopulationSize; ++j) {
                int solIdx1 = solIndices[j];
                double curDiversity = 0.0;
                totalObjRank.emplace_back(std::make_pair(solIdx1, population[solIdx1].getTotalObjectiveValue(vrpConfig)));
                for (int k = 0; k < kPopulationSize; ++k) {
                    int solIdx2 = solIndices[k];
                    if (solIdx1 == solIdx2) {
                        continue;
                    }

                    curDiversity += calculateSimilarity(population[solIdx1], population[solIdx2]);
                }
                totalDiversity += curDiversity;
                diversityRank.emplace_back(std::make_pair(solIdx1, curDiversity));
            }

            for (int j = 0; j < kPopulationSize; ++j) {
                int solIdx = solIndices[j];
                fitnessRank.emplace_back(std::make_pair(solIdx, totalObjRank[j].second + 0.5 * diversityRank[j].second / totalDiversity));
            }

            std::sort(fitnessRank.begin(), fitnessRank.end(), [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                return a.second > b.second;
            });

            int replacePopulationSize = int(kPopulationReplaceRatio * kPopulationSize);
            for (int j = 0; j < replacePopulationSize; ++j) {
                int solIdx = fitnessRank[j].first;
                buildInitialSolution(routeNum, population[solIdx], 0);
            }

            nonImprovement = 0;
        }

        currentTime = std::chrono::steady_clock::now();
        duration = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
        std::cout << std::left << std::setw(12) << i << std::setw(12) << bestSolution.getTotalDistance()
                  << std::setw(20) << bestSolution.getTotalTwViolation() << std::setw(20)
                  << bestSolution.getTotalCapacityViolation() << std::setw(20) << duration << std::endl;

        if (gMemeticOptimizeCapacity) {
            if (bestSolution.getTotalCapacityViolation() <= 0.0) {
                vrpConfig.capacityPenaltyCoeff *= 0.9;
                vrpConfig.capacityPenaltyCoeff = std::max(vrpConfig.capacityPenaltyCoeff, capacityPenaltyLower);
            } else {
                vrpConfig.capacityPenaltyCoeff *= 1.1;
                vrpConfig.capacityPenaltyCoeff = std::min(vrpConfig.capacityPenaltyCoeff, capacityPenaltyUpper);
            }
        }

        if (!gMemeticHardTw) {
            if (bestSolution.getTotalTwViolation() <= 0.0) {
                vrpConfig.twPenaltyCoeff *= 0.9;
                vrpConfig.twPenaltyCoeff = std::max(vrpConfig.twPenaltyCoeff, kPenaltyCoeffLower);
            } else {
                vrpConfig.twPenaltyCoeff *= 1.1;
                vrpConfig.twPenaltyCoeff = std::min(vrpConfig.twPenaltyCoeff, kPenaltyCoeffUpper);
            }
        }

        if (duration > timeLimit) {
            break;
        }
    }
}
