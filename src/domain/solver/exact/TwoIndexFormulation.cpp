#include "domain/solver/exact/TwoIndexFormulation.h"

#include <gurobi_c++.h>

#include <cassert>
#include <chrono>
#include <cstdint>
#include <ctime>
// #include <filesystem>
#include <iostream>
#include <unordered_map>

#include "domain/solver/memetic/removal/Removal.h"
#include "io/vrp_sol_writer.h"

class CallBack : public GRBCallback {
   public:
    int totalNodeNumInGraph_{-1};
    int totalPickupNodeNumInGraph_{-1};
    const std::unordered_map<int, int>& newOriginalNodeIndexMap_;
    const std::vector<std::vector<GRBVar>>& arcVariables_;
    VrpSolution& newSol_;
    const std::string& resultFolder_;
    CallBack(int totalNodeNumInGraph, int totalPickupNodeNumInGraph,
             const std::unordered_map<int, int>& newOriginalNodeIndexMap,
             const std::vector<std::vector<GRBVar>>& arcVariables, VrpSolution& newSol,
             const std::string& resultFolder)
        : totalNodeNumInGraph_(totalNodeNumInGraph),
          totalPickupNodeNumInGraph_(totalPickupNodeNumInGraph),
          newOriginalNodeIndexMap_(newOriginalNodeIndexMap),
          arcVariables_(arcVariables),
          newSol_(newSol),
          resultFolder_(resultFolder) {}

   protected:
    void callback() {
        try {
            if (where == GRB_CB_MIPSOL) {
                std::vector<int> routeStartNodeIds;
                std::vector<int> nodeNextNodeIds;
                nodeNextNodeIds.resize(totalNodeNumInGraph_, -1);
                double* x = getSolution(&arcVariables_[0][0], totalNodeNumInGraph_);

                for (int i = 1; i <= totalPickupNodeNumInGraph_; ++i) {
                    double arcVarValue = x[i];
                    if (arcVarValue > 0.1) {
                        routeStartNodeIds.push_back(i);
                    }
                }
                delete[] x;
                x = nullptr;

                for (int i = 1; i < totalNodeNumInGraph_ - 1; ++i) {
                    x = getSolution(&arcVariables_[i][0], totalNodeNumInGraph_);
                    for (int j = 1; j < totalNodeNumInGraph_; ++j) {
                        double arcVarValue = x[j];
                        if (arcVarValue > 0.1) {
                            nodeNextNodeIds[i] = j;
                        }
                    }
                    delete[] x;
                    x = nullptr;
                }
                newSol_.totalValueValid = false;
                int routeIndex = 0;
                for (int startNodeId : routeStartNodeIds) {
                    newSol_.routes[routeIndex].clearData();

                    int originalStartNodeId = newOriginalNodeIndexMap_.find(startNodeId)->second;
                    newSol_.routes[routeIndex].startNode = &newSol_.routeNodes[originalStartNodeId];
                    newSol_.routes[routeIndex].startNode->preNode = nullptr;

                    RouteNode* preNode = newSol_.routes[routeIndex].startNode;
                    int routeNodeNum = 1;

                    int currentNodeId = nodeNextNodeIds[startNodeId];
                    while (totalNodeNumInGraph_ - 1 != currentNodeId) {
                        int originalCurrentNodeId = newOriginalNodeIndexMap_.find(currentNodeId)->second;
                        RouteNode* currentNode = &newSol_.routeNodes[originalCurrentNodeId];

                        preNode->nexNode = currentNode;
                        currentNode->preNode = preNode;

                        preNode = currentNode;

                        currentNodeId = nodeNextNodeIds[currentNodeId];
                        routeNodeNum += 1;
                    }

                    preNode->nexNode = nullptr;
                    newSol_.routes[routeIndex].endNode = preNode;
                    newSol_.routes[routeIndex].nodeNum = routeNodeNum;
                    newSol_.updateRouteData(routeIndex, newSol_.routes[routeIndex].startNode,
                                           newSol_.routes[routeIndex].endNode);

                    routeIndex += 1;
                }

                std::string resultFolderPath = resultFolder_;
                std::stringstream stream;
                stream << std::fixed << std::setprecision(2) << newSol_.getTotalDistance() << "_"
                       << newSol_.getTotalTwViolation() << "_" << newSol_.getTotalCapacityViolation();
                std::string solQualityStr = stream.str();
                std::string solFileName = newSol_.vrpProblem->vrpData.name + "." + std::to_string(newSol_.routes.size()) +
                                          "_" + solQualityStr + ".txt";

                resultFolderPath += '/' + solFileName;

                std::string solFilePath = resultFolderPath;
                WriteSol(newSol_, solFilePath);
            }
        } catch (GRBException e) {
            std::cout << "Error number: " << e.getErrorCode() << std::endl;
            std::cout << e.getMessage() << std::endl;
        } catch (...) {
            std::cout << "Error during callback" << std::endl;
        }
    }
};

void twoIndexFormulationSolve(const VrpConfig& vrpConfig, const VrpProblem* vrpProblem, const VrpSolution* initSolution,
                              bool freeNodes, int freeNodesNum, bool freeRoutes,
                              const std::vector<int>& freeRouteIndices, bool fixRouteNum,
                              const std::unordered_map<int, std::unordered_set<int>>& validArcsData,
                              const std::unordered_map<int, std::unordered_set<int>>& requestPairingData,
                              double timeLimit, bool pricingSubProblem,
                              const PricingSubProblemData& pricingSubProblemData,
                              PricingSubProblemSolution& pricingSubProblemSolution, VrpSolution& vrpSolution,
                              const std::string& resultFolder) {
    if (nullptr != initSolution) {
        assert(initSolution->unassignedNodeIds.empty());
        assert(initSolution->totalCapacityViolation <= 0.0);
        assert(initSolution->totalTwViolation <= 0.0);
        vrpSolution.copySolution(*initSolution);
    }

    std::unordered_map<int, int> originalNewNodeIndexMap;
    std::unordered_map<int, int> newOriginalNodeIndexMap;
    int pickupNodeCounter = 0;
    int pickupNodeNum = (int)vrpProblem->vrpData.pickNodeIds.size();
    for (int i = 1; i < vrpProblem->vrpData.nodeNum; ++i) {
        if (Utils::VrpNodeType::kPickup == vrpProblem->vrpData.nodeList[i].nodeType) {
            pickupNodeCounter += 1;
            originalNewNodeIndexMap[i] = pickupNodeCounter;
            newOriginalNodeIndexMap[pickupNodeCounter] = i;

            int tmpDeliveryNodeIndex = vrpProblem->vrpData.nodeList[i].corresNodeId;
            originalNewNodeIndexMap[tmpDeliveryNodeIndex] = pickupNodeCounter + pickupNodeNum;
            newOriginalNodeIndexMap[pickupNodeCounter + pickupNodeNum] = tmpDeliveryNodeIndex;

            if (pricingSubProblem) {
                if (pricingSubProblemData.dualValueForPickupNodes.find(i) ==
                    pricingSubProblemData.dualValueForPickupNodes.end()) {
                    std::cerr << "Dual value for pickup node " << i << " not found" << std::endl;
                    exit(0);
                }
            }
        }
    }

    int totalPickupNodeNumInGraph = (int)vrpProblem->vrpData.pickNodeIds.size();
    int totalNodeNumInGraph = vrpProblem->vrpData.nodeNum + 1;

    std::vector<std::vector<GRBVar>> arcVariables;
    arcVariables.resize(totalNodeNumInGraph);
    for (int i = 0; i < totalNodeNumInGraph; ++i) {
        arcVariables[i].resize(totalNodeNumInGraph);
    }
    std::vector<GRBVar> startServiceTimeVariables;
    startServiceTimeVariables.resize(totalNodeNumInGraph);
    std::vector<GRBVar> vehicleLoadVariables;
    vehicleLoadVariables.resize(totalNodeNumInGraph);
    std::vector<GRBVar> routeIdentifierVariables;
    routeIdentifierVariables.resize(totalNodeNumInGraph);

    GRBVar* vars = 0;

    try {
        GRBEnv env = GRBEnv(true);
        std::string logFileName = "two_index_formulation";
        if (pricingSubProblem) {
            logFileName.append("_pricing_sub_problem");
        } else {
            logFileName.append("_original");
        }
        logFileName.append(".log");
        env.set("LogFile", logFileName);
        env.set("MIPGap", "0.0");
        env.set("TimeLimit", std::to_string(timeLimit));
        env.start();

        GRBModel model = GRBModel(env);

        for (int i = 0; i < totalNodeNumInGraph; ++i) {
            for (int j = 0; j < totalNodeNumInGraph; ++j) {
                double tmpInfeasibleArc = false;
                if (i == j) {
                    tmpInfeasibleArc = true;
                }
                if (!tmpInfeasibleArc && 0 == j) {
                    tmpInfeasibleArc = true;
                }
                if (!tmpInfeasibleArc && totalNodeNumInGraph - 1 == i) {
                    tmpInfeasibleArc = true;
                }
                if (!tmpInfeasibleArc && 1 <= i && i <= totalPickupNodeNumInGraph && totalNodeNumInGraph - 1 == j) {
                    tmpInfeasibleArc = true;
                }
                if (!tmpInfeasibleArc && i < totalNodeNumInGraph - 1 && i > totalPickupNodeNumInGraph &&
                    i - j == totalPickupNodeNumInGraph) {
                    tmpInfeasibleArc = true;
                }

                int tmpFirstOriginalNodeIndex = 0;
                if (1 <= i && i < totalNodeNumInGraph - 1) {
                    tmpFirstOriginalNodeIndex = newOriginalNodeIndexMap[i];
                }
                int tmpSecondOriginalNodeIndex = 0;
                if (1 <= j && j < totalNodeNumInGraph - 1) {
                    tmpSecondOriginalNodeIndex = newOriginalNodeIndexMap[j];
                }

                if (!tmpInfeasibleArc) {
                    if (vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].candiNextNodes.find(
                            tmpSecondOriginalNodeIndex) ==
                        vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].candiNextNodes.end()) {
                        tmpInfeasibleArc = true;
                    }
                }

                if (!tmpInfeasibleArc && !validArcsData.empty()) {
                    if (validArcsData.find(tmpFirstOriginalNodeIndex)->second.find(tmpSecondOriginalNodeIndex) ==
                        validArcsData.find(tmpFirstOriginalNodeIndex)->second.end()) {
                        tmpInfeasibleArc = true;
                    }
                }

                double varUpperBound = 1.0;
                if (tmpInfeasibleArc) {
                    varUpperBound = 0.0;
                }

                double objCoeff = vrpProblem->vrpData.disMatrix[tmpFirstOriginalNodeIndex][tmpSecondOriginalNodeIndex];
                if (pricingSubProblem) {
                    if (Utils::VrpNodeType::kPickup ==
                        vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].nodeType) {
                        if (pricingSubProblemData.dualValueForPickupNodes.find(tmpFirstOriginalNodeIndex) !=
                            pricingSubProblemData.dualValueForPickupNodes.end()) {
                            objCoeff -=
                                pricingSubProblemData.dualValueForPickupNodes.find(tmpFirstOriginalNodeIndex)->second;
                        } else {
                            std::cerr << "Dual value for pickup node " << i << " not found" << std::endl;
                            exit(0);
                        }
                    }
                } else if (0 == i) {
                    if (nullptr == initSolution || !fixRouteNum) {
                        objCoeff += 10000.0;
                    }
                }

                std::string varName = "x_" + std::to_string(i) + "_" + std::to_string(j);

                GRBVar newVar = model.addVar(0.0, varUpperBound, objCoeff, GRB_BINARY, varName);

                arcVariables[i][j] = newVar;
            }
        }

        for (int i = 0; i < totalNodeNumInGraph; ++i) {
            std::string varName = "b_" + std::to_string(i);

            int tmpFirstOriginalNodeIndex = 0;
            if (1 <= i && i < totalNodeNumInGraph - 1) {
                tmpFirstOriginalNodeIndex = newOriginalNodeIndexMap[i];
            }
            double varLower = vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].timeWindow.first;
            double varUpper = vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].timeWindow.second;

            if (0 == i) {
                varUpper = varLower;
            }
            GRBVar newVar = model.addVar(varLower, varUpper, 0.0, GRB_CONTINUOUS, varName);
            startServiceTimeVariables[i] = newVar;
        }

        for (int i = 0; i < totalNodeNumInGraph; ++i) {
            std::string varName = "q_" + std::to_string(i);

            int tmpFirstOriginalNodeIndex = 0;
            if (1 <= i && i < totalNodeNumInGraph - 1) {
                tmpFirstOriginalNodeIndex = newOriginalNodeIndexMap[i];
            }

            double varLower = std::max(0.0, vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].demand);
            double varUpper = std::min(
                vrpProblem->vrpData.vehicleCapacity,
                vrpProblem->vrpData.vehicleCapacity + vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].demand);

            GRBVar newVar = model.addVar(varLower, varUpper, 0.0, GRB_CONTINUOUS, varName);
            vehicleLoadVariables[i] = newVar;
        }

        for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
            double varLower = 1.0;
            double varUpper = totalPickupNodeNumInGraph;
            std::string varName = "v_" + std::to_string(i);

            GRBVar newVar = model.addVar(varLower, varUpper, 0.0, GRB_CONTINUOUS, varName);
            routeIdentifierVariables[i] = newVar;
        }

        if (pricingSubProblem) {
            for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
                GRBLinExpr linExpr;
                for (int j = 0; j < totalNodeNumInGraph - 1; ++j) {
                    linExpr += arcVariables[j][i];
                }
                for (int j = 1; j < totalNodeNumInGraph; ++j) {
                    linExpr -= arcVariables[i][j];
                }
                std::string constrName = "flow_balance_" + std::to_string(i);
                model.addConstr(linExpr, GRB_EQUAL, 0.0, constrName);
            }
        } else {
            for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
                GRBLinExpr linExpr;
                for (int j = 0; j < totalNodeNumInGraph - 1; ++j) {
                    linExpr += arcVariables[j][i];
                }
                std::string constrName = "in_degree_" + std::to_string(i);
                model.addConstr(linExpr, GRB_EQUAL, 1.0, constrName);
            }

            for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
                GRBLinExpr linExpr;
                for (int j = 1; j < totalNodeNumInGraph; ++j) {
                    linExpr += arcVariables[i][j];
                }
                std::string constrName = "out_degree_" + std::to_string(i);
                model.addConstr(linExpr, GRB_EQUAL, 1.0, constrName);
            }
        }

        for (int i = 0; i < totalNodeNumInGraph - 1; ++i) {
            GRBVar firstStartServiceVar = startServiceTimeVariables[i];
            int tmpFirstOriginalNodeIndex = 0;
            if (1 <= i && i < totalNodeNumInGraph - 1) {
                tmpFirstOriginalNodeIndex = newOriginalNodeIndexMap[i];
            }
            const VrpNode& tmpFirstNode = vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex];

            for (int j = 1; j < totalNodeNumInGraph; ++j) {
                if (i == j) {
                    continue;
                }

                GRBVar secondStartServiceVar = startServiceTimeVariables[j];

                int tmpSecondOriginalNodeIndex = 0;
                if (1 <= j && j < totalNodeNumInGraph - 1) {
                    tmpSecondOriginalNodeIndex = newOriginalNodeIndexMap[j];
                }

                const VrpNode& tmpSecondNode = vrpProblem->vrpData.nodeList[tmpSecondOriginalNodeIndex];

                double tmpTravelTime =
                    vrpProblem->vrpData.disMatrix[tmpFirstOriginalNodeIndex][tmpSecondOriginalNodeIndex] +
                    tmpFirstNode.serveTime;
                double tmpBigM = tmpFirstNode.timeWindow.second - tmpSecondNode.timeWindow.first + tmpTravelTime;

                GRBLinExpr linExpr;
                linExpr += firstStartServiceVar;
                linExpr -= secondStartServiceVar;
                linExpr += tmpBigM * arcVariables[i][j];
                std::string constrName = "start_service_" + std::to_string(i) + "_" + std::to_string(j);
                model.addConstr(linExpr, GRB_LESS_EQUAL, tmpBigM - tmpTravelTime, constrName);
            }
        }

        for (int i = 0; i < totalNodeNumInGraph - 1; ++i) {
            GRBVar firstVehicleLoadVar = vehicleLoadVariables[i];
            int tmpFirstOriginalNodeIndex = 0;
            if (1 <= i && i < totalNodeNumInGraph - 1) {
                tmpFirstOriginalNodeIndex = newOriginalNodeIndexMap[i];
            }
            const VrpNode& tmpFirstNode = vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex];

            for (int j = 0; j < totalNodeNumInGraph; ++j) {
                if (i == j) {
                    continue;
                }

                GRBVar secondVehicleLoadVar = vehicleLoadVariables[j];

                int tmpSecondOriginalNodeIndex = 0;
                if (1 <= j && j < totalNodeNumInGraph - 1) {
                    tmpSecondOriginalNodeIndex = newOriginalNodeIndexMap[j];
                }

                const VrpNode& tmpSecondNode = vrpProblem->vrpData.nodeList[tmpSecondOriginalNodeIndex];

                double tmpDemand = tmpSecondNode.demand;
                double tmpBigM = vrpProblem->vrpData.vehicleCapacity + tmpSecondNode.demand;

                GRBLinExpr linExpr;
                linExpr += firstVehicleLoadVar;
                linExpr -= secondVehicleLoadVar;
                linExpr += tmpBigM * arcVariables[i][j];
                std::string constrName = "vehicle_load_" + std::to_string(i) + "_" + std::to_string(j);
                model.addConstr(linExpr, GRB_LESS_EQUAL, tmpBigM - tmpDemand, constrName);
            }
        }

        for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
            GRBVar pickupStartServiceTime = startServiceTimeVariables[i];
            GRBVar deliveryStartServiceTime = startServiceTimeVariables[i + totalPickupNodeNumInGraph];

            int tmpFirstOriginalNodeIndex = newOriginalNodeIndexMap[i];
            int tmpSecondOriginalNodeIndex = newOriginalNodeIndexMap[i + totalPickupNodeNumInGraph];

            double tmpTravelTime =
                vrpProblem->vrpData.disMatrix[tmpFirstOriginalNodeIndex][tmpSecondOriginalNodeIndex] +
                vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].serveTime;
            GRBLinExpr linExpr;
            linExpr += pickupStartServiceTime;
            linExpr -= deliveryStartServiceTime;

            std::string constrName = "pd_prece_" + std::to_string(i);
            model.addConstr(linExpr, GRB_LESS_EQUAL, -tmpTravelTime, constrName);
        }

        for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
            GRBVar pickupRouteIdentifierVar = routeIdentifierVariables[i];
            GRBVar deliveryRouteIdentifierVar = routeIdentifierVariables[i + totalPickupNodeNumInGraph];

            GRBLinExpr linExpr;
            linExpr += pickupRouteIdentifierVar;
            linExpr -= deliveryRouteIdentifierVar;

            std::string constrName = "pd_route_id_" + std::to_string(i);
            model.addConstr(linExpr, GRB_EQUAL, 0.0, constrName);
        }

        for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
            GRBVar routeIdentifierVar = routeIdentifierVariables[i];

            GRBLinExpr linExpr;
            linExpr += routeIdentifierVar;
            linExpr -= i * arcVariables[0][i];

            std::string constrName = "route_id_first_l_" + std::to_string(i);
            model.addConstr(linExpr, GRB_GREATER_EQUAL, 0.0, constrName);
        }

        for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
            GRBVar routeIdentifierVar = routeIdentifierVariables[i];

            GRBLinExpr linExpr;
            linExpr += routeIdentifierVar;
            linExpr += (totalPickupNodeNumInGraph - i) * arcVariables[0][i];

            std::string constrName = "route_id_first_u_" + std::to_string(i);
            model.addConstr(linExpr, GRB_LESS_EQUAL, totalPickupNodeNumInGraph, constrName);
        }

        for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
            GRBVar firstRouteIdentifierVar = routeIdentifierVariables[i];

            for (int j = 1; j < totalNodeNumInGraph - 1; ++j) {
                if (j == i) {
                    continue;
                }

                GRBVar secondRouteIdentifierVar = routeIdentifierVariables[j];

                GRBLinExpr linExpr;
                linExpr += firstRouteIdentifierVar;
                linExpr -= secondRouteIdentifierVar;
                linExpr += totalPickupNodeNumInGraph * arcVariables[i][j];

                std::string constrName = "route_id_l_" + std::to_string(i) + "_" + std::to_string(j);
                model.addConstr(linExpr, GRB_LESS_EQUAL, totalPickupNodeNumInGraph, constrName);
            }
        }

        for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
            GRBVar firstRouteIdentifierVar = routeIdentifierVariables[i];

            for (int j = 1; j < totalNodeNumInGraph - 1; ++j) {
                if (j == i) {
                    continue;
                }

                GRBVar secondRouteIdentifierVar = routeIdentifierVariables[j];

                GRBLinExpr linExpr;
                linExpr += firstRouteIdentifierVar;
                linExpr -= secondRouteIdentifierVar;
                linExpr -= totalPickupNodeNumInGraph * arcVariables[i][j];

                std::string constrName = "route_id_u_" + std::to_string(i) + "_" + std::to_string(j);
                model.addConstr(linExpr, GRB_GREATER_EQUAL, -totalPickupNodeNumInGraph, constrName);
            }
        }

        if (pricingSubProblem) {
            GRBLinExpr linExpr1;
            for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
                linExpr1 += arcVariables[0][i];
            }
            std::string constrName = "single_route_from_depot";
            model.addConstr(linExpr1, GRB_EQUAL, 1.0, constrName);

            GRBLinExpr linExpr2;
            for (int i = totalPickupNodeNumInGraph + 1; i < totalNodeNumInGraph - 1; ++i) {
                linExpr2 += arcVariables[i][totalNodeNumInGraph - 1];
            }
            constrName = "single_route_to_depot";
            model.addConstr(linExpr2, GRB_EQUAL, 1.0, constrName);

            GRBLinExpr linExpr3;
            for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
                for (int j = 1; j < totalNodeNumInGraph; ++j) {
                    linExpr3 += arcVariables[i][j];
                }
            }
            constrName = "route_length_limit";
            model.addConstr(linExpr3, GRB_LESS_EQUAL, pricingSubProblemData.pickupNodeNumLimitInRoute, constrName);
        }

        if (!requestPairingData.empty()) {
            for (auto& [pickupNodeId, pairingNodeIdSet] : requestPairingData) {
                GRBVar firstPickupRouteIdentifier = routeIdentifierVariables[originalNewNodeIndexMap[pickupNodeId]];

                for (int pairingNodeId : pairingNodeIdSet) {
                    GRBVar secondPickupRouteIdentifier =
                        routeIdentifierVariables[originalNewNodeIndexMap[pairingNodeId]];

                    GRBLinExpr tmpLinExpr;
                    tmpLinExpr += firstPickupRouteIdentifier;
                    tmpLinExpr -= secondPickupRouteIdentifier;
                    std::string constrName =
                        "request_pairing_constr_" + std::to_string(pickupNodeId) + "_" + std::to_string(pairingNodeId);
                    model.addConstr(tmpLinExpr, GRB_EQUAL, 0.0, constrName);
                }
            }
        }

        if (nullptr != initSolution) {
            if (fixRouteNum) {
                GRBLinExpr tmpLinExpr;
                for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
                    tmpLinExpr += arcVariables[0][i];
                }
                std::string constrName = "route_num_constraint";
                model.addConstr(tmpLinExpr, GRB_EQUAL, (double)initSolution->routes.size(), constrName);
            }
            if (freeNodes) {
                VrpSolution tmpSol(initSolution->vrpProblem);
                tmpSol.copySolution(*initSolution);

                while (tmpSol.unassignedNodeIds.size() * 2 < freeNodesNum) {
                    stringRemoval(vrpConfig, tmpSol);
                }
                std::vector<uint8_t> nodeFreeFlags;
                nodeFreeFlags.resize(totalNodeNumInGraph, 0);

                for (int i = 0; i < freeNodesNum / 2; ++i) {
                    int pickupNodeId = tmpSol.unassignedNodeIds[i].first;
                    int deliveryNodeId = tmpSol.unassignedNodeIds[i].second;

                    int newPickupNodeId = originalNewNodeIndexMap[pickupNodeId];
                    int newDeliveryNodeId = originalNewNodeIndexMap[deliveryNodeId];

                    nodeFreeFlags[newPickupNodeId] = 1;
                    nodeFreeFlags[newDeliveryNodeId] = 1;
                }

                for (int i = 0; i < totalNodeNumInGraph; ++i) {
                    for (int j = 0; j < totalNodeNumInGraph; ++j) {
                        arcVariables[i][j].set(GRB_DoubleAttr_Start, 0.0);
                    }
                }
                startServiceTimeVariables[0].set(GRB_DoubleAttr_Start,
                                                 vrpProblem->vrpData.nodeList[0].timeWindow.first);
                vehicleLoadVariables[0].set(GRB_DoubleAttr_Start, 0.0);

                double maxFinalDepotStartServiceTime = 0.0;
                for (const VrpRoute& route : initSolution->routes) {
                    int prevNodeId = 0;
                    int prevNewNodeId = 0;

                    const RouteNode* currentNode = route.startNode;
                    double startServiceTime = vrpProblem->vrpData.nodeList[0].timeWindow.first;
                    double vehicleLoad = 0.0;

                    int firstNodeId = currentNode->nodeId;
                    int firstNewNodeId = originalNewNodeIndexMap[firstNodeId];

                    while (nullptr != currentNode) {
                        int currentNodeId = currentNode->nodeId;
                        int tmpCurrentNewNodeId = originalNewNodeIndexMap[currentNodeId];

                        arcVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, 1.0);
                        startServiceTime += vrpProblem->vrpData.nodeList[prevNodeId].serveTime;
                        startServiceTime += vrpProblem->vrpData.disMatrix[prevNodeId][currentNodeId];
                        startServiceTime =
                            std::max(startServiceTime, vrpProblem->vrpData.nodeList[currentNodeId].timeWindow.first);
                        startServiceTimeVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, startServiceTime);

                        vehicleLoad += vrpProblem->vrpData.nodeList[currentNodeId].demand;
                        vehicleLoadVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, vehicleLoad);

                        routeIdentifierVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, firstNewNodeId);

                        if (!nodeFreeFlags[prevNewNodeId] && !nodeFreeFlags[tmpCurrentNewNodeId]) {
                            arcVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_LB, 1.0);
                        }

                        currentNode = currentNode->nexNode;
                        prevNodeId = currentNodeId;
                        prevNewNodeId = tmpCurrentNewNodeId;
                    }

                    arcVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start, 1.0);
                    if (!nodeFreeFlags[prevNewNodeId]) {
                        arcVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_LB, 1.0);
                    }
                    startServiceTime += vrpProblem->vrpData.nodeList[prevNodeId].serveTime;
                    startServiceTime += vrpProblem->vrpData.disMatrix[prevNodeId][0];
                    maxFinalDepotStartServiceTime = std::max(maxFinalDepotStartServiceTime, startServiceTime);
                }

                startServiceTimeVariables[totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start,
                                                                       maxFinalDepotStartServiceTime);
                vehicleLoadVariables[totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start, 0.0);
            } else {
                std::vector<uint8_t> routeFreeFlags;
                if (freeRoutes) {
                    routeFreeFlags.resize(initSolution->routes.size(), 0);
                } else {
                    routeFreeFlags.resize(initSolution->routes.size(), 1);
                }
                for (int routeIndex : freeRouteIndices) {
                    routeFreeFlags[routeIndex] = 1;
                    std::cout << "Free Route " << routeIndex
                              << "; Node num: " << initSolution->routes[routeIndex].nodeNum << std::endl;
                }
                for (int i = 0; i < totalNodeNumInGraph; ++i) {
                    for (int j = 0; j < totalNodeNumInGraph; ++j) {
                        arcVariables[i][j].set(GRB_DoubleAttr_Start, 0.0);
                    }
                }
                startServiceTimeVariables[0].set(GRB_DoubleAttr_Start,
                                                 vrpProblem->vrpData.nodeList[0].timeWindow.first);
                vehicleLoadVariables[0].set(GRB_DoubleAttr_Start, 0.0);

                double maxFinalDepotStartServiceTime = 0.0;
                for (const VrpRoute& route : initSolution->routes) {
                    uint8_t routeFree = routeFreeFlags[route.routeId];
                    int prevNodeId = 0;
                    int prevNewNodeId = 0;

                    const RouteNode* currentNode = route.startNode;
                    double startServiceTime = vrpProblem->vrpData.nodeList[0].timeWindow.first;
                    double vehicleLoad = 0.0;

                    int firstNodeId = currentNode->nodeId;
                    int firstNewNodeId = originalNewNodeIndexMap[firstNodeId];

                    while (nullptr != currentNode) {
                        int currentNodeId = currentNode->nodeId;
                        int tmpCurrentNewNodeId = originalNewNodeIndexMap[currentNodeId];

                        arcVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, 1.0);
                        startServiceTime += vrpProblem->vrpData.nodeList[prevNodeId].serveTime;
                        startServiceTime += vrpProblem->vrpData.disMatrix[prevNodeId][currentNodeId];
                        startServiceTime =
                            std::max(startServiceTime, vrpProblem->vrpData.nodeList[currentNodeId].timeWindow.first);
                        startServiceTimeVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, startServiceTime);

                        vehicleLoad += vrpProblem->vrpData.nodeList[currentNodeId].demand;
                        vehicleLoadVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, vehicleLoad);

                        routeIdentifierVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, firstNewNodeId);

                        if (!routeFree) {
                            arcVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_LB, 1.0);
                            startServiceTimeVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_LB, startServiceTime);
                            startServiceTimeVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_UB, startServiceTime);
                            vehicleLoadVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_LB, vehicleLoad);
                            vehicleLoadVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_UB, vehicleLoad);
                            routeIdentifierVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_LB, firstNewNodeId);
                            routeIdentifierVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_UB, firstNewNodeId);
                        }

                        currentNode = currentNode->nexNode;
                        prevNodeId = currentNodeId;
                        prevNewNodeId = tmpCurrentNewNodeId;
                    }

                    arcVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start, 1.0);
                    if (!routeFree) {
                        arcVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_LB, 1.0);
                    }
                    startServiceTime += vrpProblem->vrpData.nodeList[prevNodeId].serveTime;
                    startServiceTime += vrpProblem->vrpData.disMatrix[prevNodeId][0];
                    maxFinalDepotStartServiceTime = std::max(maxFinalDepotStartServiceTime, startServiceTime);
                }

                startServiceTimeVariables[totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start,
                                                                       maxFinalDepotStartServiceTime);
                vehicleLoadVariables[totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start, 0.0);
            }
        }

        if (!pricingSubProblem) {
            model.update();
            CallBack cb = CallBack(totalNodeNumInGraph, totalPickupNodeNumInGraph, newOriginalNodeIndexMap,
                                   arcVariables, vrpSolution, resultFolder);
            model.setCallback(&cb);
        }

        // auto now = std::chrono::system_clock::now();
        // std::time_t nowTime = std::chrono::system_clock::to_time_t(now);

        // std::tm* nowTm = std::localtime(&nowTime);

        // char buffer[80];
        // std::strftime(buffer, 80, "%Y%m%d%H%M%S", nowTm);

        // std::string timeStr(buffer);

        // model.write("two_index_model_" + timeStr + ".lp");

        model.optimize();
    } catch (GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    } catch (...) {
        std::cout << "Error during optimization" << std::endl;
    }

    delete[] vars;
}