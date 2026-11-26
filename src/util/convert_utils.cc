#include "util/convert_utils.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>

void ConvertVrpProblem(const PDPTW& pdptw, VrpProblem& vrpProblem) {
    VrpData& vrpData = vrpProblem.vrpData;

    vrpData.name = pdptw.instance_name;
    vrpData.numScenarios = pdptw.scenario_num;
    vrpData.nodeNum = pdptw.task_num;
    vrpData.vehicleUpLimit = pdptw.vehicle_num;
    vrpData.vehicleCapacity = pdptw.vehicle_capacity;
    vrpData.nodeList.resize(vrpData.nodeNum);
    vrpData.pickNodeIds.reserve((vrpData.nodeNum - 1) / 2);
    vrpData.disMatrix.resize(vrpData.nodeNum);

    for (int i = 0; i < vrpData.nodeNum; ++i) {
        idx_t newIndex = pdptw.original_new_task_index_map.find(i)->second;

        VrpNode& vrpNode = vrpData.nodeList[i];
        vrpNode.id = i;
        vrpNode.demand.assign(vrpData.numScenarios, 0.0);
        for (int j = 0; j < vrpData.numScenarios; ++j) {
            vrpNode.demand[j] = static_cast<double>(pdptw.task_demands[newIndex][j]);
        }
        if (0 == i) {
            vrpNode.nodeType = Utils::VrpNodeType::kDepot;
        } else {
            if (1 == newIndex % 2) {
                vrpNode.nodeType = Utils::VrpNodeType::kPickup;
                vrpData.pickNodeIds.push_back(i);
                vrpNode.corresNodeId = pdptw.new_original_task_index_map[newIndex + 1];
            } else {
                vrpNode.nodeType = Utils::VrpNodeType::kDelivery;
                vrpNode.corresNodeId = pdptw.new_original_task_index_map[newIndex - 1];
            }
        }
        vrpNode.serveTime = pdptw.task_service_times[newIndex];
        vrpNode.timeWindow.first = pdptw.task_time_windows[2 * newIndex];
        vrpNode.timeWindow.second = pdptw.task_time_windows[2 * newIndex + 1];
        vrpNode.lon = pdptw.task_positions[2 * newIndex];
        vrpNode.lat = pdptw.task_positions[2 * newIndex + 1];

        vrpData.disMatrix[i].resize(vrpData.nodeNum);
        int disMatrixOffset = newIndex * pdptw.task_num;
        for (int j = 0; j < vrpData.nodeNum; ++j) {
            if (1 == pdptw.feasible_arcs[disMatrixOffset + j]) {
                vrpNode.candiNextNodes.insert(pdptw.new_original_task_index_map[j]);
            }

            if (1 == pdptw.feasible_arcs[j * pdptw.task_num + newIndex]) {
                vrpNode.candiPreNodes.insert(pdptw.new_original_task_index_map[j]);
            }

            vrpData.disMatrix[i][j] =
                pdptw.distance_matrix[disMatrixOffset + pdptw.original_new_task_index_map.find(j)->second];
        }
    }

    vrpData.demandMatrix.assign(static_cast<size_t>(vrpData.nodeNum) * vrpData.numScenarios, 0.0f);
    for (int i = 0; i < vrpData.nodeNum; ++i) {
        const VrpNode& vrpNode = vrpData.nodeList[i];
        size_t offset = static_cast<size_t>(i) * vrpData.numScenarios;
        for (int j = 0; j < vrpData.numScenarios; ++j) {
            vrpData.demandMatrix[offset + j] = static_cast<float>(vrpNode.demand[j]);
        }
    }

    vrpProblem.adjacentList.resize(vrpData.nodeNum);
    for (int i = 0; i < vrpData.nodeNum; ++i) {
        vrpProblem.adjacentList[i].resize(vrpData.nodeNum, 0);
        std::iota(vrpProblem.adjacentList[i].begin(), vrpProblem.adjacentList[i].end(), 0);
        std::sort(vrpProblem.adjacentList[i].begin(), vrpProblem.adjacentList[i].end(),
                  [&](int a, int b) { return vrpData.disMatrix[i][a] < vrpData.disMatrix[i][b]; });
    }

    double maxDistance = 0.0;
    double maxDemand = 0.0;
    for (int i = 0; i < (int)vrpData.pickNodeIds.size(); ++i) {
        int firstRequestId = vrpData.pickNodeIds[i];
        double firstRequestDemand = vrpData.nodeList[firstRequestId].demand[0];
        assert(firstRequestDemand > 0.0);
        for (int j = 0; j < (int)vrpData.pickNodeIds.size(); ++j) {
            if (i == j) {
                continue;
            }
            int secondRequestId = vrpData.pickNodeIds[j];
            double secondRequestDemand = vrpData.nodeList[secondRequestId].demand[0];
            assert(secondRequestDemand > 0.0);
            double pickupNodeDis = vrpData.disMatrix[firstRequestId][secondRequestId];
            double deliveryNodeDis = vrpData.disMatrix[vrpData.nodeList[firstRequestId].corresNodeId]
                                                      [vrpData.nodeList[secondRequestId].corresNodeId];
            maxDistance = std::max(pickupNodeDis, maxDistance);
            maxDistance = std::max(deliveryNodeDis, maxDistance);

            vrpProblem.requestDistanceMap[firstRequestId][secondRequestId] = pickupNodeDis + deliveryNodeDis;
            vrpProblem.requestDemandDiffMap[firstRequestId][secondRequestId] =
                std::fabs(firstRequestDemand - secondRequestDemand);
        }
        maxDemand = std::max(maxDemand, vrpData.nodeList[firstRequestId].demand[0]);
    }
    for (auto& [key1, value1] : vrpProblem.requestDistanceMap) {
        for (auto& [key2, value2] : value1) {
            value2 /= maxDistance;
        }
    }
    for (auto& [key1, value1] : vrpProblem.requestDemandDiffMap) {
        for (auto& [key2, value2] : value1) {
            value2 /= maxDemand;
        }
    }

    for (int pickupNodeId : vrpData.pickNodeIds) {
        int deliveryNodeId = vrpData.nodeList[pickupNodeId].corresNodeId;
        bool ableInsertBetween = false;
        for (int j = 1; j < vrpData.nodeNum; ++j) {
            if (j == pickupNodeId || j == deliveryNodeId) {
                continue;
            }

            double tmpEarliestStartServiceTime = vrpData.nodeList[pickupNodeId].timeWindow.first;
            tmpEarliestStartServiceTime += vrpData.nodeList[pickupNodeId].serveTime;
            tmpEarliestStartServiceTime += vrpData.disMatrix[pickupNodeId][j];
            if (tmpEarliestStartServiceTime > vrpData.nodeList[j].timeWindow.second) {
                continue;
            }
            tmpEarliestStartServiceTime = std::max(tmpEarliestStartServiceTime, vrpData.nodeList[j].timeWindow.first);
            tmpEarliestStartServiceTime += vrpData.nodeList[j].serveTime;
            tmpEarliestStartServiceTime += vrpData.disMatrix[j][deliveryNodeId];
            if (tmpEarliestStartServiceTime <= vrpData.nodeList[deliveryNodeId].timeWindow.second) {
                ableInsertBetween = true;
                break;
            }
        }

        if (!ableInsertBetween) {
            if (1 != vrpData.nodeList[pickupNodeId].candiNextNodes.size()) {
                exit(0);
            }
            if (1 != vrpData.nodeList[deliveryNodeId].candiPreNodes.size()) {
                exit(0);
            }
            if (deliveryNodeId != *vrpData.nodeList[pickupNodeId].candiNextNodes.begin()) {
                exit(0);
            }
            if (pickupNodeId != *vrpData.nodeList[deliveryNodeId].candiPreNodes.begin()) {
                exit(0);
            }
        }
    }
}

void ConvertVrpSolution(const Solution& solution, VrpSolution& vrpSolution) {
    std::vector<uint8_t> unassignedNodeFlag(vrpSolution.vrpProblem->vrpData.nodeNum, 1);

    const PDPTW& pdptw = solution.problem_;
    vrpSolution.vehicleNum = (int)(solution.routes_.size());
    vrpSolution.routes.clear();
    for (int i = 0; i < (int)solution.routes_.size(); ++i) {
        vrpSolution.routes.emplace_back(i);
        const std::vector<idx_t>& route = solution.routes_[i];

        RouteNode* prevNode = nullptr;
        for (idx_t taskIndex : route) {
            if (0 == taskIndex) {
                continue;
            }

            int nodeId = pdptw.new_original_task_index_map[taskIndex];
            RouteNode* tmpNode = &vrpSolution.routeNodes[nodeId];
            if (nullptr == prevNode) {
                vrpSolution.routes[i].startNode = tmpNode;
            } else {
                prevNode->nexNode = tmpNode;
            }

            tmpNode->preNode = prevNode;
            tmpNode->routeId = i;

            unassignedNodeFlag[nodeId] = 0;

            vrpSolution.routes[i].nodeNum += 1;
            prevNode = tmpNode;
        }

        vrpSolution.routes[i].endNode = prevNode;
        prevNode->nexNode = nullptr;

        vrpSolution.updateRouteData(i, vrpSolution.routes[i].startNode, vrpSolution.routes[i].endNode);
    }

    for (int i = 0; i < vrpSolution.unassignedNodeIds.size();) {
        const std::pair<int, int>& unassignedNodePair = vrpSolution.unassignedNodeIds[i];
        if (!unassignedNodeFlag[unassignedNodePair.first] && !unassignedNodeFlag[unassignedNodePair.second]) {
            vrpSolution.unassignedNodeIds.erase(vrpSolution.unassignedNodeIds.begin() + i);
        } else {
            ++i;
        }
    }
}