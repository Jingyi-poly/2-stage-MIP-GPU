#include "domain/solver/memetic/removal/Removal.h"

#include <algorithm>
#include <cassert>
#include <unordered_map>
#include <unordered_set>

#include "util/random_utils.h"

void stringRemoval(const VrpConfig& vrpConfig, VrpSolution& vrpSolution) {
    const VrpData& vrpData = vrpSolution.vrpProblem->vrpData;
    double avgNodeNum = 0.0;
    for (const VrpRoute& vrpRoute : vrpSolution.routes) {
        avgNodeNum += vrpRoute.nodeNum;
    }
    avgNodeNum /= (double)vrpSolution.routes.size();

    double maxStringCardinality = GetMaxStringCardinality(avgNodeNum);
    double maxStringNum = GetMaxStringNum(maxStringCardinality);
    int stringNum = GetStringNum(maxStringNum);

    int seedNode = GetRandInt(1, vrpData.nodeNum - 1);
    std::unordered_set<int> ruinedRoutes;

    std::vector<int> toRemovedNodeIds;
    toRemovedNodeIds.resize(vrpData.nodeNum);
    int toRemovedNodeNum;

    for (int adjacentNodeId : vrpSolution.vrpProblem->adjacentList[seedNode]) {
        if (0 == adjacentNodeId) {
            continue;
        }
        int adjacentRouteId = vrpSolution.routeNodes[adjacentNodeId].routeId;
        if (adjacentRouteId < 0) {
            assert(nullptr == vrpSolution.routeNodes[adjacentNodeId].preNode);
            assert(nullptr == vrpSolution.routeNodes[adjacentNodeId].nexNode);
            continue;
        }
        assert(adjacentRouteId < vrpSolution.routes.size());

        if (ruinedRoutes.find(adjacentRouteId) != ruinedRoutes.end()) {
            continue;
        }

        double maxRouteStringCardinality =
            GetMaxRouteStringCardinality(maxStringCardinality, vrpSolution.routes[adjacentRouteId].nodeNum);
        int removedStringCardinality = GetRemovedStringCardinality(maxRouteStringCardinality);

        int maxPrevNodeNum = removedStringCardinality - 1;
        int prevNodeNum = 0;
        int prevNodeId = adjacentNodeId;
        while (0 != prevNodeId && prevNodeNum < maxPrevNodeNum) {
            RouteNode* tmpNode = vrpSolution.routeNodes[prevNodeId].preNode;
            if (nullptr == tmpNode) {
                break;
            } else {
                prevNodeId = tmpNode->nodeId;
            }
            prevNodeNum++;
        }

        prevNodeNum = GetRandInt(0, prevNodeNum);
        toRemovedNodeNum = 0;
        int visitedTaskNum = 0;
        prevNodeId = adjacentNodeId;
        while (0 != prevNodeId && visitedTaskNum < prevNodeNum) {
            RouteNode* tmpPrevNode = vrpSolution.routeNodes[prevNodeId].preNode;
            if (nullptr == tmpPrevNode) {
                break;
            } else {
                prevNodeId = tmpPrevNode->nodeId;
            }

            if (0 != prevNodeId) {
                toRemovedNodeIds[toRemovedNodeNum] = prevNodeId;
                toRemovedNodeNum++;
                toRemovedNodeIds[toRemovedNodeNum] = tmpPrevNode->corresRouteNode->nodeId;
                toRemovedNodeNum++;
            }

            visitedTaskNum++;
        }

        toRemovedNodeIds[toRemovedNodeNum] = adjacentNodeId;
        toRemovedNodeNum++;
        toRemovedNodeIds[toRemovedNodeNum] = vrpData.nodeList[adjacentNodeId].corresNodeId;
        toRemovedNodeNum++;

        int nextNodeNum = removedStringCardinality - prevNodeNum - 1;
        visitedTaskNum = 0;
        int nextNodeId = adjacentNodeId;
        while (0 != nextNodeId && visitedTaskNum < nextNodeNum) {
            RouteNode* tmpNextNode = vrpSolution.routeNodes[nextNodeId].nexNode;
            if (nullptr == tmpNextNode) {
                break;
            } else {
                nextNodeId = tmpNextNode->nodeId;
            }

            if (0 != nextNodeId) {
                toRemovedNodeIds[toRemovedNodeNum] = nextNodeId;
                toRemovedNodeNum++;
                toRemovedNodeIds[toRemovedNodeNum] = tmpNextNode->corresRouteNode->nodeId;
                toRemovedNodeNum++;
            }
            visitedTaskNum++;
        }

        assert(0 == toRemovedNodeNum % 2);

        for (int i = 0; i < toRemovedNodeNum; i += 2) {
            int tmpToRemoveNodeId = toRemovedNodeIds[i];
            assert(vrpSolution.routeNodes[tmpToRemoveNodeId].corresRouteNode->nodeId == toRemovedNodeIds[i + 1]);
            assert(vrpSolution.routeNodes[toRemovedNodeIds[i + 1]].corresRouteNode->nodeId == toRemovedNodeIds[i]);

            if (0 > vrpSolution.routeNodes[tmpToRemoveNodeId].routeId) {
                continue;
            }
            assert(vrpSolution.routeNodes[tmpToRemoveNodeId].routeId == adjacentRouteId);
            assert(vrpSolution.routeNodes[toRemovedNodeIds[i + 1]].routeId == adjacentRouteId);

            if (Utils::VrpNodeType::kPickup == vrpSolution.vrpProblem->vrpData.nodeList[tmpToRemoveNodeId].nodeType) {
                vrpSolution.removeNode(adjacentRouteId, tmpToRemoveNodeId, toRemovedNodeIds[i + 1], false);

            } else {
                vrpSolution.removeNode(adjacentRouteId, toRemovedNodeIds[i + 1], tmpToRemoveNodeId, false);
            }
        }

        if (vrpSolution.routes[adjacentRouteId].nodeNum > 0) {
            vrpSolution.updateRouteData(adjacentRouteId, vrpSolution.routes[adjacentRouteId].startNode,
                                        vrpSolution.routes[adjacentRouteId].endNode);
        }

        ruinedRoutes.insert(adjacentRouteId);
        if ((int)ruinedRoutes.size() >= stringNum) {
            break;
        }
    }
}

double calculateRemovalCost(const VrpConfig& vrpConfig, const VrpSolution& vrpSolution, int routeId,
                            RouteNode* pickupNode, RouteNode* deliveryNode) {
    assert(pickupNode->routeId == routeId);
    assert(deliveryNode->routeId == routeId);
    assert(pickupNode->corresRouteNode == deliveryNode);
    assert(deliveryNode->corresRouteNode == pickupNode);
    assert(vrpSolution.routes[routeId].nodeNum > 0);

    const VrpData& vrpData = vrpSolution.vrpProblem->vrpData;
    const VrpRoute& vrpRoute = vrpSolution.routes[routeId];
    double oldTotalCost = vrpRoute.distance + vrpRoute.twViolation * vrpConfig.twPenaltyCoeff +
                          vrpRoute.endNode->accumulatedCapacityViolation * vrpConfig.capacityPenaltyCoeff;

    RouteNode* prevNode = pickupNode->preNode;
    double prevLeaveTime = vrpData.nodeList[0].timeWindow.first + vrpData.nodeList[0].serveTime;
    double prevForwardTwPenaltySlack = 0.0;
    std::vector<double> prevLoad = std::vector<double>(vrpData.numScenarios, 0.0);
    double prevAccumulatedCapacityViolations = 0.0;
    double prevAccumulatedDistance = 0.0;

    if (nullptr != prevNode) {
        prevLeaveTime = prevNode->leaveTime;
        prevForwardTwPenaltySlack = prevNode->forwardTwPenaltySlack;
        std::copy_n(prevNode->loadStatus.begin(), vrpData.numScenarios, prevLoad.begin());
        prevAccumulatedCapacityViolations = prevNode->accumulatedCapacityViolation;
        prevAccumulatedDistance = prevNode->accumulatedDistance;
    }

    if (deliveryNode->preNode == pickupNode) {
        assert(pickupNode->nexNode == deliveryNode);
        if (deliveryNode->nexNode == nullptr) {
            assert(vrpRoute.endNode == deliveryNode);
            if (pickupNode->preNode == nullptr) {
                assert(vrpRoute.startNode == pickupNode);
                assert(vrpRoute.endNode == deliveryNode);
                assert(vrpRoute.nodeNum == 2);
                return oldTotalCost;
            } else {
                int pickupPreNodeId = pickupNode->preNode->nodeId;
                double distanceDelta =
                    vrpRoute.distance - prevAccumulatedDistance - vrpData.disMatrix[pickupPreNodeId][0];
                double capacityViolationDelta =
                    vrpRoute.endNode->accumulatedCapacityViolation - prevAccumulatedCapacityViolations;
                double twViolationDelta = vrpRoute.twViolation - prevForwardTwPenaltySlack -
                                          std::max(prevLeaveTime + vrpData.disMatrix[pickupPreNodeId][0] -
                                                       vrpData.nodeList[0].timeWindow.second,
                                                   0.0);

                return distanceDelta + capacityViolationDelta * vrpConfig.capacityPenaltyCoeff +
                       twViolationDelta * vrpConfig.twPenaltyCoeff;
            }
        } else {
            if (pickupNode->preNode == nullptr) {
                assert(vrpRoute.startNode == pickupNode);
                double distanceDelta =
                    deliveryNode->nexNode->accumulatedDistance - vrpData.disMatrix[0][deliveryNode->nexNode->nodeId];
                double capacityViolationDelta = deliveryNode->accumulatedCapacityViolation;
                double twViolationDelta =
                    vrpRoute.twViolation -
                    (deliveryNode->nexNode->backwardTwPenaltySlack +
                     std::max(vrpData.nodeList[0].timeWindow.first + vrpData.nodeList[0].serveTime +
                                  vrpData.disMatrix[0][deliveryNode->nexNode->nodeId] -
                                  deliveryNode->nexNode->latestStartServiceTime,
                              0.0));

                return distanceDelta + capacityViolationDelta * vrpConfig.capacityPenaltyCoeff +
                       twViolationDelta * vrpConfig.twPenaltyCoeff;
            } else {
                double distanceDelta = deliveryNode->nexNode->accumulatedDistance - prevAccumulatedDistance -
                                       vrpData.disMatrix[pickupNode->preNode->nodeId][deliveryNode->nexNode->nodeId];
                double capacityViolationDelta =
                    deliveryNode->accumulatedCapacityViolation - prevAccumulatedCapacityViolations;
                double twViolationDelta =
                    vrpRoute.twViolation - prevForwardTwPenaltySlack - deliveryNode->nexNode->backwardTwPenaltySlack -
                    std::max(prevLeaveTime +
                                 vrpData.disMatrix[pickupNode->preNode->nodeId][deliveryNode->nexNode->nodeId] -
                                 deliveryNode->nexNode->latestStartServiceTime,
                             0.0);

                return distanceDelta + capacityViolationDelta * vrpConfig.capacityPenaltyCoeff +
                       twViolationDelta * vrpConfig.twPenaltyCoeff;
            }
        }
    }

    RouteNode* currentNode = pickupNode->nexNode;
    while (true) {
        int currentNodeId = currentNode->nodeId;
        double arcDistance = vrpData.disMatrix[nullptr == prevNode ? 0 : prevNode->nodeId][currentNodeId];
        double currentArriveTime = prevLeaveTime + arcDistance;

        const VrpNode& nodeData = vrpData.nodeList[currentNodeId];
        double currentAccumulatedCapacityViolations = prevAccumulatedCapacityViolations;
        for (int i = 0; i < vrpData.numScenarios; ++i) {
            prevLoad[i] += nodeData.demand[i];
            currentAccumulatedCapacityViolations += std::max(0.0, prevLoad[i] - vrpData.vehicleCapacity);
        }

        double currentForwardTwPenaltySlack =
            prevForwardTwPenaltySlack + std::max(0.0, currentArriveTime - nodeData.timeWindow.second);
        double currentStartServiceTime =
            std::min(std::max(currentArriveTime, nodeData.timeWindow.first), nodeData.timeWindow.second);

        double currentAccumulatedDistance = prevAccumulatedDistance + arcDistance;

        prevLeaveTime = currentStartServiceTime + nodeData.serveTime;
        prevForwardTwPenaltySlack = currentForwardTwPenaltySlack;
        prevAccumulatedDistance = currentAccumulatedDistance;
        prevAccumulatedCapacityViolations = currentAccumulatedCapacityViolations;

        prevNode = currentNode;
        currentNode = currentNode->nexNode;
        if (currentNode == deliveryNode) {
            break;
        }
    }

    if (deliveryNode->nexNode == nullptr) {
        double distanceDelta = vrpRoute.distance - prevAccumulatedDistance - vrpData.disMatrix[prevNode->nodeId][0];
        double capacityViolationDelta =
            vrpRoute.endNode->accumulatedCapacityViolation - prevAccumulatedCapacityViolations;
        double twViolationDelta =
            vrpRoute.twViolation - prevForwardTwPenaltySlack -
            std::max(prevLeaveTime + vrpData.disMatrix[prevNode->nodeId][0] - vrpData.nodeList[0].timeWindow.second,
                     0.0);

        return distanceDelta + capacityViolationDelta * vrpConfig.capacityPenaltyCoeff +
               twViolationDelta * vrpConfig.twPenaltyCoeff;
    } else {
        double distanceDelta = deliveryNode->nexNode->accumulatedDistance - prevAccumulatedDistance -
                               vrpData.disMatrix[prevNode->nodeId][deliveryNode->nexNode->nodeId];
        double capacityViolationDelta = deliveryNode->accumulatedCapacityViolation - prevAccumulatedCapacityViolations;
        double twViolationDelta =
            vrpRoute.twViolation - prevForwardTwPenaltySlack - deliveryNode->nexNode->backwardTwPenaltySlack -
            std::max(prevLeaveTime + vrpData.disMatrix[prevNode->nodeId][deliveryNode->nexNode->nodeId] -
                         deliveryNode->nexNode->latestStartServiceTime,
                     0.0);

        return distanceDelta + capacityViolationDelta * vrpConfig.capacityPenaltyCoeff +
               twViolationDelta * vrpConfig.twPenaltyCoeff;
    }
}

void worstRemoval(const VrpConfig& vrpConfig, VrpSolution& vrpSolution) {
    const VrpData& vrpData = vrpSolution.vrpProblem->vrpData;

    std::vector<int> assignedPickupNodeIds;
    std::unordered_map<int, double> assignedPickupNodeCostMap;

    for (const VrpRoute& route : vrpSolution.routes) {
        RouteNode* currentNode = route.startNode;
        while (nullptr != currentNode) {
            int currentNodeId = currentNode->nodeId;
            if (Utils::VrpNodeType::kPickup == vrpData.nodeList[currentNodeId].nodeType) {
                assignedPickupNodeIds.push_back(currentNodeId);
                double cost = calculateRemovalCost(vrpConfig, vrpSolution, route.routeId, currentNode,
                                                   currentNode->corresRouteNode);

                assignedPickupNodeCostMap[currentNodeId] = cost;
            }
            currentNode = currentNode->nexNode;
        }
    }
    for (int i = 0; i < kAverageRemovedTasks; ++i) {
        std::sort(assignedPickupNodeIds.begin(), assignedPickupNodeIds.end(),
                  [&](int a, int b) { return assignedPickupNodeCostMap[a] > assignedPickupNodeCostMap[b]; });

        double tmpRandomVal = GetRandDouble(0.0, 1.0);
        tmpRandomVal = std::pow(tmpRandomVal, kRemovalRandomDegree);
        int removedPickupNodeId = assignedPickupNodeIds[std::min(
            (int)std::floor(tmpRandomVal * (int)assignedPickupNodeIds.size()), (int)assignedPickupNodeIds.size() - 1)];

        RouteNode* pickupNode = &vrpSolution.routeNodes[removedPickupNodeId];
        int toRemoveRouteId = pickupNode->routeId;

        vrpSolution.removeNode(toRemoveRouteId, pickupNode->nodeId, pickupNode->corresRouteNode->nodeId, true);
        for (int j = 0; j < (int)assignedPickupNodeIds.size();) {
            if (pickupNode->nodeId == assignedPickupNodeIds[j]) {
                assignedPickupNodeIds[j] = assignedPickupNodeIds.back();
                assignedPickupNodeIds.pop_back();
                break;
            } else {
                ++j;
            }
        }

        if (assignedPickupNodeIds.empty()) {
            break;
        }

        RouteNode* currentNode = vrpSolution.routes[toRemoveRouteId].startNode;
        while (nullptr != currentNode) {
            int currentNodeId = currentNode->nodeId;
            if (Utils::VrpNodeType::kPickup == vrpData.nodeList[currentNodeId].nodeType) {
                double cost = calculateRemovalCost(vrpConfig, vrpSolution, toRemoveRouteId, currentNode,
                                                   currentNode->corresRouteNode);

                assignedPickupNodeCostMap[currentNodeId] = cost;
            }
            currentNode = currentNode->nexNode;
        }
    }
}

void randomRemoval(const VrpConfig& vrpConfig, VrpSolution& vrpSolution) {
    const VrpData& vrpData = vrpSolution.vrpProblem->vrpData;

    std::vector<int> assignedPickupNodeIds;
    assignedPickupNodeIds.reserve(vrpData.pickNodeIds.size());

    for (const VrpRoute& route : vrpSolution.routes) {
        RouteNode* currentNode = route.startNode;
        while (nullptr != currentNode) {
            int currentNodeId = currentNode->nodeId;
            if (Utils::VrpNodeType::kPickup == vrpData.nodeList[currentNodeId].nodeType) {
                assignedPickupNodeIds.push_back(currentNodeId);
            }
            currentNode = currentNode->nexNode;
        }
    }
    std::shuffle(assignedPickupNodeIds.begin(), assignedPickupNodeIds.end(), GetRandomGenerator());

    for (int i = 0; i < std::min(kAverageRemovedTasks, (int)assignedPickupNodeIds.size()); ++i) {
        int removedPickupNodeId = assignedPickupNodeIds[i];

        RouteNode* pickupNode = &vrpSolution.routeNodes[removedPickupNodeId];
        int toRemoveRouteId = pickupNode->routeId;

        vrpSolution.removeNode(toRemoveRouteId, pickupNode->nodeId, pickupNode->corresRouteNode->nodeId, true);
    }
}

void shawRemoval(const VrpConfig& vrpConfig, VrpSolution& vrpSolution) {
    const VrpProblem* vrpProblem = vrpSolution.vrpProblem;
    const VrpData& vrpData = vrpProblem->vrpData;

    std::vector<int> assignedPickupNodeIds;
    assignedPickupNodeIds.reserve(vrpData.pickNodeIds.size());

    for (const VrpRoute& route : vrpSolution.routes) {
        RouteNode* currentNode = route.startNode;
        while (nullptr != currentNode) {
            int currentNodeId = currentNode->nodeId;
            if (Utils::VrpNodeType::kPickup == vrpData.nodeList[currentNodeId].nodeType) {
                assignedPickupNodeIds.push_back(currentNodeId);
            }
            currentNode = currentNode->nexNode;
        }
    }

    double maxStartServiceTime = 0.0;
    for (int assignedPickupNodeId : assignedPickupNodeIds) {
        int assignedDeliveryNodeId = vrpData.nodeList[assignedPickupNodeId].corresNodeId;
        double pickupStartServiceTime = vrpSolution.routeNodes[assignedPickupNodeId].startServiceTime;
        double deliveryStartServiceTime = vrpSolution.routeNodes[assignedDeliveryNodeId].startServiceTime;
        maxStartServiceTime = std::max(maxStartServiceTime, pickupStartServiceTime);
        maxStartServiceTime = std::max(maxStartServiceTime, deliveryStartServiceTime);
    }

    std::vector<int> removedPickupNodeIds;
    removedPickupNodeIds.reserve(kAverageRemovedTasks);
    int randIndex = GetRandInt(0, (int)assignedPickupNodeIds.size() - 1);
    int removedPickupNodeId = assignedPickupNodeIds[randIndex];
    removedPickupNodeIds.push_back(removedPickupNodeId);
    vrpSolution.removeNode(vrpSolution.routeNodes[removedPickupNodeId].routeId, removedPickupNodeId,
                           vrpSolution.routeNodes[removedPickupNodeId].corresRouteNode->nodeId, true);

    assignedPickupNodeIds[randIndex] = assignedPickupNodeIds.back();
    assignedPickupNodeIds.pop_back();

    std::unordered_map<int, double> requestRelatednessMap;

    for (int i = 1; i < kAverageRemovedTasks; ++i) {
        assert(i == (int)removedPickupNodeIds.size());
        randIndex = GetRandInt(0, (int)removedPickupNodeIds.size() - 1);
        int tmpPickupNodeId = removedPickupNodeIds[randIndex];
        int tmpDeliveryNodeId = vrpData.nodeList[tmpPickupNodeId].corresNodeId;
        double tmpPickupStartServiceTime = vrpSolution.routeNodes[tmpPickupNodeId].startServiceTime;
        double tmpDeliveryStartServiceTime = vrpSolution.routeNodes[tmpDeliveryNodeId].startServiceTime;

        for (int tmpAssignedPickupNodeId : assignedPickupNodeIds) {
            int tmpAssignedDeliveryNodeId = vrpData.nodeList[tmpAssignedPickupNodeId].corresNodeId;

            double tmpAssignedPickupStartServiceTime = vrpSolution.routeNodes[tmpAssignedPickupNodeId].startServiceTime;
            double tmpAssignedDeliveryStartServiceTime =
                vrpSolution.routeNodes[tmpAssignedDeliveryNodeId].startServiceTime;
            double timeDiff =
                std::fabs(tmpPickupStartServiceTime - tmpAssignedPickupStartServiceTime) / maxStartServiceTime +
                std::fabs(tmpDeliveryStartServiceTime - tmpAssignedDeliveryStartServiceTime) / maxStartServiceTime;

            requestRelatednessMap[tmpAssignedPickupNodeId] =
                kRequestDistanceWeight *
                    vrpSolution.vrpProblem->requestDistanceMap[tmpPickupNodeId][tmpAssignedPickupNodeId] +
                kRequestDemandDiffWeight *
                    vrpSolution.vrpProblem->requestDemandDiffMap[tmpPickupNodeId][tmpAssignedPickupNodeId] +
                kRequestTemporalWeight * timeDiff;
        }

        std::sort(assignedPickupNodeIds.begin(), assignedPickupNodeIds.end(),
                  [&](int a, int b) { return requestRelatednessMap[a] < requestRelatednessMap[b]; });

        double tmpRandomVal = GetRandDouble(0.0, 1.0);
        tmpRandomVal = std::pow(tmpRandomVal, kRemovalRandomDegree);
        removedPickupNodeId = assignedPickupNodeIds[std::min(
            (int)std::floor(tmpRandomVal * (int)assignedPickupNodeIds.size()), (int)assignedPickupNodeIds.size() - 1)];
        removedPickupNodeIds.push_back(removedPickupNodeId);

        RouteNode* pickupNode = &vrpSolution.routeNodes[removedPickupNodeId];
        int toRemoveRouteId = pickupNode->routeId;

        vrpSolution.removeNode(toRemoveRouteId, pickupNode->nodeId, pickupNode->corresRouteNode->nodeId, true);
        for (int j = 0; j < (int)assignedPickupNodeIds.size();) {
            if (pickupNode->nodeId == assignedPickupNodeIds[j]) {
                assignedPickupNodeIds[j] = assignedPickupNodeIds.back();
                assignedPickupNodeIds.pop_back();
                break;
            } else {
                ++j;
            }
        }

        if (assignedPickupNodeIds.empty()) {
            break;
        }

        std::unordered_map<int, double> nodeStartServiceTimeMap;
        RouteNode* currentNode = vrpSolution.routes[toRemoveRouteId].startNode;
        while (nullptr != currentNode) {
            int currentNodeId = currentNode->nodeId;
            nodeStartServiceTimeMap[currentNodeId] = currentNode->startServiceTime;
            currentNode = currentNode->nexNode;
        }
    }
}