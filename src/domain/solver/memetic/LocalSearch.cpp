#include "domain/solver/memetic/LocalSearch.h"

#include <cassert>

#include "domain/solver/memetic/insert/Insert.h"
#include "domain/solver/memetic/removal/Removal.h"
#include "util/random_utils.h"

bool outRelocateMove(const VrpConfig& vrpConfig, VrpSolution& vrpSolution, int pickupNodeId, int deliveryNodeId) {
    
    assert(pickupNodeId > 0);
    assert(deliveryNodeId > 0);
    assert(pickupNodeId != deliveryNodeId);
    assert(vrpSolution.routeNodes[pickupNodeId].corresRouteNode->nodeId == deliveryNodeId);
    assert(vrpSolution.routeNodes[deliveryNodeId].corresRouteNode->nodeId == pickupNodeId);

    RouteNode* pickupNode = &vrpSolution.routeNodes[pickupNodeId];
    RouteNode* deliveryNode = &vrpSolution.routeNodes[deliveryNodeId];
    assert(pickupNode->corresRouteNode == deliveryNode);
    assert(deliveryNode->corresRouteNode == pickupNode);
    assert(pickupNode->routeId == deliveryNode->routeId);

    int currentRouteId = pickupNode->routeId;

    double removalCost = calculateRemovalCost(vrpConfig, vrpSolution, currentRouteId, pickupNode, deliveryNode);

    int minInsertCostRouteId = -1;
    double minInsertCost = std::numeric_limits<double>::max();
    std::vector<InsertionData> insertions;
    RouteNode* pickupInsertPrevNode = nullptr;
    RouteNode* deliveryInsertPrevNode = nullptr;
    for (const VrpRoute& route : vrpSolution.routes) {
        if (route.routeId == currentRouteId) {
            continue;
        }

        insertions.clear();
        calInsertionData(vrpSolution, pickupNode, deliveryNode, route.routeId, insertions, true);
        for (const InsertionData& insertion : insertions) {
            double tmpCost = insertion.deltaDistance;
            tmpCost += vrpConfig.twPenaltyCoeff * insertion.deltaTwPenalty;
            tmpCost += vrpConfig.capacityPenaltyCoeff * insertion.deltaCapacityViolations;

            if (tmpCost < minInsertCost) {
                minInsertCost = tmpCost;
                pickupInsertPrevNode = insertion.pickupPrevNode;
                deliveryInsertPrevNode = insertion.deliveryPrevNode;
                minInsertCostRouteId = route.routeId;
            }
        }
    }

    if (pickupInsertPrevNode == nullptr || deliveryInsertPrevNode == nullptr) {
        return false;
    }

    if (minInsertCost >= removalCost) {
        return false;
    }

    vrpSolution.removeNode(currentRouteId, pickupNodeId, deliveryNodeId, true);
    vrpSolution.insertNode(minInsertCostRouteId, pickupNodeId, deliveryNodeId, pickupInsertPrevNode,
                           deliveryInsertPrevNode, true);

    return true;
}

bool outRelocateMove(const VrpConfig& vrpConfig, VrpSolution& vrpSolution) {
    std::vector<int> assignedPickupNodeIds;
    assignedPickupNodeIds.reserve(vrpSolution.vrpProblem->vrpData.nodeNum / 2);

    for (const VrpRoute& route : vrpSolution.routes) {
        if (route.nodeNum <= 0) {
            continue;
        }

        RouteNode* currentNode = route.startNode;
        while (nullptr != currentNode) {
            if (Utils::VrpNodeType::kPickup == vrpSolution.vrpProblem->vrpData.nodeList[currentNode->nodeId].nodeType) {
                assignedPickupNodeIds.push_back(currentNode->nodeId);
            }
            currentNode = currentNode->nexNode;
        }
    }

    while (!assignedPickupNodeIds.empty()) {
        int randIndex = GetRandInt(0, (int)assignedPickupNodeIds.size() - 1);

        int selectedAssignedPickupNodeId = assignedPickupNodeIds[randIndex];
        int selectedAssignedDeliveryNodeId =
            vrpSolution.routeNodes[selectedAssignedPickupNodeId].corresRouteNode->nodeId;

        if (vrpSolution.routes[vrpSolution.routeNodes[selectedAssignedPickupNodeId].routeId].nodeNum <= 2) {
            assignedPickupNodeIds[randIndex] = assignedPickupNodeIds.back();
            assignedPickupNodeIds.pop_back();
            continue;
        }

        bool tmpRes =
            outRelocateMove(vrpConfig, vrpSolution, selectedAssignedPickupNodeId, selectedAssignedDeliveryNodeId);

        if (tmpRes) {
            return true;
        } else {
            assignedPickupNodeIds[randIndex] = assignedPickupNodeIds.back();
            assignedPickupNodeIds.pop_back();
        }
    }

    return false;
}

double get_reversed_cost(const VrpConfig& vrpConfig, const VrpSolution& vrpSolution, int routeId,
                            RouteNode* node_lef, RouteNode* node_rig) {
    const VrpRoute& vrpRoute = vrpSolution.routes[routeId];

    RouteNode* prevNode = node_lef->preNode;
    RouteNode* nextNode = node_rig->nexNode;

    double prevLeaveTime = prevNode->leaveTime;
    std::vector<double> prevLoadStatus = prevNode->loadStatus;

    double distanceDelta = 0.0;
    distanceDelta -= vrpSolution.vrpProblem->vrpData.disMatrix[prevNode->nodeId][node_lef->nodeId];
    distanceDelta -= vrpSolution.vrpProblem->vrpData.disMatrix[node_rig->nodeId][nextNode->nodeId];
    distanceDelta += vrpSolution.vrpProblem->vrpData.disMatrix[prevNode->nodeId][node_rig->nodeId];
    distanceDelta += vrpSolution.vrpProblem->vrpData.disMatrix[node_lef->nodeId][nextNode->nodeId];

    // double Previous_Tw_violation = vrpRoute.twViolation - prevNode->forwardTwPenaltySlack;
    // double Previous_Capacity_violation = vrpRoute.endNode->accumulatedCapacityViolation - prevNode->accumulatedCapacityViolation;
        
    double Tw_violation = prevNode->forwardTwPenaltySlack;
    double Capacity_violation = prevNode->accumulatedCapacityViolation;
    
    RouteNode* currentNode = node_rig;

    while(currentNode != node_lef->preNode)
    {
        int currentNodeId = currentNode->nodeId;
        double arcDistance = vrpSolution.vrpProblem->vrpData.disMatrix[prevNode->nodeId][currentNodeId];
        double currentArriveTime = prevLeaveTime + arcDistance;

        const VrpNode& nodeData = vrpSolution.vrpProblem->vrpData.nodeList[currentNodeId];

        Tw_violation += std::max(0.0, currentArriveTime - nodeData.timeWindow.second);
        for (int i = 0; i < vrpSolution.vrpProblem->vrpData.numScenarios; ++i) {
            prevLoadStatus[i] += nodeData.demand[i];
            Capacity_violation += std::max(0.0, prevLoadStatus[i] - vrpSolution.vrpProblem->vrpData.vehicleCapacity);
        }

        double currentStartServiceTime = std::min(std::max(currentArriveTime, nodeData.timeWindow.first), nodeData.timeWindow.second);
        prevLeaveTime = currentStartServiceTime + nodeData.serveTime;

        prevNode = currentNode;
        currentNode = currentNode->preNode;
    }
    currentNode = nextNode;
    // while(currentNode != nullptr)
    // {
    //     int currentNodeId = currentNode->nodeId;
    //     double arcDistance = vrpSolution.vrpProblem->vrpData.disMatrix[nullptr == prevNode ? 0 : prevNode->nodeId][currentNodeId];
    //     double currentArriveTime = prevLeaveTime + arcDistance;

    //     const VrpNode& nodeData = vrpSolution.vrpProblem->vrpData.nodeList[currentNodeId];
    //     double currentLoad = prevLoad + nodeData.demand;

    //     Tw_violation += std::max(0.0, currentArriveTime - nodeData.timeWindow.second);
    //     Capacity_violation += std::max(0.0, currentLoad - vrpSolution.vrpProblem->vrpData.vehicleCapacity);

    //     double currentStartServiceTime = std::min(std::max(currentArriveTime, nodeData.timeWindow.first), nodeData.timeWindow.second);
    //     prevLeaveTime = currentStartServiceTime + nodeData.serveTime;
    //     prevLoad = currentLoad;

    //     prevNode = currentNode;
    //     currentNode = currentNode->preNode;
    // }

    // double arcDistance = vrpSolution.vrpProblem->vrpData.disMatrix[prevNode == nullptr ? 0 : prevNode->nodeId][0];
    // Tw_violation += std::max(0.0, prevLeaveTime + arcDistance - vrpSolution.vrpProblem->vrpData.nodeList[0].timeWindow.second);

    //double Tw_violation_Delta = Tw_violation - Previous_Tw_violation;
    //double Capacity_violation_Delta = Capacity_violation - Previous_Capacity_violation;

    double Tw_violation_Delta = Tw_violation + currentNode->backwardTwPenaltySlack +
    std::max(prevLeaveTime + vrpSolution.vrpProblem->vrpData.disMatrix[prevNode->nodeId][currentNode->nodeId] 
    - currentNode->latestStartServiceTime, 0.0) - vrpRoute.twViolation;

    double Capacity_violation_Delta = Capacity_violation - prevNode->accumulatedCapacityViolation;

    // printf("value = %.5f\n",distanceDelta + Capacity_violation_Delta * vrpConfig.capacityPenaltyCoeff +
    //            Tw_violation_Delta * vrpConfig.twPenaltyCoeff);
    // printf("v1 = %.5f\n",Capacity_violation_Delta);
    // printf("v2 = %.5f\n",Tw_violation_Delta);
    // printf("zzr = %.5f\n",vrpRoute.twViolation);
    // printf("sss = %.5f",Tw_violation + currentNode->backwardTwPenaltySlack + std::max(prevLeaveTime + vrpSolution.vrpProblem->vrpData.disMatrix[prevNode->nodeId][currentNode->nodeId] 
    // - currentNode->latestStartServiceTime, 0.0));

    // exit(0);
    return distanceDelta + Capacity_violation_Delta * vrpConfig.capacityPenaltyCoeff +
               Tw_violation_Delta * vrpConfig.twPenaltyCoeff;
}

void reverse_route(VrpSolution& vrpSolution, int routeId, RouteNode* node_lef, RouteNode* node_rig, bool updateData) {
    RouteNode* prevNode = node_lef->preNode;
    RouteNode* nextNode = node_rig->nexNode;
    RouteNode* currentNode = node_rig;
    while(currentNode != node_lef->preNode)
    {
        prevNode->nexNode = currentNode;
        prevNode = currentNode;
        currentNode = currentNode -> preNode;
    }
    node_lef->nexNode = nextNode;
    currentNode = node_lef->preNode;
    while(currentNode != nextNode)
    {
        (currentNode->nexNode)->preNode = currentNode;
        currentNode = currentNode -> nexNode;
    }
    if (updateData) {
        vrpSolution.updateRouteData(routeId, node_rig, node_lef);
    }
    vrpSolution.checkData();
}

// int tot=0;
bool search_2_opt(const VrpConfig& vrpConfig, VrpSolution& vrpSolution)
{
    std::vector<bool> vis;
    std::vector<int> nodeIds;
    vis.resize(vrpSolution.vrpProblem->vrpData.nodeNum);
    for (VrpRoute& route : vrpSolution.routes) 
    {
        if (route.nodeNum <= 4) continue;
        RouteNode* currentNode = route.startNode;
        nodeIds.clear();
        while (nullptr != currentNode) 
        {
            nodeIds.push_back(currentNode->nodeId);
            currentNode = currentNode->nexNode;
        }
        for(int k1 = 1; k1 < route.nodeNum - 1; k1++)
        {
            for(int k2 = k1; k2 < route.nodeNum - 1; k2++)
            {
                if(vis[vrpSolution.routeNodes[nodeIds[k2]].corresRouteNode->nodeId]) break;
                vis[vrpSolution.routeNodes[nodeIds[k2]].nodeId] = true;
                if(k2 == k1) continue;
                double cost_Delta = get_reversed_cost(vrpConfig, vrpSolution, 
                route.routeId, &vrpSolution.routeNodes[nodeIds[k1]], 
                &vrpSolution.routeNodes[nodeIds[k2]]);
                if(cost_Delta < -1e-5)
                {
                    double w1 = vrpSolution.getTotalObjectiveValue(vrpConfig);
                    for(int k = k1; k <= k2; k++) 
                        vis[vrpSolution.routeNodes[nodeIds[k]].nodeId] = false;
                    reverse_route(vrpSolution, route.routeId, &vrpSolution.routeNodes[nodeIds[k1]], 
                    &vrpSolution.routeNodes[nodeIds[k2]], true);
                    double w2 = vrpSolution.getTotalObjectiveValue(vrpConfig);
                    // assert(abs(w2-w1-cost_Delta)<1e-5);
                    // printf("yes!\n");
                    // printf("value = %.5f\n", cost_Delta);
                    // printf("%.5f %.5f %.5f\n",w1, w2, w2-w1);
                    // exit(0);
                    return true;
                }
            }
            for(int k2 = k1; k2 < route.nodeNum-1; k2++)
            {
                vis[vrpSolution.routeNodes[nodeIds[k2]].nodeId] = false;
            }
        }
    }
    return false;
}

bool outExchangeTwoRequests(const VrpConfig& vrpConfig, VrpSolution& vrpSolution, int firstPickupNodeId,
                            int firstDeliveryNodeId, int secondPickupNodeId, int secondDeliveryNodeId) {
    assert(firstPickupNodeId != secondPickupNodeId);
    assert(firstDeliveryNodeId != secondDeliveryNodeId);
    assert(vrpSolution.routeNodes[firstPickupNodeId].routeId == vrpSolution.routeNodes[firstDeliveryNodeId].routeId);
    assert(vrpSolution.routeNodes[secondPickupNodeId].routeId == vrpSolution.routeNodes[secondDeliveryNodeId].routeId);

    int firstRouteId = vrpSolution.routeNodes[firstPickupNodeId].routeId;
    int secondRouteId = vrpSolution.routeNodes[secondPickupNodeId].routeId;
    assert(firstRouteId != secondRouteId);

    double firstDistance = vrpSolution.routes[firstRouteId].distance;
    double firstTwViolation = vrpSolution.routes[firstRouteId].twViolation;
    double firstCapacityViolation = vrpSolution.routes[firstRouteId].endNode == nullptr
                                        ? 0.0
                                        : vrpSolution.routes[firstRouteId].endNode->accumulatedCapacityViolation;

    double secondDistance = vrpSolution.routes[secondRouteId].distance;
    double secondTwViolation = vrpSolution.routes[secondRouteId].twViolation;
    double secondCapacityViolation = vrpSolution.routes[secondRouteId].endNode == nullptr
                                         ? 0.0
                                         : vrpSolution.routes[secondRouteId].endNode->accumulatedCapacityViolation;

    vrpSolution.removeNode(firstRouteId, firstPickupNodeId, firstDeliveryNodeId, true);
    vrpSolution.removeNode(secondRouteId, secondPickupNodeId, secondDeliveryNodeId, true);

    double savingCostAfterRemoval = (firstDistance - vrpSolution.routes[firstRouteId].distance) +
                                    (secondDistance - vrpSolution.routes[secondRouteId].distance);
    savingCostAfterRemoval +=
        vrpConfig.twPenaltyCoeff * (firstTwViolation - vrpSolution.routes[firstRouteId].twViolation);
    savingCostAfterRemoval +=
        vrpConfig.twPenaltyCoeff * (secondTwViolation - vrpSolution.routes[secondRouteId].twViolation);
    savingCostAfterRemoval +=
        vrpConfig.capacityPenaltyCoeff *
        (firstCapacityViolation - (vrpSolution.routes[firstRouteId].endNode == nullptr
                                       ? 0.0
                                       : vrpSolution.routes[firstRouteId].endNode->accumulatedCapacityViolation));
    savingCostAfterRemoval +=
        vrpConfig.capacityPenaltyCoeff *
        (secondCapacityViolation - (vrpSolution.routes[secondRouteId].endNode == nullptr
                                        ? 0.0
                                        : vrpSolution.routes[secondRouteId].endNode->accumulatedCapacityViolation));

    std::vector<InsertionData> firstInsertions;
    calInsertionData(vrpSolution, &vrpSolution.routeNodes[firstPickupNodeId],
                     &vrpSolution.routeNodes[firstDeliveryNodeId], secondRouteId, firstInsertions, true);

    const InsertionData* firstBestInsertionData = nullptr;
    double firstMinInsertCost = std::numeric_limits<double>::max();
    for (const InsertionData& insertion : firstInsertions) {
        double tmpCost = insertion.deltaDistance;
        tmpCost += vrpConfig.twPenaltyCoeff * insertion.deltaTwPenalty;
        tmpCost += vrpConfig.capacityPenaltyCoeff * insertion.deltaCapacityViolations;

        if (tmpCost < firstMinInsertCost) {
            firstMinInsertCost = tmpCost;
            firstBestInsertionData = &insertion;
        }
    }

    std::vector<InsertionData> secondInsertions;
    calInsertionData(vrpSolution, &vrpSolution.routeNodes[secondPickupNodeId],
                     &vrpSolution.routeNodes[secondDeliveryNodeId], firstRouteId, secondInsertions, true);

    const InsertionData* secondBestInsertionData = nullptr;
    double secondMinInsertCost = std::numeric_limits<double>::max();
    for (const InsertionData& insertion : secondInsertions) {
        double tmpCost = insertion.deltaDistance;
        tmpCost += vrpConfig.twPenaltyCoeff * insertion.deltaTwPenalty;
        tmpCost += vrpConfig.capacityPenaltyCoeff * insertion.deltaCapacityViolations;

        if (tmpCost < secondMinInsertCost) {
            secondMinInsertCost = tmpCost;
            secondBestInsertionData = &insertion;
        }
    }

    if (firstBestInsertionData == nullptr || secondBestInsertionData == nullptr) {
        return false;
    }

    if (firstMinInsertCost + secondMinInsertCost >= savingCostAfterRemoval) {
        return false;
    }

    assert(firstBestInsertionData->routeId == secondRouteId);
    assert(secondBestInsertionData->routeId == firstRouteId);

    vrpSolution.insertNode(firstBestInsertionData->routeId, firstPickupNodeId, firstDeliveryNodeId,
                           firstBestInsertionData->pickupPrevNode, firstBestInsertionData->deliveryPrevNode, true);
    vrpSolution.insertNode(secondBestInsertionData->routeId, secondPickupNodeId, secondDeliveryNodeId,
                           secondBestInsertionData->pickupPrevNode, secondBestInsertionData->deliveryPrevNode, true);

    return true;
}

bool outExchangeMove(const VrpConfig& vrpConfig, VrpSolution& vrpSolution, int pickupNodeId, int deliveryNodeId) {
    assert(pickupNodeId > 0);
    assert(deliveryNodeId > 0);
    assert(pickupNodeId != deliveryNodeId);
    assert(vrpSolution.routeNodes[pickupNodeId].corresRouteNode->nodeId == deliveryNodeId);
    assert(vrpSolution.routeNodes[deliveryNodeId].corresRouteNode->nodeId == pickupNodeId);

    RouteNode* pickupNode = &vrpSolution.routeNodes[pickupNodeId];
    RouteNode* deliveryNode = &vrpSolution.routeNodes[deliveryNodeId];
    assert(pickupNode->corresRouteNode == deliveryNode);
    assert(deliveryNode->corresRouteNode == pickupNode);
    assert(pickupNode->routeId == deliveryNode->routeId);

    int currentRouteId = pickupNode->routeId;

    const std::vector<int>& adjacentList = vrpSolution.vrpProblem->adjacentList[pickupNodeId];

    int exchangeNeighborCounter = 0;
    VrpSolution newSolution(vrpSolution.vrpProblem);
    for (int adjacentNodeId : adjacentList) {
        if (0 == adjacentNodeId || adjacentNodeId == pickupNodeId || adjacentNodeId == deliveryNodeId) {
            continue;
        }
        if (vrpSolution.routeNodes[adjacentNodeId].routeId == currentRouteId) {
            continue;
        }
        if (Utils::VrpNodeType::kDelivery == vrpSolution.vrpProblem->vrpData.nodeList[adjacentNodeId].nodeType) {
            continue;
        }

        newSolution.copySolution(vrpSolution);
        bool tmpRes = outExchangeTwoRequests(vrpConfig, newSolution, pickupNodeId, deliveryNodeId, adjacentNodeId,
                                             newSolution.routeNodes[adjacentNodeId].corresRouteNode->nodeId);

        if (tmpRes) {
            vrpSolution.copySolution(newSolution);
            return true;
        } else {
            exchangeNeighborCounter += 1;
        }

        if (exchangeNeighborCounter > kExchangeNeighborNum) {
            break;
        }
    }

    return false;
}

bool outExchangeMove(const VrpConfig& vrpConfig, VrpSolution& vrpSolution) {
    std::vector<int> assignedPickupNodeIds;
    assignedPickupNodeIds.reserve(vrpSolution.vrpProblem->vrpData.nodeNum / 2);

    for (const VrpRoute& route : vrpSolution.routes) {
        if (route.nodeNum <= 0) {
            continue;
        }

        RouteNode* currentNode = route.startNode;
        while (nullptr != currentNode) {
            if (Utils::VrpNodeType::kPickup == vrpSolution.vrpProblem->vrpData.nodeList[currentNode->nodeId].nodeType) {
                assignedPickupNodeIds.push_back(currentNode->nodeId);
            }
            currentNode = currentNode->nexNode;
        }
    }

    while (!assignedPickupNodeIds.empty()) {
        int randIndex = GetRandInt(0, (int)assignedPickupNodeIds.size() - 1);

        int selectedAssignedPickupNodeId = assignedPickupNodeIds[randIndex];
        int selectedAssignedDeliveryNodeId =
            vrpSolution.routeNodes[selectedAssignedPickupNodeId].corresRouteNode->nodeId;

        bool tmpRes =
            outExchangeMove(vrpConfig, vrpSolution, selectedAssignedPickupNodeId, selectedAssignedDeliveryNodeId);

        if (tmpRes) {
            return true;
        } else {
            assignedPickupNodeIds[randIndex] = assignedPickupNodeIds.back();
            assignedPickupNodeIds.pop_back();
        }
    }

    return false;
}

void localSearch(const VrpConfig& vrpConfig, VrpSolution& vrpSolution) 
{
    int tot = 0;
    while (true) 
    {
        bool tmpRes = false;
        // tmpRes |= outRelocateMove(vrpConfig, vrpSolution);
        tmpRes |= search_2_opt(vrpConfig, vrpSolution);
        if (!tmpRes) break;
        ++tot; 
        if(tot > 100) break;
    }
    // while(true)
    // {
    //     if(!outRelocateMove(vrpConfig, vrpSolution)) break;
    //     tot++;
    //     if(tot > 30) break;
    // }
    // int tot=0;
    // while (true) 
    // {
    //     bool tmpRes = search_2_opt(vrpConfig, vrpSolution);
    //     if (tmpRes) continue; 
    //     else break;
    // } 
    // while (true) {
    //     bool tmpRes = outExchangeMove(vrpConfig, vrpSolution);
    //     if (tmpRes) {
    //         continue;
    //     } else {
    //         break;
    //     }
    // }
}