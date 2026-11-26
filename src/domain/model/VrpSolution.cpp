#include "domain/model/VrpSolution.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "domain/eval/ScenarioCapacityEvaluator.h"
#include "domain/model/entity/VrpNode.h"

VrpSolution::VrpSolution(VrpProblem* problem) {
    vrpProblem = problem;
    preSol = nullptr;
    nexSol = nullptr;
    vehicleNum = 0;
    totalDistance = 0.0;
    feasibility = Feasibility::INFEASIBLE;
    for (int i = 0; i < vrpProblem->vrpData.nodeNum; ++i) {
        routeNodes.emplace_back(i, vrpProblem->vrpData.numScenarios);
    }
    for (int i = 1; i < vrpProblem->vrpData.nodeNum; ++i) {
        int corresNodeId = vrpProblem->vrpData.nodeList[i].corresNodeId;
        routeNodes[i].corresRouteNode = &routeNodes[corresNodeId];
    }
    totalTwViolation = 0.0;
    totalCapacityViolation = 0.0;
    totalPickupDeliveryPrecedenceViolation = 0.0;
    totalPickupDeliveryPairingViolation = 0.0;
    for (int pickupNodeId : vrpProblem->vrpData.pickNodeIds) {
        unassignedNodeIds.emplace_back(pickupNodeId, vrpProblem->vrpData.nodeList[pickupNodeId].corresNodeId);
    }
    totalValueValid = true;
}

void VrpSolution::removeRoute(int routeId) {
    totalValueValid = false;

    RouteNode* currentNode = routes[routeId].startNode;

    while (nullptr != currentNode) {
        int nodeId = currentNode->nodeId;
        const VrpNode& node = vrpProblem->vrpData.nodeList[nodeId];
        if (Utils::VrpNodeType::kPickup == node.nodeType) {
            unassignedNodeIds.emplace_back(nodeId, node.corresNodeId);
        }

        RouteNode* currentNodeBak = currentNode->nexNode;

        currentNode->clearData();
        currentNode = currentNodeBak;
    }

    routes[routeId].clearData();

    checkData();
}

void VrpSolution::removeNode(int routeId, int pickupNodeId, int deliveryNodeId, bool updateData) {
    totalValueValid = false;

    VrpRoute& route = routes[routeId];
    RouteNode& pickupNode = routeNodes[pickupNodeId];
    RouteNode& deliveryNode = routeNodes[deliveryNodeId];

    RouteNode* pickupPrevNode = pickupNode.preNode;
    RouteNode* pickupNextNode = pickupNode.nexNode;
    int pickupRouteId = pickupNode.routeId;
    assert(pickupRouteId == routeId);

    RouteNode* deliveryPrevNode = deliveryNode.preNode;
    RouteNode* deliveryNextNode = deliveryNode.nexNode;
    int deliveryRouteId = deliveryNode.routeId;
    assert(deliveryRouteId == routeId);

    if (nullptr == pickupPrevNode) {
        if (pickupNextNode == &deliveryNode) {
            if (nullptr == deliveryNextNode) {
                route.clearData();
            } else {
                route.startNode = deliveryNextNode;
                deliveryNextNode->preNode = nullptr;
            }
        } else {
            route.startNode = pickupNextNode;
            pickupNextNode->preNode = nullptr;

            deliveryPrevNode->nexNode = deliveryNextNode;
            if (nullptr == deliveryNextNode) {
                route.endNode = deliveryPrevNode;
            } else {
                deliveryNextNode->preNode = deliveryPrevNode;
            }
        }
    } else {
        if (pickupNextNode == &deliveryNode) {
            pickupPrevNode->nexNode = deliveryNextNode;
            if (nullptr == deliveryNextNode) {
                route.endNode = pickupPrevNode;
            } else {
                deliveryNextNode->preNode = pickupPrevNode;
            }
        } else {
            pickupPrevNode->nexNode = pickupNextNode;
            pickupNextNode->preNode = pickupPrevNode;

            deliveryPrevNode->nexNode = deliveryNextNode;
            if (nullptr == deliveryNextNode) {
                route.endNode = deliveryPrevNode;
            } else {
                deliveryNextNode->preNode = deliveryPrevNode;
            }
        }
    }

    if (0 < route.nodeNum) {
        route.nodeNum -= 2;
    }

    pickupNode.clearData();
    deliveryNode.clearData();
    unassignedNodeIds.emplace_back(pickupNodeId, deliveryNodeId);

    if (updateData && route.nodeNum) {
        if (nullptr == pickupPrevNode) {
            if (nullptr == deliveryNextNode) {
                updateRouteData(routeId, route.startNode, route.endNode);
            } else {
                updateRouteData(routeId, route.startNode,
                                deliveryNextNode->preNode == nullptr ? route.startNode : deliveryNextNode->preNode);
            }
        } else {
            if (nullptr == deliveryNextNode) {
                updateRouteData(routeId, pickupPrevNode->nexNode == nullptr ? route.endNode : pickupPrevNode->nexNode,
                                route.endNode);
            } else {
                updateRouteData(routeId, pickupPrevNode->nexNode, deliveryNextNode->preNode);
            }
        }
    }

    checkData();
}

void VrpSolution::updateRouteData(int routeId, RouteNode* forwardStartNode, RouteNode* backwardStartNode) {
    checkData();
    totalValueValid = false;

    const bool useGpuCapacity = vrpProblem->useGpuCapacityEval && vrpProblem->capacityEvaluator != nullptr;

    VrpRoute& route = routes[routeId];

    RouteNode* currentNode = route.startNode;
    RouteNode* prevNode = nullptr;
    double prevLeaveTime = vrpProblem->vrpData.nodeList[0].timeWindow.first + vrpProblem->vrpData.nodeList[0].serveTime;
    double prevForwardTwPenaltySlack = 0.0;
    std::vector<double> prevLoadStatus = std::vector<double>(vrpProblem->vrpData.numScenarios, 0.0);
    double prevAccumulatedCapacityViolations = 0.0;
    double prevAccumulatedDistance = 0.0;

    if (route.startNode != forwardStartNode) {
        currentNode = forwardStartNode;
        prevNode = currentNode->preNode;
        prevLeaveTime = prevNode->leaveTime;
        prevForwardTwPenaltySlack = prevNode->forwardTwPenaltySlack;
        std::copy_n(prevNode->loadStatus.begin(), vrpProblem->vrpData.numScenarios, prevLoadStatus.begin());
        prevAccumulatedCapacityViolations = prevNode->accumulatedCapacityViolation;
        prevAccumulatedDistance = prevNode->accumulatedDistance;
    }

    double arcDistance;
    double latestStartTime;
    while (nullptr != currentNode) {
        const VrpNode& vrpNode = vrpProblem->vrpData.nodeList[currentNode->nodeId];
        arcDistance = vrpProblem->vrpData.disMatrix[prevNode == nullptr ? 0 : prevNode->nodeId][currentNode->nodeId];
        currentNode->accumulatedDistance = prevAccumulatedDistance + arcDistance;
        currentNode->arriveTime = prevLeaveTime + arcDistance;

        latestStartTime = vrpNode.timeWindow.second;
        currentNode->startServiceTime =
            std::min(std::max(currentNode->arriveTime, vrpNode.timeWindow.first), latestStartTime);
        currentNode->leaveTime = currentNode->startServiceTime + vrpNode.serveTime;
        currentNode->forwardTwPenaltySlack =
            prevForwardTwPenaltySlack + std::max(0.0, currentNode->arriveTime - latestStartTime);

        // Per-scenario capacity is recomputed for the whole route by the GPU
        // evaluator below; skip the inline CPU loop in that mode.
        if (!useGpuCapacity) {
            currentNode->accumulatedCapacityViolation = prevAccumulatedCapacityViolations;
            for (int i = 0; i < vrpProblem->vrpData.numScenarios; ++i) {
                currentNode->loadStatus[i] = prevLoadStatus[i] + vrpNode.demand[i];
                currentNode->accumulatedCapacityViolation +=
                    std::max(0.0, currentNode->loadStatus[i] - vrpProblem->vrpData.vehicleCapacity);
            }
        }

        currentNode->routeId = routeId;

        prevNode = currentNode;
        prevLeaveTime = currentNode->leaveTime;
        if (!useGpuCapacity) {
            std::copy_n(currentNode->loadStatus.begin(), vrpProblem->vrpData.numScenarios, prevLoadStatus.begin());
            assert(std::all_of(currentNode->loadStatus.begin(), currentNode->loadStatus.end(),
                               [](double load) { return load >= 0.0; }));
            prevAccumulatedCapacityViolations = currentNode->accumulatedCapacityViolation;
        }
        prevForwardTwPenaltySlack = currentNode->forwardTwPenaltySlack;
        prevAccumulatedDistance = currentNode->accumulatedDistance;

        currentNode = currentNode->nexNode;
    }

    arcDistance = vrpProblem->vrpData.disMatrix[prevNode == nullptr ? 0 : prevNode->nodeId][0];
    route.distance = prevAccumulatedDistance + arcDistance;
    route.twViolation = prevForwardTwPenaltySlack +
                        std::max(0.0, prevLeaveTime + arcDistance - vrpProblem->vrpData.nodeList[0].timeWindow.second);

    currentNode = route.endNode;
    RouteNode* nextNode = nullptr;
    double nextLatestStartServiceTime = vrpProblem->vrpData.nodeList[0].timeWindow.second;
    double nextBackTwPenaltySlack = 0.0;

    if (route.endNode != backwardStartNode) {
        currentNode = backwardStartNode;
        nextNode = currentNode->nexNode;
        nextLatestStartServiceTime = nextNode->latestStartServiceTime;
        nextBackTwPenaltySlack = nextNode->backwardTwPenaltySlack;
    }

    double earliestStartTime;
    while (nullptr != currentNode) {
        const VrpNode& currentNodeData = vrpProblem->vrpData.nodeList[currentNode->nodeId];
        arcDistance = vrpProblem->vrpData.disMatrix[currentNode->nodeId][nextNode == nullptr ? 0 : nextNode->nodeId];

        latestStartTime = nextLatestStartServiceTime - arcDistance - currentNodeData.serveTime;
        earliestStartTime = currentNodeData.timeWindow.first;
        currentNode->latestStartServiceTime =
            std::max(earliestStartTime, std::min(latestStartTime, currentNodeData.timeWindow.second));

        currentNode->backwardTwPenaltySlack =
            nextBackTwPenaltySlack + std::max(0.0, earliestStartTime - latestStartTime);

        assert(std::fabs(currentNode->backwardTwPenaltySlack + currentNode->forwardTwPenaltySlack +
                         std::max(0.0, currentNode->startServiceTime - currentNode->latestStartServiceTime) -
                         route.twViolation) < 1E-4);
        currentNode->routeId = routeId;

        nextNode = currentNode;
        nextLatestStartServiceTime = currentNode->latestStartServiceTime;
        nextBackTwPenaltySlack = currentNode->backwardTwPenaltySlack;

        currentNode = currentNode->preNode;
    }

    if (useGpuCapacity && route.startNode != nullptr) {
        // Capacity violation depends only on the prefix of demands (no time
        // coupling), so recomputing the full route here yields exactly the same
        // per-node loadStatus / accumulatedCapacityViolation as the inline CPU
        // pass, regardless of which segment was structurally updated.
        const int numScenarios = vrpProblem->vrpData.numScenarios;
        RouteBatch batch;
        batch.routeOffsets.push_back(0);
        for (RouteNode* node = route.startNode; node != nullptr; node = node->nexNode) {
            batch.nodeIds.push_back(node->nodeId);
        }
        batch.routeOffsets.push_back(static_cast<int>(batch.nodeIds.size()));

        std::vector<double> loadsOut;
        std::vector<double> accViolOut;
        vrpProblem->capacityEvaluator->Evaluate(batch, loadsOut, accViolOut);

        int k = 0;
        for (RouteNode* node = route.startNode; node != nullptr; node = node->nexNode, ++k) {
            std::copy_n(loadsOut.begin() + static_cast<size_t>(k) * numScenarios, numScenarios,
                        node->loadStatus.begin());
            node->accumulatedCapacityViolation = accViolOut[k];
        }
    }

#ifndef NDEBUG
    double latestStartServiceTimeAtDepot = route.startNode->latestStartServiceTime -
                                           vrpProblem->vrpData.disMatrix[0][route.startNode->nodeId] -
                                           vrpProblem->vrpData.nodeList[0].serveTime;
    double backwardTwPenaltySlackAtDepot =
        route.startNode->backwardTwPenaltySlack +
        std::max(0.0, vrpProblem->vrpData.nodeList[0].timeWindow.first - latestStartServiceTimeAtDepot);

    assert(std::fabs(backwardTwPenaltySlackAtDepot - route.twViolation) < 1E-4);

    currentNode = route.startNode;
    while (currentNode != nullptr) {
        double tmpForward = currentNode->forwardTwPenaltySlack;
        double tmpBackward = currentNode->backwardTwPenaltySlack;
        double tmpPenalty = tmpForward + tmpBackward +
                            std::max(0.0, currentNode->startServiceTime - currentNode->latestStartServiceTime);
        assert(std::fabs(tmpPenalty - route.twViolation) < 1E-4);

        currentNode = currentNode->nexNode;
    }
#endif

    checkData();
}

void VrpSolution::insertNode(int routeId, int pickupNodeId, int deliveryNodeId, RouteNode* pickupPreNode,
                             RouteNode* deliveryPreNode, bool updateData) {
    totalValueValid = false;

    RouteNode* pickupNextNode;
    RouteNode* deliveryNextNode;

    if (nullptr == pickupPreNode) {
        pickupNextNode = routes[routeId].startNode;
    } else {
        pickupNextNode = pickupPreNode->nexNode;
    }

    if (nullptr == deliveryPreNode) {
        deliveryNextNode = routes[routeId].startNode;
    } else {
        deliveryNextNode = deliveryPreNode->nexNode;
    }

    RouteNode* pickupNode = &routeNodes[pickupNodeId];
    RouteNode* deliveryNode = &routeNodes[deliveryNodeId];
    assert(0 > pickupNode->routeId);
    assert(0 > deliveryNode->routeId);
    assert(nullptr == pickupNode->preNode);
    assert(nullptr == pickupNode->nexNode);
    assert(nullptr == deliveryNode->preNode);
    assert(nullptr == deliveryNode->nexNode);

    if (nullptr == pickupPreNode) {
        if (nullptr == deliveryPreNode) {
            routes[routeId].startNode = pickupNode;
            pickupNode->preNode = nullptr;
            pickupNode->nexNode = deliveryNode;
            deliveryNode->preNode = pickupNode;
        } else {
            routes[routeId].startNode = pickupNode;
            pickupNode->preNode = nullptr;

            pickupNode->nexNode = pickupNextNode;
            pickupNextNode->preNode = pickupNode;

            deliveryPreNode->nexNode = deliveryNode;
            deliveryNode->preNode = deliveryPreNode;
        }
    } else {
        if (pickupPreNode == deliveryPreNode) {
            pickupPreNode->nexNode = pickupNode;
            pickupNode->preNode = pickupPreNode;
            pickupNode->nexNode = deliveryNode;
            deliveryNode->preNode = pickupNode;
        } else {
            pickupPreNode->nexNode = pickupNode;
            pickupNode->preNode = pickupPreNode;

            pickupNode->nexNode = pickupNextNode;
            pickupNextNode->preNode = pickupNode;

            deliveryPreNode->nexNode = deliveryNode;
            deliveryNode->preNode = deliveryPreNode;
        }
    }

    if (nullptr == deliveryNextNode) {
        routes[routeId].endNode = deliveryNode;
        deliveryNode->nexNode = nullptr;
    } else {
        deliveryNode->nexNode = deliveryNextNode;
        deliveryNextNode->preNode = deliveryNode;
    }

    routes[routeId].nodeNum += 2;
    pickupNode->routeId = routeId;
    deliveryNode->routeId = routeId;

    for (int i = 0; i < unassignedNodeIds.size();) {
        if (pickupNodeId == unassignedNodeIds[i].first) {
            assert(deliveryNodeId == unassignedNodeIds[i].second);
            unassignedNodeIds[i] = unassignedNodeIds.back();
            unassignedNodeIds.pop_back();
            break;
        } else {
            ++i;
        }
    }

    if (updateData) {
        updateRouteData(routeId, pickupNode, deliveryNode);
    }

    checkData();
}

void VrpSolution::copyRoute(const VrpSolution& fromSol, int fromRouteId, int toRouteId) {
    VrpRoute& toRoute = routes[toRouteId];
    const VrpRoute& fromRoute = fromSol.routes[fromRouteId];

    assert(0 >= toRoute.nodeNum);
    assert(nullptr == toRoute.startNode);
    assert(nullptr == toRoute.endNode);

    totalValueValid = false;
    toRoute = fromRoute;
    toRoute.routeId = toRouteId;
    if (nullptr != fromRoute.startNode) {
        assert(0 < fromRoute.nodeNum);
        toRoute.startNode = &routeNodes[fromRoute.startNode->nodeId];
        toRoute.endNode = &routeNodes[fromRoute.endNode->nodeId];
    } else {
        toRoute.startNode = nullptr;
        toRoute.endNode = nullptr;
    }

    RouteNode* currentNode = fromRoute.startNode;
    int currentNodeId;
    while (nullptr != currentNode) {
        currentNodeId = currentNode->nodeId;
        routeNodes[currentNodeId] = fromSol.routeNodes[currentNodeId];
        routeNodes[currentNodeId].routeId = toRouteId;
        routeNodes[currentNodeId].preNode = fromSol.routeNodes[currentNodeId].preNode == nullptr
                                                ? nullptr
                                                : &routeNodes[fromSol.routeNodes[currentNodeId].preNode->nodeId];
        routeNodes[currentNodeId].nexNode = fromSol.routeNodes[currentNodeId].nexNode == nullptr
                                                ? nullptr
                                                : &routeNodes[fromSol.routeNodes[currentNodeId].nexNode->nodeId];
        routeNodes[currentNodeId].corresRouteNode =
            &routeNodes[vrpProblem->vrpData.nodeList[currentNodeId].corresNodeId];

        currentNode = currentNode->nexNode;
    }

    for (int i = 0; i < (int)unassignedNodeIds.size();) {
        if (0 <= routeNodes[unassignedNodeIds[i].first].routeId) {
            assert(routeNodes[unassignedNodeIds[i].first].routeId == routeNodes[unassignedNodeIds[i].second].routeId);
            unassignedNodeIds[i] = unassignedNodeIds.back();
            unassignedNodeIds.pop_back();
        } else {
            ++i;
        }
    }

    checkData();
}

void VrpSolution::copySolution(const VrpSolution& fromSol) {
    vrpProblem = fromSol.vrpProblem;
    preSol = fromSol.preSol;
    nexSol = fromSol.nexSol;

    vehicleNum = fromSol.vehicleNum;
    totalDistance = fromSol.totalDistance;
    for (RouteNode& routeNode : routeNodes) {
        routeNode.clearData();
    }
    routes.clear();
    for (int i = 0; i < (int)fromSol.routes.size(); ++i) {
        assert(i == fromSol.routes[i].routeId);
        routes.emplace_back(fromSol.routes[i].routeId);
        copyRoute(fromSol, i, i);
    }
    feasibility = fromSol.feasibility;
    twViolations = fromSol.twViolations;
    totalTwViolation = fromSol.totalTwViolation;
    capacityViolations = fromSol.capacityViolations;
    totalCapacityViolation = fromSol.totalCapacityViolation;
    totalPickupDeliveryPrecedenceViolation = fromSol.totalPickupDeliveryPrecedenceViolation;
    totalPickupDeliveryPairingViolation = fromSol.totalPickupDeliveryPairingViolation;
    unassignedNodeIds = fromSol.unassignedNodeIds;
    totalValueValid = fromSol.totalValueValid;

    checkData();
}

void VrpSolution::calTotalValue() {
    totalDistance = 0.0;
    totalCapacityViolation = 0.0;
    totalTwViolation = 0.0;

    for (const VrpRoute& route : routes) {
        if (0 >= route.nodeNum) {
            continue;
        }
        totalDistance += route.distance;
        assert(nullptr != route.endNode);
        totalCapacityViolation += route.endNode->accumulatedCapacityViolation;
        totalTwViolation += route.twViolation;
    }

    totalValueValid = true;
}

double VrpSolution::getTotalDistance() {
    if (totalValueValid) {
        return totalDistance;
    }
    calTotalValue();

    return totalDistance;
}

double VrpSolution::getTotalTwViolation() {
    if (totalValueValid) {
        return totalTwViolation;
    }
    calTotalValue();

    return totalTwViolation;
}

double VrpSolution::getTotalCapacityViolation() {
    if (totalValueValid) {
        return totalCapacityViolation;
    }
    calTotalValue();

    return totalCapacityViolation;
}

double VrpSolution::getTotalObjectiveValue(const VrpConfig& vrpConfig) {
    double objectiveValue = 0.0;
    objectiveValue += vrpConfig.vehicleCost * (double)routes.size();
    objectiveValue += vrpConfig.twPenaltyCoeff * getTotalTwViolation();
    objectiveValue += vrpConfig.capacityPenaltyCoeff * getTotalCapacityViolation();
    objectiveValue += getTotalDistance();

    return objectiveValue;
}

double VrpSolution::getRouteHash() {
    double hash = 0.0;
    for (const VrpRoute& route : routes) {
        if (0 >= route.nodeNum) {
            continue;
        }
        hash += route.startNode->nodeId * kRouteHashFactor;
    }

    return hash;
}

bool VrpSolution::isSameWith(VrpSolution& otherSol) {
    if (routes.size() != otherSol.routes.size()) {
        return false;
    }

    if (std::fabs(getTotalDistance() - otherSol.getTotalDistance()) > 1E-4) {
        return false;
    }

    if (std::fabs(getTotalCapacityViolation() - otherSol.getTotalCapacityViolation()) > 1E-4) {
        return false;
    }

    if (std::fabs(getTotalTwViolation() - otherSol.getTotalTwViolation()) > 1E-4) {
        return false;
    }

    if (std::fabs(getRouteHash() - otherSol.getRouteHash()) > 1E-4) {
        return false;
    }

    return true;
}

void VrpSolution::checkData() {
#ifndef NDEBUG
    for (const VrpRoute& route : routes) {
        if (0 >= route.nodeNum) {
            assert(nullptr == route.startNode);
            assert(nullptr == route.endNode);
        }

        if (nullptr == route.startNode) {
            assert(nullptr == route.endNode);
            assert(0 == route.nodeNum);
        }
        if (nullptr == route.endNode) {
            assert(nullptr == route.startNode);
            assert(0 == route.nodeNum);
        }
        if (0 >= route.nodeNum) {
            continue;
        }

        int tmpNodeNum = 0;
        RouteNode* currentNode = route.startNode;
        while (nullptr != currentNode) {
            tmpNodeNum++;
            currentNode = currentNode->nexNode;
        }
        assert(tmpNodeNum == route.nodeNum);

        assert(nullptr != route.startNode);
        assert(nullptr != route.endNode);

        int startNodeId = route.startNode->nodeId;
        int endNodeId = route.endNode->nodeId;

        assert(route.startNode == &routeNodes[startNodeId]);
        assert(route.endNode == &routeNodes[endNodeId]);

        assert(route.startNode->preNode == nullptr);
        assert(route.endNode->nexNode == nullptr);
    }

    for (const RouteNode& routeNode : routeNodes) {
        RouteNode* preNode = routeNode.preNode;
        RouteNode* nextNode = routeNode.nexNode;

        if (nullptr != preNode) {
            assert(preNode->nexNode == &routeNode);
            assert(preNode == &routeNodes[preNode->nodeId]);
        } else {
            if (0 > routeNode.routeId) {
                assert(nullptr == preNode);
                assert(nullptr == nextNode);
            } else {
                assert(&routeNode == routes[routeNode.routeId].startNode);
            }
        }

        if (nullptr != nextNode) {
            assert(nextNode->preNode == &routeNode);
            assert(nextNode == &routeNodes[nextNode->nodeId]);
        }

        if (0 != routeNode.nodeId) {
            assert(routeNode.corresRouteNode ==
                   &routeNodes[vrpProblem->vrpData.nodeList[routeNode.nodeId].corresNodeId]);
        }
    }
#endif
}