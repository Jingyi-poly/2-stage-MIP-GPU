#include "domain/solver/memetic/LargeNeighborhoodSearch.h"

#include <algorithm>

#include "domain/solver/memetic/insert/Insert.h"
#include "domain/solver/memetic/removal/Removal.h"

void insertEmptyRoute(VrpSolution& vrpSolution) {
    for (VrpRoute& vrpRoute : vrpSolution.routes) {
        if (0 < vrpRoute.nodeNum) {
            continue;
        }

        RouteNode* selectRouteNode = nullptr;
        for (int i = 1; i < (int)vrpSolution.routeNodes.size(); ++i) {
            RouteNode& routeNode = vrpSolution.routeNodes[i];
            if (routeNode.forwardTwPenaltySlack > 0.0 || routeNode.accumulatedCapacityViolation > 0.0) {
                selectRouteNode = &routeNode;
                break;
            }
        }

        if (nullptr == selectRouteNode) {
            continue;
        }

        int nodeId = selectRouteNode->nodeId;
        int pickupNodeId;
        int deliverNodeId;
        const VrpNode& nodeData = vrpSolution.vrpProblem->vrpData.nodeList[nodeId];
        if (Utils::VrpNodeType::kPickup == nodeData.nodeType) {
            pickupNodeId = nodeId;
            deliverNodeId = nodeData.corresNodeId;
        } else {
            pickupNodeId = nodeData.corresNodeId;
            deliverNodeId = nodeId;
        }
        vrpSolution.removeNode(selectRouteNode->routeId, pickupNodeId, deliverNodeId, true);
        vrpSolution.insertNode(vrpRoute.routeId, pickupNodeId, deliverNodeId, nullptr, nullptr, true);
    }
}

void largeNeighborhoodSearch(const VrpConfig& vrpConfig, VrpSolution& vrpSolution) {
    VrpSolution newSol(vrpSolution.vrpProblem);
    newSol.copySolution(vrpSolution);

    for (int i = 0; i < 50; ++i) {
        double tmpRandValue = GetRandDouble(0.0, 1.0);
        if (tmpRandValue < kRandomRemovalProb) {
            randomRemoval(vrpConfig, newSol);
        } else if (tmpRandValue < kRandomRemovalProb + kWorstRemovalProb) {
            worstRemoval(vrpConfig, newSol);
        } else if (tmpRandValue < kRandomRemovalProb + kWorstRemovalProb + kShawRemovalProb) {
            shawRemoval(vrpConfig, newSol);
        } else {
            stringRemoval(vrpConfig, newSol);
        }
        greedyInsertWithBlinks(vrpConfig, newSol, true);
        insertEmptyRoute(newSol);

        if (newSol.getTotalObjectiveValue(vrpConfig) < vrpSolution.getTotalObjectiveValue(vrpConfig)) {
            vrpSolution.copySolution(newSol);
        } else {
            newSol.copySolution(vrpSolution);
        }
    }
}