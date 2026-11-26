#include "domain/solver/memetic/Perturb.h"

#include <cassert>

#include "domain/solver/memetic/insert/Insert.h"
#include "util/random_utils.h"

int selectInsertion(const VrpConfig& vrpConfig, const std::vector<InsertionData>& candidateInsertions) {
    std::vector<double> insertionCosts;
    insertionCosts.reserve(candidateInsertions.size());
    double insertionCostsSum = 0.0;
    for (const InsertionData& insertion : candidateInsertions) {
        double tmpCost = insertion.deltaDistance;
        tmpCost += insertion.deltaTwPenalty * vrpConfig.twPenaltyCoeff;
        tmpCost += insertion.deltaCapacityViolations * vrpConfig.capacityPenaltyCoeff;

        insertionCostsSum += tmpCost;
        insertionCosts.push_back(tmpCost);
    }

    double tmpRandValue = GetRandDouble(0.0, 1.0) * insertionCostsSum;
    double tmpAccumulateCost = 0.0;
    int selectedInsertionIndex = -1;
    for (int i = 0; i < (int)insertionCosts.size(); ++i) {
        tmpAccumulateCost += insertionCosts[i];
        if (tmpAccumulateCost >= tmpRandValue) {
            selectedInsertionIndex = i;
            break;
        }
    }

    if (-1 == selectedInsertionIndex) {
        selectedInsertionIndex = (int)insertionCosts.size() - 1;
    }

    return selectedInsertionIndex;
}

void randomMove(const VrpConfig& vrpConfig, VrpSolution& vrpSolution) {
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
    if (assignedPickupNodeIds.empty()) return;

    int randIndex = GetRandInt(0, (int)assignedPickupNodeIds.size() - 1);
    int firstPickupNodeId = assignedPickupNodeIds[randIndex];
    int firstDeliveryNodeId = vrpSolution.routeNodes[firstPickupNodeId].corresRouteNode->nodeId;
    int firstRouteId = vrpSolution.routeNodes[firstPickupNodeId].routeId;
    assert(firstRouteId == vrpSolution.routeNodes[firstDeliveryNodeId].routeId);

    // EAX can legitimately produce a child with every request in one route.
    // Perturbation is a two-route swap, so there is no valid second pickup in
    // that state.  The former route-node arithmetic passed an empty range to
    // GetRandInt and then indexed routeNodes[-1] in release builds.  Select from
    // the actual pickups on other routes and make this move a no-op when none
    // exist; the caller can still keep or reject the child normally.
    std::vector<int> otherRoutePickupNodeIds;
    otherRoutePickupNodeIds.reserve(assignedPickupNodeIds.size());
    for (int pickupNodeId : assignedPickupNodeIds) {
        if (vrpSolution.routeNodes[pickupNodeId].routeId != firstRouteId) {
            otherRoutePickupNodeIds.push_back(pickupNodeId);
        }
    }
    if (otherRoutePickupNodeIds.empty()) return;

    randIndex = GetRandInt(0, static_cast<int>(otherRoutePickupNodeIds.size()) - 1);
    int secondPickupNodeId = otherRoutePickupNodeIds[randIndex];
    int secondDeliveryNodeId = vrpSolution.routeNodes[secondPickupNodeId].corresRouteNode->nodeId;
    int secondRouteId = vrpSolution.routeNodes[secondPickupNodeId].routeId;
    assert(secondRouteId == vrpSolution.routeNodes[secondDeliveryNodeId].routeId);
    assert(firstRouteId != secondRouteId);

    vrpSolution.removeNode(firstRouteId, firstPickupNodeId, firstDeliveryNodeId, true);
    vrpSolution.removeNode(secondRouteId, secondPickupNodeId, secondDeliveryNodeId, true);

    std::vector<InsertionData> insertions;
    calInsertionData(vrpSolution, &vrpSolution.routeNodes[firstPickupNodeId],
                     &vrpSolution.routeNodes[firstDeliveryNodeId], secondRouteId, insertions, true);

    int selectedInsertionIndex = selectInsertion(vrpConfig, insertions);
    vrpSolution.insertNode(secondRouteId, firstPickupNodeId, firstDeliveryNodeId,
                           insertions[selectedInsertionIndex].pickupPrevNode,
                           insertions[selectedInsertionIndex].deliveryPrevNode, true);

    insertions.clear();
    calInsertionData(vrpSolution, &vrpSolution.routeNodes[secondPickupNodeId],
                     &vrpSolution.routeNodes[secondDeliveryNodeId], firstRouteId, insertions, true);
    selectedInsertionIndex = selectInsertion(vrpConfig, insertions);
    vrpSolution.insertNode(firstRouteId, secondPickupNodeId, secondDeliveryNodeId,
                           insertions[selectedInsertionIndex].pickupPrevNode,
                           insertions[selectedInsertionIndex].deliveryPrevNode, true);
}

void perturb(const VrpConfig& vrpConfig, VrpSolution& vrpSolution) {
    for (int i = 0; i < kRandomMoveNumInPerturb; ++i) {
        randomMove(vrpConfig, vrpSolution);
    }
}
