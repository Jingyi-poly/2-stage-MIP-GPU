#include "domain/solver/memetic/crossover/SrexCrossover.h"

#include <algorithm>
#include <cassert>
#include <numeric>
#include <unordered_set>

#include "domain/solver/memetic/LargeNeighborhoodSearch.h"
#include "domain/solver/memetic/LocalSearch.h"
#include "domain/solver/memetic/Perturb.h"
#include "domain/solver/memetic/crossover/RouteSubset.h"
#include "domain/solver/memetic/insert/Insert.h"
#include "domain/solver/memetic/removal/Removal.h"

void srexCrossover(const VrpConfig& vrpConfig, VrpSolution& firstSol, VrpSolution& secondSol,
                   std::chrono::steady_clock::time_point startTime, double timeLimit, VrpSolution& childSol) {
    VrpSolution newFirstSol(firstSol.vrpProblem);
    newFirstSol.copySolution(firstSol);

    if (secondSol.isSameWith(newFirstSol)) {
        perturb(vrpConfig, newFirstSol);

        if (secondSol.isSameWith(newFirstSol)) {
            childSol.copySolution(newFirstSol);
            return;
        }
    }
    std::vector<RouteSubset> routeSubsets;
    for (int i = 0; i < kSrexRouteSubsetNum; ++i) {
        routeSubsets.emplace_back(newFirstSol, secondSol);

        RouteSubset& routeSubset = routeSubsets[i];
        routeSubset.SelectRandomInitialRoutes();
        routeSubset.SearchNeighborhood();
        routeSubset.clearLcsValues();
        routeSubset.clearArcSequence();
    }

    std::vector<int> routeSubsetIndices(kSrexRouteSubsetNum, 0);
    std::iota(routeSubsetIndices.begin(), routeSubsetIndices.end(), 0);
    routeSubsetIndices.erase(std::remove_if(routeSubsetIndices.begin(), routeSubsetIndices.end(),
                                            [&](int routeSubsetIndex) {
                                                return routeSubsets[routeSubsetIndex].first_route_indices_.size() ==
                                                       newFirstSol.routes.size();
                                            }),
                             routeSubsetIndices.end());

    if (routeSubsetIndices.empty()) {
        childSol.copySolution(newFirstSol);
        return;
    }
    std::sort(routeSubsetIndices.begin(), routeSubsetIndices.end(),
              [&routeSubsets](int a, int b) { return routeSubsets[a].lcs_value_ > routeSubsets[b].lcs_value_; });

    for (int i = 1; i < (int)routeSubsetIndices.size();) {
        if (routeSubsets[routeSubsetIndices[i]].IsDuplicatedWith(routeSubsets[routeSubsetIndices[i - 1]])) {
            routeSubsetIndices.erase(routeSubsetIndices.begin() + i);
        } else {
            ++i;
        }
    }

    routeSubsetIndices.resize(std::min((int)routeSubsetIndices.size(), kCrossoverChildSolNum));

    bool firstCandidateChild = true;
    std::chrono::steady_clock::time_point currentTime;
    for (int routeSubsetIndex : routeSubsetIndices) {
        const RouteSubset& routeSubset = routeSubsets[routeSubsetIndex];

        std::unordered_set<int> nodesInFirstRouteSubset;
        std::unordered_set<int> nodesInSecondRouteSubset;
        std::vector<int> nodesInFirstNotInSecond;
        std::vector<int> nodesInSecondNotInFirst;

        for (int routeId : routeSubset.first_route_indices_) {
            RouteNode* currentNode = newFirstSol.routes[routeId].startNode;
            while (nullptr != currentNode) {
                nodesInFirstRouteSubset.insert(currentNode->nodeId);
                currentNode = currentNode->nexNode;
            }
        }

        for (int routeId : routeSubset.second_route_indices_) {
            RouteNode* currentNode = secondSol.routes[routeId].startNode;
            while (nullptr != currentNode) {
                int currentNodeId = currentNode->nodeId;
                if (nodesInFirstRouteSubset.find(currentNodeId) == nodesInFirstRouteSubset.end()) {
                    nodesInSecondNotInFirst.push_back(currentNodeId);
                }
                nodesInSecondRouteSubset.insert(currentNode->nodeId);
                currentNode = currentNode->nexNode;
            }
        }

        for (int nodeId : nodesInFirstRouteSubset) {
            if (nodesInSecondRouteSubset.find(nodeId) == nodesInSecondRouteSubset.end()) {
                nodesInFirstNotInSecond.push_back(nodeId);
            }
        }

        VrpSolution firstChildSol(newFirstSol.vrpProblem);
        firstChildSol.copySolution(newFirstSol);

        for (int routeId : routeSubset.first_route_indices_) {
            firstChildSol.removeRoute(routeId);
        }

        std::unordered_set<int> affectedRouteIndices;
        for (int nodeId : nodesInSecondNotInFirst) {
            const VrpNode& node = firstChildSol.vrpProblem->vrpData.nodeList[nodeId];
            if (Utils::VrpNodeType::kPickup == node.nodeType) {
                int tmpRouteId = firstChildSol.routeNodes[nodeId].routeId;
                affectedRouteIndices.insert(tmpRouteId);
                firstChildSol.removeNode(tmpRouteId, nodeId, node.corresNodeId, false);
            }
        }

        for (int routeId : affectedRouteIndices) {
            if (0 >= firstChildSol.routes[routeId].nodeNum) {
                firstChildSol.routes[routeId].clearData();
            } else {
                assert(nullptr != firstChildSol.routes[routeId].startNode);
                assert(nullptr != firstChildSol.routes[routeId].endNode);
                firstChildSol.updateRouteData(routeId, firstChildSol.routes[routeId].startNode,
                                              firstChildSol.routes[routeId].endNode);
            }
        }

        assert(routeSubset.first_route_indices_.size() == routeSubset.second_route_indices_.size());

        for (size_t i = 0; i < routeSubset.first_route_indices_.size(); ++i) {
            int firstRouteId = routeSubset.first_route_indices_[i];
            int secondRouteId = routeSubset.second_route_indices_[i];

            firstChildSol.copyRoute(secondSol, secondRouteId, firstRouteId);
        }

        greedyInsertWithBlinks(vrpConfig, firstChildSol, true);
        insertEmptyRoute(firstChildSol);
        localSearch(vrpConfig, firstChildSol);
        largeNeighborhoodSearch(vrpConfig, firstChildSol);
        insertEmptyRoute(firstChildSol);
        localSearch(vrpConfig, firstChildSol);

        VrpSolution secondChildSol(newFirstSol.vrpProblem);
        secondChildSol.copySolution(newFirstSol);

        for (int routeId : routeSubset.first_route_indices_) {
            secondChildSol.removeRoute(routeId);
        }

        for (size_t i = 0; i < routeSubset.first_route_indices_.size(); ++i) {
            int firstRouteId = routeSubset.first_route_indices_[i];
            int secondRouteId = routeSubset.second_route_indices_[i];

            bool completeRoute = true;

            const VrpRoute& routeInSecondSol = secondSol.routes[secondRouteId];

            RouteNode* firstNode = nullptr;
            RouteNode* prevNode = nullptr;
            RouteNode* lastSameWithSecondSolNode = nullptr;

            RouteNode* currentNode = routeInSecondSol.startNode;
            assert(secondChildSol.routes[firstRouteId].nodeNum == 0);
            while (nullptr != currentNode) {
                int currentNodeId = currentNode->nodeId;
                if (nodesInFirstRouteSubset.find(currentNodeId) == nodesInFirstRouteSubset.end()) {
                    completeRoute = false;
                    currentNode = currentNode->nexNode;
                    continue;
                } else {
                    RouteNode* nodeInSecondChildSol = &secondChildSol.routeNodes[currentNodeId];
                    if (nullptr == firstNode) {
                        firstNode = nodeInSecondChildSol;
                    }
                    secondChildSol.routeNodes[currentNodeId] = secondSol.routeNodes[currentNodeId];
                    secondChildSol.routeNodes[currentNodeId].routeId = firstRouteId;
                    secondChildSol.routeNodes[currentNodeId].corresRouteNode =
                        &secondChildSol
                             .routeNodes[newFirstSol.vrpProblem->vrpData.nodeList[currentNodeId].corresNodeId];
                    nodeInSecondChildSol->preNode = prevNode;
                    secondChildSol.routes[firstRouteId].nodeNum += 1;
                    if (nullptr != prevNode) {
                        prevNode->nexNode = nodeInSecondChildSol;
                    }

                    if (completeRoute) {
                        lastSameWithSecondSolNode = nodeInSecondChildSol;
                    }

                    prevNode = nodeInSecondChildSol;
                    currentNode = currentNode->nexNode;
                }
            }

            if (nullptr != firstNode) {
                secondChildSol.routes[firstRouteId].startNode = firstNode;
                secondChildSol.routes[firstRouteId].endNode = prevNode;
                secondChildSol.routes[firstRouteId].endNode->nexNode = nullptr;

                if (completeRoute) {
                    secondChildSol.routes[firstRouteId].clearData();
                    secondChildSol.copyRoute(secondSol, secondRouteId, firstRouteId);
                } else {
                    secondChildSol.updateRouteData(
                        firstRouteId,
                        lastSameWithSecondSolNode == nullptr            ? secondChildSol.routes[firstRouteId].startNode
                        : lastSameWithSecondSolNode->nexNode == nullptr ? secondChildSol.routes[firstRouteId].endNode
                                                                        : lastSameWithSecondSolNode->nexNode,
                        secondChildSol.routes[firstRouteId].endNode);
                }
            } else {
                secondChildSol.routes[firstRouteId].clearData();
            }
        }

        for (int i = 0; i < (int)secondChildSol.unassignedNodeIds.size();) {
            if (0 <= secondChildSol.routeNodes[secondChildSol.unassignedNodeIds[i].first].routeId) {
                assert(secondChildSol.routeNodes[secondChildSol.unassignedNodeIds[i].first].routeId ==
                       secondChildSol.routeNodes[secondChildSol.unassignedNodeIds[i].second].routeId);
                secondChildSol.unassignedNodeIds[i] = secondChildSol.unassignedNodeIds.back();
                secondChildSol.unassignedNodeIds.pop_back();
            } else {
                ++i;
            }
        }
        greedyInsertWithBlinks(vrpConfig, secondChildSol, true);
        insertEmptyRoute(secondChildSol);
        localSearch(vrpConfig, secondChildSol);
        largeNeighborhoodSearch(vrpConfig, secondChildSol);
        insertEmptyRoute(secondChildSol);
        localSearch(vrpConfig, secondChildSol);

        bool firstChildBetter = true;
        double firstChildObjectiveValue = firstChildSol.getTotalObjectiveValue(vrpConfig);
        double secondChildObjectiveValue = secondChildSol.getTotalObjectiveValue(vrpConfig);
        if (firstChildObjectiveValue > secondChildObjectiveValue) {
            firstChildBetter = false;
        }

        if (firstCandidateChild) {
            if (firstChildBetter) {
                childSol.copySolution(firstChildSol);
            } else {
                childSol.copySolution(secondChildSol);
            }
        } else {
            if (firstChildBetter) {
                if (childSol.getTotalObjectiveValue(vrpConfig) > firstChildObjectiveValue) {
                    childSol.copySolution(firstChildSol);
                }
            } else {
                if (childSol.getTotalObjectiveValue(vrpConfig) > secondChildObjectiveValue) {
                    childSol.copySolution(secondChildSol);
                }
            }
        }
        firstCandidateChild = false;

        currentTime = std::chrono::steady_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
        if (duration > timeLimit) {
            break;
        }
    }
}