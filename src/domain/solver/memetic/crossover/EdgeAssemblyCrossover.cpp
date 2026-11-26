#include "domain/solver/memetic/crossover/EdgeAssemblyCrossover.h"

#include <algorithm>
#include <cassert>
#include <unordered_map>
#include <unordered_set>

#include "domain/solver/memetic/LargeNeighborhoodSearch.h"
#include "domain/solver/memetic/LocalSearch.h"
#include "domain/solver/memetic/Perturb.h"
#include "domain/solver/memetic/insert/Insert.h"
#include "util/random_utils.h"

void eaxInitGraph(const VrpSolution& sol, EaxGraph& graph, uint8_t source) {
    int nodeNum = sol.vrpProblem->vrpData.nodeNum;
    int routeNum = (int)sol.routes.size();

    graph.edges.resize(nodeNum - 1 + routeNum);
    graph.nodeHeadEdge.resize(nodeNum, nullptr);
    graph.nodeTailEdge.resize(nodeNum, nullptr);
    graph.depotHeadEdges.reserve(routeNum);
    graph.depotTailEdges.reserve(routeNum);

    int edgeIndex = 0;
    for (const VrpRoute& route : sol.routes) {
        if (0 >= route.nodeNum) {
            continue;
        }
        RouteNode* preNode = nullptr;
        RouteNode* currentNode = route.startNode;
        while (nullptr != currentNode) {
            EaxEdge& edge = graph.edges[edgeIndex];
            edge.head = (preNode == nullptr ? 0 : preNode->nodeId);
            edge.tail = currentNode->nodeId;
            edge.source = source;
            edge.index = edgeIndex;
            preNode = currentNode;
            currentNode = currentNode->nexNode;

            if (0 == edge.head) {
                graph.depotHeadEdges.push_back(&edge);
            } else {
                graph.nodeHeadEdge[edge.head] = &edge;
            }

            graph.nodeTailEdge[edge.tail] = &edge;
            ++edgeIndex;
        }

        graph.edges[edgeIndex].head = (preNode == nullptr ? 0 : preNode->nodeId);
        graph.edges[edgeIndex].tail = 0;
        graph.edges[edgeIndex].source = source;
        graph.edges[edgeIndex].index = edgeIndex;

        if (0 == graph.edges[edgeIndex].head) {
            graph.depotHeadEdges.push_back(&graph.edges[edgeIndex]);
        } else {
            graph.nodeHeadEdge[graph.edges[edgeIndex].head] = &graph.edges[edgeIndex];
        }

        graph.depotTailEdges.push_back(&graph.edges[edgeIndex]);

        ++edgeIndex;
    }

    assert(graph.edges.size() == edgeIndex);
    graph.edges.resize(edgeIndex);
}

bool calGraphCircle(EaxGraph& firstGraph, EaxGraph& secondGraph, GraphCircle& graphCircle) {
    graphCircle.graphDiffEdges.reserve(firstGraph.edges.size() + secondGraph.edges.size());
    graphCircle.firstGraphEdgeDiffFlags.resize(firstGraph.edges.size(), 0);
    graphCircle.secondGraphEdgeDiffFlags.resize(secondGraph.edges.size(), 0);

    std::vector<const EaxEdge*>& graphDiffEdges = graphCircle.graphDiffEdges;
    std::vector<int>& circleStart = graphCircle.circleStart;
    std::vector<uint8_t>& firstGraphEdgeDiffFlags = graphCircle.firstGraphEdgeDiffFlags;
    std::vector<uint8_t>& secondGraphEdgeDiffFlags = graphCircle.secondGraphEdgeDiffFlags;

    for (const EaxEdge& edge : firstGraph.edges) {
        int head = edge.head;
        int tail = edge.tail;

        if (0 == head) {
            int headInSecond = secondGraph.nodeTailEdge[tail]->head;
            if (0 != headInSecond) {
                graphDiffEdges.push_back(&edge);
                firstGraphEdgeDiffFlags[edge.index] = 1;
            }
        } else if (0 == tail) {
            int tailInSecond = secondGraph.nodeHeadEdge[head]->tail;
            if (0 != tailInSecond) {
                graphDiffEdges.push_back(&edge);
                firstGraphEdgeDiffFlags[edge.index] = 1;
            }
        } else {
            int tailInSecond = secondGraph.nodeHeadEdge[head]->tail;
            if (tail != tailInSecond) {
                graphDiffEdges.push_back(&edge);
                firstGraphEdgeDiffFlags[edge.index] = 1;
            }
        }
    }

    for (const EaxEdge& edge : secondGraph.edges) {
        int head = edge.head;
        int tail = edge.tail;

        if (0 == head) {
            int headInFirst = firstGraph.nodeTailEdge[tail]->head;
            if (0 != headInFirst) {
                graphDiffEdges.push_back(&edge);
                secondGraphEdgeDiffFlags[edge.index] = 1;
            }
        } else if (0 == tail) {
            int tailInFirst = firstGraph.nodeHeadEdge[head]->tail;
            if (0 != tailInFirst) {
                graphDiffEdges.push_back(&edge);
                secondGraphEdgeDiffFlags[edge.index] = 1;
            }
        } else {
            int tailInFirst = firstGraph.nodeHeadEdge[head]->tail;
            if (tail != tailInFirst) {
                graphDiffEdges.push_back(&edge);
                secondGraphEdgeDiffFlags[edge.index] = 1;
            }
        }
    }

    std::shuffle(graphDiffEdges.begin(), graphDiffEdges.end(), GetRandomGenerator());
    std::unordered_map<const EaxEdge*, int> edgePtrIndexMap;
    for (int i = 0; i < (int)graphDiffEdges.size(); ++i) {
        edgePtrIndexMap[graphDiffEdges[i]] = i;
    }

    int circleStartEdgeIndex = 0;
    while (circleStartEdgeIndex < (int)graphDiffEdges.size()) {
        circleStart.push_back(circleStartEdgeIndex);

        const EaxEdge* prevEdge = graphDiffEdges[circleStartEdgeIndex];
        int currentCircleHead = prevEdge->head;
        int currentCircleTail = prevEdge->tail;

        bool currentIn = true;
        int currentCircleStartNode = currentCircleHead;
        int currentCircleEndNode = currentCircleTail;
        int firstEdgeSource = prevEdge->source;

        if (0 == prevEdge->source) {
            firstGraphEdgeDiffFlags[prevEdge->index] = 0;
        } else {
            secondGraphEdgeDiffFlags[prevEdge->index] = 0;
        }

        while (true) {
            circleStartEdgeIndex += 1;
            currentIn = !currentIn;
            EaxEdge* nextEdge = nullptr;
            if (0 == currentCircleEndNode) {
                std::vector<EaxEdge*> candidateEdgeIndices;
                candidateEdgeIndices.reserve(firstGraph.depotHeadEdges.size());

                if (0 == prevEdge->source) {
                    if (currentIn) {
                        for (EaxEdge* tmpEdge : secondGraph.depotHeadEdges) {
                            if (secondGraphEdgeDiffFlags[tmpEdge->index]) {
                                candidateEdgeIndices.push_back(tmpEdge);
                            }
                        }
                    } else {
                        for (EaxEdge* tmpEdge : secondGraph.depotTailEdges) {
                            if (secondGraphEdgeDiffFlags[tmpEdge->index]) {
                                candidateEdgeIndices.push_back(tmpEdge);
                            }
                        }
                    }
                } else {
                    if (currentIn) {
                        for (EaxEdge* tmpEdge : firstGraph.depotHeadEdges) {
                            if (firstGraphEdgeDiffFlags[tmpEdge->index]) {
                                candidateEdgeIndices.push_back(tmpEdge);
                            }
                        }
                    } else {
                        for (EaxEdge* tmpEdge : firstGraph.depotTailEdges) {
                            if (firstGraphEdgeDiffFlags[tmpEdge->index]) {
                                candidateEdgeIndices.push_back(tmpEdge);
                            }
                        }
                    }
                }

                if (candidateEdgeIndices.empty()) {
                    // Degenerate AB-cycle structure (can arise when parents use a
                    // different number of (non-empty) routes, e.g. with spare
                    // vehicles). Abort so the caller can fall back to a parent copy.
                    return false;
                }
                int tmpRandIndex = GetRandInt(0, (int)candidateEdgeIndices.size() - 1);
                nextEdge = candidateEdgeIndices[tmpRandIndex];
            } else {
                if (0 == prevEdge->source) {
                    nextEdge = currentIn ? secondGraph.nodeHeadEdge[currentCircleEndNode]
                                         : secondGraph.nodeTailEdge[currentCircleEndNode];
                    assert(secondGraphEdgeDiffFlags[nextEdge->index] == 1);
                } else {
                    nextEdge = currentIn ? firstGraph.nodeHeadEdge[currentCircleEndNode]
                                         : firstGraph.nodeTailEdge[currentCircleEndNode];
                    assert(firstGraphEdgeDiffFlags[nextEdge->index] == 1);
                }
            }

            int tmpIndex = edgePtrIndexMap[nextEdge];
            assert(graphDiffEdges[tmpIndex] == nextEdge);
            std::swap(graphDiffEdges[circleStartEdgeIndex], graphDiffEdges[tmpIndex]);
            edgePtrIndexMap[nextEdge] = circleStartEdgeIndex;
            edgePtrIndexMap[graphDiffEdges[tmpIndex]] = tmpIndex;
            if (0 == nextEdge->source) {
                firstGraphEdgeDiffFlags[nextEdge->index] = 0;
            } else {
                secondGraphEdgeDiffFlags[nextEdge->index] = 0;
            }
            prevEdge = nextEdge;
            currentCircleEndNode = currentIn ? nextEdge->tail : nextEdge->head;

            if (currentCircleEndNode == currentCircleStartNode && !currentIn && prevEdge->source != firstEdgeSource) {
                break;
            }
        }
        circleStartEdgeIndex += 1;
    }

    assert(circleStartEdgeIndex == (int)graphDiffEdges.size());
    circleStart.push_back(circleStartEdgeIndex);
    return true;
}

void calInterGraph(const VrpSolution& firstSol, const EaxGraph& firstGraph, const GraphCircle& graphCircle,
                   std::unordered_set<const EaxEdge*>& interGraphEdges,
                   std::vector<const EaxEdge*>& interGraphNodeHeadEdges,
                   std::vector<const EaxEdge*>& interGraphNodeTailEdges,
                   std::vector<const EaxEdge*>& interGraphDepotHeadEdges,
                   std::vector<const EaxEdge*>& interGraphDepotTailEdges) {
    interGraphEdges.reserve(firstGraph.edges.size());
    interGraphNodeHeadEdges.resize(firstSol.vrpProblem->vrpData.nodeNum, nullptr);
    interGraphNodeTailEdges.resize(firstSol.vrpProblem->vrpData.nodeNum, nullptr);
    interGraphDepotHeadEdges.reserve(firstSol.routes.size());
    interGraphDepotTailEdges.reserve(firstSol.routes.size());

    int circleNum = (int)graphCircle.circleStart.size() - 1;
    int centerCircleIndex = GetRandInt(0, circleNum - 1);
    std::vector<int> circleIndices = {centerCircleIndex};
    double tmpRandDouble = GetRandDouble(0.0, 1.0);
    if (tmpRandDouble < kEaxCircleSelectedBlockStrategyProb) {
        std::unordered_set<int> customerNodesInCenterCircle;
        int nodeNumInCenterCircle =
            graphCircle.circleStart[centerCircleIndex + 1] - graphCircle.circleStart[centerCircleIndex];
        for (int i = graphCircle.circleStart[centerCircleIndex]; i < graphCircle.circleStart[centerCircleIndex + 1];
             ++i) {
            int tmpHead = graphCircle.graphDiffEdges[i]->head;
            int tmpTail = graphCircle.graphDiffEdges[i]->tail;
            if (0 != tmpHead) {
                customerNodesInCenterCircle.insert(tmpHead);
            }
            if (0 != tmpTail) {
                customerNodesInCenterCircle.insert(tmpTail);
            }
        }

        for (int i = 0; i < circleNum; ++i) {
            if (i == centerCircleIndex) {
                continue;
            }

            int circleNodeNum = graphCircle.circleStart[i + 1] - graphCircle.circleStart[i];
            if (circleNodeNum >= nodeNumInCenterCircle) {
                continue;
            }

            bool shareCustomerNode = false;
            for (int j = graphCircle.circleStart[i]; j < graphCircle.circleStart[i + 1]; ++j) {
                int tmpHead = graphCircle.graphDiffEdges[j]->head;
                int tmpTail = graphCircle.graphDiffEdges[j]->tail;

                if (0 != tmpHead && customerNodesInCenterCircle.find(tmpHead) != customerNodesInCenterCircle.end()) {
                    shareCustomerNode = true;
                    break;
                }
                if (0 != tmpTail && customerNodesInCenterCircle.find(tmpTail) != customerNodesInCenterCircle.end()) {
                    shareCustomerNode = true;
                    break;
                }
            }

            if (shareCustomerNode) {
                circleIndices.push_back(i);
            }
        }
    }

    std::vector<const EaxEdge*> eSetGraphEdges;
    eSetGraphEdges.reserve(graphCircle.circleStart.back());
    std::unordered_map<int, std::unordered_set<const EaxEdge*>> eSetNodeHeadEdgeMap;
    std::unordered_map<int, std::unordered_set<const EaxEdge*>> eSetNodeTailEdgeMap;
    std::unordered_set<const EaxEdge*> eSetDepotHeadEdges;
    std::unordered_set<const EaxEdge*> eSetDepotTailEdges;

    for (int circleIndex : circleIndices) {
        for (int i = graphCircle.circleStart[circleIndex]; i < graphCircle.circleStart[circleIndex + 1]; ++i) {
            const EaxEdge* tmpEdge = graphCircle.graphDiffEdges[i];
            int tmpHead = tmpEdge->head;
            int tmpTail = tmpEdge->tail;

            if (0 == tmpHead) {
                eSetDepotHeadEdges.insert(tmpEdge);
            } else {
                eSetNodeHeadEdgeMap[tmpHead].insert(tmpEdge);
            }

            if (0 == tmpTail) {
                eSetDepotTailEdges.insert(tmpEdge);
            } else {
                eSetNodeTailEdgeMap[tmpTail].insert(tmpEdge);
            }

            eSetGraphEdges.push_back(tmpEdge);
        }
    }

    for (const EaxEdge& edge : firstGraph.edges) {
        int tmpHead = edge.head;
        int tmpTail = edge.tail;

        if (0 == tmpHead) {
            if (eSetDepotHeadEdges.find(&edge) == eSetDepotHeadEdges.end()) {
                interGraphEdges.insert(&edge);
                interGraphDepotHeadEdges.push_back(&edge);
            }
        } else {
            if (eSetNodeHeadEdgeMap.find(tmpHead) == eSetNodeHeadEdgeMap.end() ||
                eSetNodeHeadEdgeMap[tmpHead].find(&edge) == eSetNodeHeadEdgeMap[tmpHead].end()) {
                interGraphEdges.insert(&edge);
                assert(nullptr == interGraphNodeHeadEdges[edge.head]);
                interGraphNodeHeadEdges[edge.head] = &edge;
            }
        }

        if (0 == tmpTail) {
            if (eSetDepotTailEdges.find(&edge) == eSetDepotTailEdges.end()) {
                interGraphEdges.insert(&edge);
                interGraphDepotTailEdges.push_back(&edge);
            }
        } else {
            if (eSetNodeTailEdgeMap.find(tmpTail) == eSetNodeTailEdgeMap.end() ||
                eSetNodeTailEdgeMap[tmpTail].find(&edge) == eSetNodeTailEdgeMap[tmpTail].end()) {
                interGraphEdges.insert(&edge);
                assert(nullptr == interGraphNodeTailEdges[edge.tail]);
                interGraphNodeTailEdges[edge.tail] = &edge;
            }
        }
    }

    for (const EaxEdge* edge : eSetGraphEdges) {
        if (1 == edge->source) {
            int tmpHead = edge->head;
            int tmpTail = edge->tail;
            interGraphEdges.insert(edge);

            if (0 == tmpHead) {
                interGraphDepotHeadEdges.push_back(edge);
            } else {
                assert(nullptr == interGraphNodeHeadEdges[tmpHead]);
                interGraphNodeHeadEdges[tmpHead] = edge;
            }

            if (0 == tmpTail) {
                interGraphDepotTailEdges.push_back(edge);
            } else {
                assert(nullptr == interGraphNodeTailEdges[tmpTail]);
                interGraphNodeTailEdges[tmpTail] = edge;
            }
        }
    }
}

void constructInterSol(std::unordered_set<const EaxEdge*>& interGraphEdges,
                       std::vector<const EaxEdge*>& interGraphNodeHeadEdges,
                       std::vector<const EaxEdge*>& interGraphNodeTailEdges,
                       std::vector<const EaxEdge*>& interGraphDepotHeadEdges,
                       std::vector<const EaxEdge*>& interGraphDepotTailEdges, VrpSolution& interSol,
                       std::vector<std::vector<int>>& subtours) {
    int routeIndex = 0;
    std::unordered_set<const EaxEdge*> edgeInRouteSet;

    for (RouteNode& routeNode : interSol.routeNodes) {
        routeNode.clearData();
    }

    interSol.routes.clear();

    for (const EaxEdge* depotHeadEdge : interGraphDepotHeadEdges) {
        interSol.routes.emplace_back(routeIndex);
        RouteNode* prevNode = nullptr;
        RouteNode* currentNode = &interSol.routeNodes[depotHeadEdge->tail];
        edgeInRouteSet.insert(depotHeadEdge);

        while (nullptr != currentNode) {
            interSol.routes[routeIndex].nodeNum += 1;

            currentNode->preNode = prevNode;
            if (nullptr != prevNode) {
                prevNode->nexNode = currentNode;
            } else {
                interSol.routes[routeIndex].startNode = currentNode;
            }

            currentNode->routeId = routeIndex;
            prevNode = currentNode;
            int currentNodeId = currentNode->nodeId;
            const EaxEdge* nextEdge = interGraphNodeHeadEdges[currentNodeId];
            edgeInRouteSet.insert(nextEdge);
            if (0 == nextEdge->tail) {
                currentNode->nexNode = nullptr;
                interSol.routes[routeIndex].endNode = currentNode;
                break;
            } else {
                currentNode = &interSol.routeNodes[nextEdge->tail];
            }
        }

        routeIndex += 1;
    }

    for (int i = routeIndex; i < (int)interSol.routes.size(); ++i) {
        interSol.routes[i].clearData();
    }

    std::unordered_set<const EaxEdge*> edgeInSubtourSet;
    for (const EaxEdge* edge : interGraphEdges) {
        if (edgeInRouteSet.find(edge) != edgeInRouteSet.end()) {
            continue;
        }

        if (edgeInSubtourSet.find(edge) != edgeInSubtourSet.end()) {
            continue;
        }

        int subtourHead = edge->head;
        int subtourTail = edge->tail;

        assert(0 != subtourHead);
        assert(0 != subtourTail);

        std::vector<int> subtour;
        subtour.push_back(subtourHead);
        edgeInSubtourSet.insert(edge);
        while (subtourTail != subtourHead) {
            subtour.push_back(subtourTail);

            const EaxEdge* nextEdge = interGraphNodeHeadEdges[subtourTail];
            subtourTail = nextEdge->tail;

            edgeInSubtourSet.insert(nextEdge);
        }

        subtours.push_back(std::move(subtour));
    }

    interSol.unassignedNodeIds.clear();
    std::unordered_set<int> unassignedNodeSet;
    for (VrpRoute& route : interSol.routes) {
        std::unordered_set<int> unpairedNodes;
        RouteNode* currentNode = route.startNode;
        while (nullptr != currentNode) {
            int currentNodeId = currentNode->nodeId;

            const VrpNode& node = interSol.vrpProblem->vrpData.nodeList[currentNodeId];
            if (Utils::VrpNodeType::kPickup == node.nodeType) {
                unpairedNodes.insert(currentNodeId);
            } else {
                int corresNodeId = node.corresNodeId;
                if (unpairedNodes.find(corresNodeId) != unpairedNodes.end()) {
                    unpairedNodes.erase(corresNodeId);
                } else {
                    unpairedNodes.insert(currentNodeId);
                }
            }

            currentNode = currentNode->nexNode;
        }

        for (int unpairedNode : unpairedNodes) {
            RouteNode& routeNode = interSol.routeNodes[unpairedNode];
            if (nullptr == routeNode.preNode) {
                route.startNode = routeNode.nexNode;
                if (nullptr == routeNode.nexNode) {
                    route.startNode = nullptr;
                    route.endNode = nullptr;
                } else {
                    route.startNode->preNode = nullptr;
                }
            } else {
                routeNode.preNode->nexNode = routeNode.nexNode;
                if (nullptr == routeNode.nexNode) {
                    route.endNode = routeNode.preNode;
                } else {
                    routeNode.nexNode->preNode = routeNode.preNode;
                }
            }

            routeNode.clearData();
            route.nodeNum -= 1;
            const VrpNode& node = interSol.vrpProblem->vrpData.nodeList[unpairedNode];

            if (Utils::VrpNodeType::kPickup == node.nodeType &&
                unassignedNodeSet.find(unpairedNode) == unassignedNodeSet.end()) {
                unassignedNodeSet.insert(unpairedNode);
                interSol.unassignedNodeIds.emplace_back(unpairedNode, node.corresNodeId);
            }
        }
    }

    for (std::vector<int>& subtour : subtours) {
        for (int nodeIdInSubtour : subtour) {
            const VrpNode& node = interSol.vrpProblem->vrpData.nodeList[nodeIdInSubtour];

            if (Utils::VrpNodeType::kPickup == node.nodeType &&
                unassignedNodeSet.find(nodeIdInSubtour) == unassignedNodeSet.end()) {
                unassignedNodeSet.insert(nodeIdInSubtour);
                interSol.unassignedNodeIds.emplace_back(nodeIdInSubtour, node.corresNodeId);
            }
        }

        std::unordered_set<int> unpairedNodes;

        for (int nodeId : subtour) {
            const VrpNode& node = interSol.vrpProblem->vrpData.nodeList[nodeId];
            int corresNodeId = node.corresNodeId;
            if (unpairedNodes.find(corresNodeId) != unpairedNodes.end()) {
                unpairedNodes.erase(corresNodeId);
            } else {
                unpairedNodes.insert(nodeId);
            }
        }

        for (int i = 0; i < subtour.size();) {
            int nodeId = subtour[i];
            if (unpairedNodes.find(nodeId) == unpairedNodes.end()) {
                i += 1;
                continue;
            } else {
                std::swap(subtour[i], subtour.back());
                subtour.pop_back();
            }
        }
    }

    subtours.erase(
        std::remove_if(subtours.begin(), subtours.end(), [](const auto& subtour) { return subtour.empty(); }),
        subtours.end());

    for (VrpRoute& route : interSol.routes) {
        if (0 == route.nodeNum) {
            route.clearData();
            continue;
        }
        interSol.updateRouteData(route.routeId, route.startNode, route.endNode);
    }
}

void calSubtourInsertionData(const VrpSolution& sol, const std::vector<int>& subtour, int breakPos, int routeId,
                             std::vector<SubtourInsertionData>& insertions) {
    const VrpRoute& route = sol.routes[routeId];

    std::vector<double> prevLoad = std::vector<double>(sol.vrpProblem->vrpData.numScenarios, 0.0);
    if (0 == route.nodeNum) {
        insertions.emplace_back();
        SubtourInsertionData& insertionData = insertions.back();
        insertionData.routeId = routeId;
        insertionData.prevNodeId = 0;
        insertionData.newVehicle = true;

        int prevNodeId = 0;
        double prevLeaveTime = sol.vrpProblem->vrpData.nodeList[0].timeWindow.first;
        double prevForwardTwPenaltySlack = 0.0;

        double prevAccumulatedCapacityViolations = 0.0;
        double prevAccumulatedDistance = 0.0;

        double arcDistance;
        double arriveTime;
        double startServiceTime;
        for (int i = breakPos; i < subtour.size() + breakPos; ++i) {
            int currentNodeId = i < subtour.size() ? subtour[i] : subtour[i - subtour.size()];
            const VrpNode& node = sol.vrpProblem->vrpData.nodeList[currentNodeId];
            arcDistance = sol.vrpProblem->vrpData.disMatrix[prevNodeId][currentNodeId];

            arriveTime = prevLeaveTime + arcDistance;
            startServiceTime = std::min(std::max(arriveTime, node.timeWindow.first), node.timeWindow.second);

            prevNodeId = currentNodeId;
            prevLeaveTime = startServiceTime + node.serveTime;
            for (int i = 0; i < sol.vrpProblem->vrpData.numScenarios; ++i) {
                prevLoad[i] += node.demand[i];
                prevAccumulatedCapacityViolations += std::max(0.0, prevLoad[i] - sol.vrpProblem->vrpData.vehicleCapacity);
            }
            prevForwardTwPenaltySlack += std::max(0.0, arriveTime - node.timeWindow.second);
            prevAccumulatedDistance += arcDistance;
        }

        arcDistance = sol.vrpProblem->vrpData.disMatrix[prevNodeId][0];
        arriveTime = prevLeaveTime + arcDistance;
        prevForwardTwPenaltySlack += std::max(0.0, arriveTime - sol.vrpProblem->vrpData.nodeList[0].timeWindow.second);
        prevAccumulatedDistance += arcDistance;

        insertionData.deltaCapacityViolations = prevAccumulatedCapacityViolations;
        insertionData.deltaTwPenalty = prevForwardTwPenaltySlack;
        insertionData.deltaDistance = prevAccumulatedDistance;

        return;
    }

    int insertPrevNodeId = 0;
    while (true) {
        int nextNodeId = 0;
        if (0 == insertPrevNodeId) {
            nextNodeId = sol.routes[routeId].startNode->nodeId;
        } else {
            RouteNode* tmpNextNode = sol.routeNodes[insertPrevNodeId].nexNode;
            if (nullptr != tmpNextNode) {
                nextNodeId = tmpNextNode->nodeId;
            }
        }

        int subtourFirstNodeId = subtour[breakPos];
        int subtourLastNodeId = (0 == breakPos ? subtour.back() : subtour[breakPos - 1]);

        double prevLeaveTime = sol.vrpProblem->vrpData.nodeList[0].timeWindow.first;
        double prevForwardTwPenaltySlack = 0.0;

        double prevAccumulatedCapacityViolations = 0.0;
        double prevAccumulatedDistance = 0.0;

        if (0 != insertPrevNodeId) {
            const RouteNode& tmpNode = sol.routeNodes[insertPrevNodeId];
            prevLeaveTime = tmpNode.leaveTime;
            prevForwardTwPenaltySlack = tmpNode.forwardTwPenaltySlack;
            std::copy_n(tmpNode.loadStatus.begin(), sol.vrpProblem->vrpData.numScenarios, prevLoad.begin());
            prevAccumulatedCapacityViolations = tmpNode.accumulatedCapacityViolation;
            prevAccumulatedDistance = tmpNode.accumulatedDistance;
        }

        double arcDistance;
        double arriveTime;
        double startServiceTime;
        int prevNodeId = insertPrevNodeId;
        for (int i = breakPos; i < subtour.size() + breakPos; ++i) {
            int currentNodeId = i < subtour.size() ? subtour[i] : subtour[i - subtour.size()];
            const VrpNode& node = sol.vrpProblem->vrpData.nodeList[currentNodeId];
            arcDistance = sol.vrpProblem->vrpData.disMatrix[prevNodeId][currentNodeId];

            arriveTime = prevLeaveTime + arcDistance;
            startServiceTime = std::min(std::max(arriveTime, node.timeWindow.first), node.timeWindow.second);

            prevNodeId = currentNodeId;
            prevLeaveTime = startServiceTime + node.serveTime;
            for (int i = 0; i < sol.vrpProblem->vrpData.numScenarios; ++i) {
                prevLoad[i] += node.demand[i];
                prevAccumulatedCapacityViolations += std::max(0.0, prevLoad[i] - sol.vrpProblem->vrpData.vehicleCapacity);
            }
            prevForwardTwPenaltySlack += std::max(0.0, arriveTime - node.timeWindow.second);
            prevAccumulatedDistance += arcDistance;
        }

        arcDistance = sol.vrpProblem->vrpData.disMatrix[prevNodeId][nextNodeId];
        arriveTime = prevLeaveTime + arcDistance;
        startServiceTime = std::min(std::max(arriveTime, sol.vrpProblem->vrpData.nodeList[nextNodeId].timeWindow.first),
                                    sol.vrpProblem->vrpData.nodeList[nextNodeId].timeWindow.second);
        prevForwardTwPenaltySlack +=
            std::max(0.0, arriveTime - sol.vrpProblem->vrpData.nodeList[nextNodeId].timeWindow.second);
        prevAccumulatedDistance += arcDistance;

        insertions.emplace_back();
        SubtourInsertionData& insertionData = insertions.back();
        insertionData.prevNodeId = insertPrevNodeId;
        insertionData.routeId = routeId;
        if (0 == nextNodeId) {
            insertionData.deltaDistance = prevAccumulatedDistance - sol.routes[routeId].distance;
            insertionData.deltaTwPenalty = prevForwardTwPenaltySlack - sol.routes[routeId].twViolation;
        } else {
            insertionData.deltaDistance = prevAccumulatedDistance - sol.routeNodes[nextNodeId].accumulatedDistance;
            insertionData.deltaTwPenalty =
                prevForwardTwPenaltySlack + sol.routeNodes[nextNodeId].backwardTwPenaltySlack +
                std::max(startServiceTime - sol.routeNodes[nextNodeId].latestStartServiceTime, 0.0) -
                sol.routes[routeId].twViolation;
        }

        insertionData.deltaCapacityViolations = prevAccumulatedCapacityViolations;
        if (0 != insertPrevNodeId) {
            insertionData.deltaCapacityViolations -= sol.routeNodes[insertPrevNodeId].accumulatedCapacityViolation;
        }

        insertionData.newVehicle = false;

        if (0 == insertPrevNodeId) {
            insertPrevNodeId = sol.routes[routeId].startNode->nodeId;
        } else {
            RouteNode* tmpNextNode = sol.routeNodes[insertPrevNodeId].nexNode;
            if (nullptr == tmpNextNode) {
                break;
            }
            insertPrevNodeId = tmpNextNode->nodeId;
        }
    }
}

void connectSubtours(const VrpConfig& vrpConfig, VrpSolution& interSol, std::vector<std::vector<int>>& subtours) {
    std::vector<int> subtourIndices;
    subtourIndices.resize(subtours.size(), 0);
    std::iota(subtourIndices.begin(), subtourIndices.end(), 0);
    std::shuffle(subtourIndices.begin(), subtourIndices.end(), GetRandomGenerator());
    std::unordered_set<int> subtourBreakPosSet;
    std::unordered_set<int> insertPickupNodeIdSet;

    for (int subtourIndex : subtourIndices) {
        std::vector<int>& subtour = subtours[subtourIndex];
        subtourBreakPosSet.clear();

        std::unordered_set<int> pickupSet;
        for (int i = 0; i < (int)subtour.size(); ++i) {
            pickupSet.clear();
            bool pickupFirst = true;
            for (int j = i; j < (int)subtour.size() + i; ++j) {
                int nodeId = (j < (int)subtour.size() ? subtour[j] : subtour[j - subtour.size()]);
                const VrpNode& node = interSol.vrpProblem->vrpData.nodeList[nodeId];
                if (Utils::VrpNodeType::kPickup == node.nodeType) {
                    pickupSet.insert(nodeId);
                } else {
                    int corresNodeId = node.corresNodeId;
                    if (pickupSet.find(corresNodeId) == pickupSet.end()) {
                        pickupFirst = false;
                        break;
                    }
                }
            }

            if (pickupFirst) {
                subtourBreakPosSet.insert(i);
            }
        }

        SubtourInsertionData* bestInsertionData = nullptr;
        double bestDeltaCost = std::numeric_limits<double>::max();
        int bestBreakPos = -1;

        std::unordered_map<int, std::unordered_map<int, std::vector<SubtourInsertionData>>> insertionDataMap;
        for (int subtourBreakPos : subtourBreakPosSet) {
            for (int routeId = 0; routeId < (int)interSol.routes.size(); ++routeId) {
                calSubtourInsertionData(interSol, subtour, subtourBreakPos, routeId,
                                        insertionDataMap[subtourBreakPos][routeId]);
                for (SubtourInsertionData& insertion : insertionDataMap[subtourBreakPos][routeId]) {
                    double insertionCost = vrpConfig.vehicleCost * (insertion.newVehicle ? 1.0 : 0.0) +
                                           insertion.deltaDistance +
                                           insertion.deltaCapacityViolations * vrpConfig.capacityPenaltyCoeff +
                                           insertion.deltaTwPenalty * vrpConfig.twPenaltyCoeff;
                    if (nullptr == bestInsertionData || insertionCost < bestDeltaCost) {
                        bestInsertionData = &insertion;
                        bestDeltaCost = insertionCost;
                        bestBreakPos = subtourBreakPos;
                    }
                }
            }
        }

        if (nullptr == bestInsertionData) {
            continue;
        }

        int insertRouteId = bestInsertionData->routeId;
        int insertPrevNodeId = bestInsertionData->prevNodeId;
        int insertNextNodeId = 0;
        if (interSol.routes[insertRouteId].nodeNum > 0) {
            if (0 == insertPrevNodeId) {
                insertNextNodeId = interSol.routes[insertRouteId].startNode->nodeId;
            } else {
                RouteNode* tmpNextNode = interSol.routeNodes[insertPrevNodeId].nexNode;
                if (nullptr == tmpNextNode) {
                    insertNextNodeId = 0;
                } else {
                    insertNextNodeId = tmpNextNode->nodeId;
                }
            }
        }

        int prevNodeId = insertPrevNodeId;
        for (int i = bestBreakPos; i < bestBreakPos + subtour.size(); ++i) {
            int currentNodeId = i < subtour.size() ? subtour[i] : subtour[i - subtour.size()];
            interSol.routeNodes[currentNodeId].routeId = insertRouteId;

            if (0 == prevNodeId) {
                interSol.routes[insertRouteId].startNode = &interSol.routeNodes[currentNodeId];
                interSol.routeNodes[currentNodeId].preNode = nullptr;
            } else {
                interSol.routeNodes[prevNodeId].nexNode = &interSol.routeNodes[currentNodeId];
                interSol.routeNodes[currentNodeId].preNode = &interSol.routeNodes[prevNodeId];
            }

            if (Utils::VrpNodeType::kPickup == interSol.vrpProblem->vrpData.nodeList[currentNodeId].nodeType) {
                insertPickupNodeIdSet.insert(currentNodeId);
            }

            prevNodeId = currentNodeId;
        }
        interSol.routes[insertRouteId].nodeNum += (int)subtour.size();

        if (0 == insertNextNodeId) {
            interSol.routes[insertRouteId].endNode = &interSol.routeNodes[prevNodeId];
            interSol.routeNodes[prevNodeId].nexNode = nullptr;
        } else {
            interSol.routeNodes[insertNextNodeId].preNode = &interSol.routeNodes[prevNodeId];
            interSol.routeNodes[prevNodeId].nexNode = &interSol.routeNodes[insertNextNodeId];
        }

        interSol.updateRouteData(insertRouteId, &interSol.routeNodes[subtour[bestBreakPos]],
                                 &interSol.routeNodes[bestBreakPos == 0 ? subtour.back() : subtour[bestBreakPos - 1]]);
    }

    for (int i = 0; i < interSol.unassignedNodeIds.size();) {
        int unassignedNodeId = interSol.unassignedNodeIds[i].first;

        if (insertPickupNodeIdSet.find(unassignedNodeId) != insertPickupNodeIdSet.end()) {
            std::swap(interSol.unassignedNodeIds[i], interSol.unassignedNodeIds.back());
            interSol.unassignedNodeIds.pop_back();
        } else {
            ++i;
        }
    }
}

void eax(const VrpConfig& vrpConfig, const VrpSolution& firstSol, const VrpSolution& secondSol, VrpSolution& childSol) {
    VrpSolution newFirstSol(firstSol.vrpProblem);
    newFirstSol.copySolution(firstSol);

    VrpSolution newSecondSol(secondSol.vrpProblem);
    newSecondSol.copySolution(secondSol);

    if (newSecondSol.isSameWith(newFirstSol)) {
        perturb(vrpConfig, newFirstSol);

        if (newSecondSol.isSameWith(newFirstSol)) {
            childSol.copySolution(newFirstSol);
            return;
        }
    }

    EaxGraph firstGraph;
    EaxGraph secondGraph;
    eaxInitGraph(newFirstSol, firstGraph, 0);
    eaxInitGraph(secondSol, secondGraph, 1);

    GraphCircle graphCircle;
    if (!calGraphCircle(firstGraph, secondGraph, graphCircle) || (int)graphCircle.circleStart.size() <= 1) {
        // Crossover could not form a valid (non-empty) cycle decomposition, e.g. the
        // parents share the same edge set; keep a parent instead of indexing past the
        // end of an empty circle list.
        childSol.copySolution(newFirstSol);
        return;
    }

    std::unordered_set<const EaxEdge*> interGraphEdges;
    std::vector<const EaxEdge*> interGraphNodeHeadEdges;
    std::vector<const EaxEdge*> interGraphNodeTailEdges;
    std::vector<const EaxEdge*> interGraphDepotHeadEdges;
    std::vector<const EaxEdge*> interGraphDepotTailEdges;
    calInterGraph(newFirstSol, firstGraph, graphCircle, interGraphEdges, interGraphNodeHeadEdges,
                  interGraphNodeTailEdges, interGraphDepotHeadEdges, interGraphDepotTailEdges);

    std::vector<std::vector<int>> subtours;
    constructInterSol(interGraphEdges, interGraphNodeHeadEdges, interGraphNodeTailEdges, interGraphDepotHeadEdges,
                      interGraphDepotTailEdges, childSol, subtours);

    connectSubtours(vrpConfig, childSol, subtours);

#ifndef NDEBUG
    std::vector<uint8_t> nodeVisitedFlag(childSol.vrpProblem->vrpData.nodeNum, 0);
    for (const VrpRoute& route : childSol.routes) {
        if (0 >= route.nodeNum) {
            assert(nullptr == route.startNode);
            assert(nullptr == route.endNode);
            continue;
        }

        std::unordered_set<int> pickupNodesSet;
        const RouteNode* currentNode = route.startNode;

        while (nullptr != currentNode) {
            int currentNodeId = currentNode->nodeId;
            assert(route.routeId == currentNode->routeId);

            if (childSol.vrpProblem->vrpData.nodeList[currentNodeId].nodeType == Utils::VrpNodeType::kPickup) {
                pickupNodesSet.insert(currentNodeId);
            } else {
                int corresNodeId = childSol.vrpProblem->vrpData.nodeList[currentNodeId].corresNodeId;
                assert(&childSol.routeNodes[corresNodeId] == currentNode->corresRouteNode);

                if (pickupNodesSet.find(corresNodeId) == pickupNodesSet.end()) {
                    printf("Pickup and Delivery Precedence Violation\n");
                    exit(0);
                } else {
                    pickupNodesSet.erase(corresNodeId);
                    assert(0 == nodeVisitedFlag[currentNodeId]);
                    assert(0 == nodeVisitedFlag[corresNodeId]);
                    nodeVisitedFlag[currentNodeId] = 1;
                    nodeVisitedFlag[corresNodeId] = 1;
                }
            }

            currentNode = currentNode->nexNode;
        }

        if (!pickupNodesSet.empty()) {
            printf("Pickup and Delivery not in Same Route Violation\n");
            exit(0);
        }
    }
    for (const std::pair<int, int>& unassignedNode : childSol.unassignedNodeIds) {
        assert(0 == nodeVisitedFlag[unassignedNode.first]);
        assert(0 == nodeVisitedFlag[unassignedNode.second]);
    }
    int unassignedNodeNum = (int)std::count(nodeVisitedFlag.begin() + 1, nodeVisitedFlag.end(), 0);
    assert(childSol.unassignedNodeIds.size() * 2 == unassignedNodeNum);
#endif
    greedyInsertWithBlinks(vrpConfig, childSol, true);
    insertEmptyRoute(childSol);
    localSearch(vrpConfig, childSol);
    largeNeighborhoodSearch(vrpConfig, childSol);
    insertEmptyRoute(childSol);
    localSearch(vrpConfig, childSol);
}