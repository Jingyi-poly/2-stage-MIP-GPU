#include "domain/solver/memetic/insert/Insert.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <unordered_map>

#include "domain/eval/ScenarioCapacityEvaluator.h"
#include "util/random_utils.h"

// In debug builds the inline CPU capacity loops are kept running even on the GPU
// path so every candidate's matrix delta can be asserted against the CPU value
// (not just the applied insertion). Compiles to a no-op in release.
#ifndef NDEBUG
constexpr bool kVerifyInsertionMatrix = true;
#else
constexpr bool kVerifyInsertionMatrix = false;
#endif
void calInsertionData(const VrpSolution& vrpSolution, RouteNode* pickupNode, RouteNode* deliveryNode, int routeId,
                      std::vector<InsertionData>& insertions, bool allowInfeasible,
                      const std::vector<double>* capacityDeltaMatrix) {
    assert(routeId < vrpSolution.routes.size());

    const VrpRoute& route = vrpSolution.routes[routeId];
    const VrpData& vrpData = vrpSolution.vrpProblem->vrpData;

    // Capacity-violation deltas precomputed on the (GPU) evaluator. Only valid for
    // the prune-free allowInfeasible enumeration, where every (pickupGap, deliveryGap)
    // candidate is generated and the matrix entry can be looked up directly.
    const bool useMatrix = allowInfeasible && capacityDeltaMatrix != nullptr;
    const int gapStride = route.nodeNum + 1;
    int pickupGap = 0;

    if (0 >= route.nodeNum) {
        insertions.emplace_back();
        InsertionData& insertionData = insertions.back();

        insertionData.routeId = routeId;
        insertionData.pickupPrevNode = nullptr;
        insertionData.deliveryPrevNode = nullptr;
        insertionData.deltaTwPenalty = 0.0;
        insertionData.deltaCapacityViolations = 0.0;
        insertionData.deltaDistance = vrpData.disMatrix[0][pickupNode->nodeId] +
                                      vrpData.disMatrix[pickupNode->nodeId][deliveryNode->nodeId] +
                                      vrpData.disMatrix[deliveryNode->nodeId][0];
        insertionData.newVehicle = true;

        return;
    }

    RouteNode* pickupPrevNode = nullptr;
    const VrpNode& pickupNodeData = vrpData.nodeList[pickupNode->nodeId];
    const VrpNode& deliveryNodeData = vrpData.nodeList[deliveryNode->nodeId];
    std::vector<double> prevLoad = std::vector<double>(vrpData.numScenarios, 0.0);
    std::vector<double> currentLoad = std::vector<double>(vrpData.numScenarios, 0.0);
    while (true) {
        RouteNode* pickupPrevNextNode = nullptr;
        if (nullptr == pickupPrevNode) {
            pickupPrevNextNode = route.startNode;
        } else {
            pickupPrevNextNode = pickupPrevNode->nexNode;
        }

        if (!allowInfeasible) {
            const VrpNode& pickupPrevNodeData =
                vrpData.nodeList[nullptr == pickupPrevNode ? 0 : pickupPrevNode->nodeId];
            bool tmpFeasible =
                pickupPrevNodeData.candiNextNodes.find(pickupNode->nodeId) != pickupPrevNodeData.candiNextNodes.end();

            if (!tmpFeasible) {
                pickupPrevNode = pickupPrevNextNode;
                if (nullptr == pickupPrevNextNode) {
                    break;
                } else {
                    continue;
                }
            }
        }

        double prevLeaveTime = vrpData.nodeList[0].timeWindow.first;
        double prevForwardTwPenaltySlack = 0.0;
        double prevAccumulatedCapacityViolations = 0.0;
        if (!useMatrix || kVerifyInsertionMatrix) {
            prevLoad.assign(vrpData.numScenarios, 0.0);
        }

        if (nullptr != pickupPrevNode) {
            prevLeaveTime = pickupPrevNode->leaveTime;
            prevForwardTwPenaltySlack = pickupPrevNode->forwardTwPenaltySlack;
            prevAccumulatedCapacityViolations = pickupPrevNode->accumulatedCapacityViolation;
            if (!useMatrix || kVerifyInsertionMatrix) {
                std::copy_n(pickupPrevNode->loadStatus.begin(), vrpData.numScenarios, prevLoad.begin());
            }
        }

        double currentArriveTime =
            prevLeaveTime +
            vrpData.disMatrix[nullptr == pickupPrevNode ? 0 : pickupPrevNode->nodeId][pickupNode->nodeId];
        double currentAccumulatedCapacityViolations = prevAccumulatedCapacityViolations;
        if (!useMatrix || kVerifyInsertionMatrix) {
            for (int i = 0; i < vrpData.numScenarios; ++i) {
                prevLoad[i] += pickupNodeData.demand[i];
                currentAccumulatedCapacityViolations += std::max(0.0, prevLoad[i] - vrpData.vehicleCapacity);
            }
        }

        double currentForwardTwPenaltySlack =
            prevForwardTwPenaltySlack + std::max(0.0, currentArriveTime - pickupNodeData.timeWindow.second);
        double currentStartServiceTime =
            std::min(std::max(currentArriveTime, pickupNodeData.timeWindow.first), pickupNodeData.timeWindow.second);

        if (!allowInfeasible) {
            bool tmpInfeasible = currentForwardTwPenaltySlack > 0.0 || currentAccumulatedCapacityViolations > 0.0;
            if (!tmpInfeasible) {
                double tmpRouteTwPenalty = prevForwardTwPenaltySlack;
                RouteNode* tmpNextNode = route.startNode;
                if (nullptr != pickupPrevNode) {
                    tmpNextNode = pickupPrevNode->nexNode;
                }

                double tmpNextLatestStartServiceTime = vrpData.nodeList[0].timeWindow.second;
                if (nullptr != tmpNextNode) {
                    tmpRouteTwPenalty += tmpNextNode->backwardTwPenaltySlack;
                    tmpNextLatestStartServiceTime = tmpNextNode->latestStartServiceTime;
                }
                tmpRouteTwPenalty +=
                    std::max(0.0, std::max(pickupNodeData.timeWindow.first, currentArriveTime) -
                                      std::min(tmpNextLatestStartServiceTime -
                                                   vrpData.disMatrix[pickupNode->nodeId]
                                                                    [tmpNextNode == nullptr ? 0 : tmpNextNode->nodeId] -
                                                   pickupNodeData.serveTime,
                                               pickupNodeData.timeWindow.second));

                if (tmpRouteTwPenalty > 0.0) {
                    tmpInfeasible = true;
                }
            }

            if (tmpInfeasible) {
                pickupPrevNode = pickupPrevNextNode;
                if (nullptr == pickupPrevNode) {
                    break;
                } else {
                    continue;
                }
            }
        }

        RouteNode* deliveryPrevNode = pickupNode;
        int deliveryGap = pickupGap;

        prevLeaveTime = currentStartServiceTime + pickupNodeData.serveTime;
        prevForwardTwPenaltySlack = currentForwardTwPenaltySlack;
        prevAccumulatedCapacityViolations = currentAccumulatedCapacityViolations;

        while (true) {
            currentArriveTime = prevLeaveTime + vrpData.disMatrix[deliveryPrevNode->nodeId][deliveryNode->nodeId];
            currentAccumulatedCapacityViolations = prevAccumulatedCapacityViolations;
            if (!useMatrix || kVerifyInsertionMatrix) {
                for (int i = 0; i < vrpData.numScenarios; ++i) {
                    currentLoad[i] = prevLoad[i] + deliveryNodeData.demand[i];
                    currentAccumulatedCapacityViolations += std::max(0.0, currentLoad[i] - vrpData.vehicleCapacity);
                }
            }
            currentForwardTwPenaltySlack =
                prevForwardTwPenaltySlack + std::max(0.0, currentArriveTime - deliveryNodeData.timeWindow.second);
            currentStartServiceTime = std::min(std::max(currentArriveTime, deliveryNodeData.timeWindow.first),
                                               deliveryNodeData.timeWindow.second);

            double tmpRouteTwPenalty = prevForwardTwPenaltySlack;
            RouteNode* tmpNextNode = nullptr;
            if (pickupNode != deliveryPrevNode) {
                tmpNextNode = deliveryPrevNode->nexNode;
            } else {
                if (nullptr != pickupPrevNode) {
                    tmpNextNode = pickupPrevNode->nexNode;
                } else {
                    tmpNextNode = route.startNode;
                }
            }

            double tmpNextLatestStartServiceTime = vrpData.nodeList[0].timeWindow.second;
            if (nullptr != tmpNextNode) {
                tmpRouteTwPenalty += tmpNextNode->backwardTwPenaltySlack;
                tmpNextLatestStartServiceTime = tmpNextNode->latestStartServiceTime;
            }
            tmpRouteTwPenalty += std::max(
                0.0,
                std::max(currentArriveTime, deliveryNodeData.timeWindow.first) -
                    std::min(
                        tmpNextLatestStartServiceTime - deliveryNodeData.serveTime -
                            vrpData.disMatrix[deliveryNode->nodeId][tmpNextNode == nullptr ? 0 : tmpNextNode->nodeId],
                        deliveryNodeData.timeWindow.second));

            if (!allowInfeasible) {
                bool tmpInfeasible = currentForwardTwPenaltySlack > 0.0 || currentAccumulatedCapacityViolations > 0.0;
                if (tmpInfeasible) {
                    break;
                }

                if (!tmpInfeasible) {
                    if (tmpRouteTwPenalty > 0.0) {
                        tmpInfeasible = true;
                    }
                }

                if (tmpInfeasible) {
                    if (nullptr == tmpNextNode) {
                        break;
                    } else {
                        const VrpNode& tmpNextNodeData =
                            vrpData.nodeList[tmpNextNode == nullptr ? 0 : tmpNextNode->nodeId];
                        currentArriveTime =
                            prevLeaveTime +
                            vrpData.disMatrix[deliveryPrevNode == nullptr ? 0 : deliveryPrevNode->nodeId]
                                             [tmpNextNode == nullptr ? 0 : tmpNextNode->nodeId];

                        for (int i = 0; i < vrpData.numScenarios; ++i) {
                            prevLoad[i] += tmpNextNodeData.demand[i];
                            prevAccumulatedCapacityViolations += std::max(0.0, prevLoad[i] - vrpData.vehicleCapacity);
                        }
                        currentForwardTwPenaltySlack =
                            prevForwardTwPenaltySlack +
                            std::max(0.0, currentArriveTime - tmpNextNodeData.timeWindow.second);
                        currentStartServiceTime =
                            std::min(std::max(currentArriveTime, tmpNextNodeData.timeWindow.first),
                                     tmpNextNodeData.timeWindow.second);

                        prevLeaveTime = currentStartServiceTime + tmpNextNodeData.serveTime;
                        prevForwardTwPenaltySlack = currentForwardTwPenaltySlack;
                        deliveryPrevNode = tmpNextNode;
                        continue;
                    }
                }
            }

            double tmpOldCapacityViolations;
            if (deliveryPrevNode == pickupNode) {
                if (nullptr == pickupPrevNode) {
                    tmpOldCapacityViolations = 0.0;
                } else {
                    tmpOldCapacityViolations = pickupPrevNode->accumulatedCapacityViolation;
                }
            } else {
                tmpOldCapacityViolations = deliveryPrevNode->accumulatedCapacityViolation;
            }

            insertions.emplace_back();
            InsertionData& insertionData = insertions.back();
            insertionData.routeId = routeId;
            insertionData.pickupPrevNode = pickupPrevNode;
            if (deliveryPrevNode == pickupNode) {
                insertionData.deliveryPrevNode = pickupPrevNode;
            } else {
                insertionData.deliveryPrevNode = deliveryPrevNode;
            }

            const double cpuDeltaCapacityViolations = currentAccumulatedCapacityViolations - tmpOldCapacityViolations;
            if (useMatrix) {
                const double matrixDelta =
                    (*capacityDeltaMatrix)[static_cast<size_t>(pickupGap) * gapStride + deliveryGap];
                if (kVerifyInsertionMatrix) {
                    assert(std::fabs(matrixDelta - cpuDeltaCapacityViolations) <=
                           1e-4 + 1e-6 * std::fabs(cpuDeltaCapacityViolations));
                }
                insertionData.deltaCapacityViolations = matrixDelta;
            } else {
                insertionData.deltaCapacityViolations = cpuDeltaCapacityViolations;
            }
            insertionData.deltaTwPenalty = tmpRouteTwPenalty - route.twViolation;
            insertionData.deltaDistance = 0.0;
            if (pickupPrevNode == deliveryPrevNode) {
                insertionData.deltaDistance =
                    vrpData.disMatrix[pickupPrevNode == nullptr ? 0 : pickupPrevNode->nodeId][pickupNode->nodeId] +
                    vrpData.disMatrix[pickupNode->nodeId][deliveryNode->nodeId];
                if (nullptr == pickupPrevNode) {
                    insertionData.deltaDistance += vrpData.disMatrix[deliveryNode->nodeId][route.startNode->nodeId];
                    insertionData.deltaDistance -= vrpData.disMatrix[0][route.startNode->nodeId];
                } else {
                    insertionData.deltaDistance +=
                        vrpData.disMatrix[deliveryNode->nodeId]
                                         [pickupPrevNextNode == nullptr ? 0 : pickupPrevNextNode->nodeId];
                    insertionData.deltaDistance -=
                        vrpData.disMatrix[pickupPrevNode->nodeId]
                                         [pickupPrevNextNode == nullptr ? 0 : pickupPrevNextNode->nodeId];
                }
            } else {
                if (nullptr == pickupPrevNode) {
                    insertionData.deltaDistance += vrpData.disMatrix[0][pickupNode->nodeId] +
                                                   vrpData.disMatrix[pickupNode->nodeId][route.startNode->nodeId] -
                                                   vrpData.disMatrix[0][route.startNode->nodeId];
                } else {
                    insertionData.deltaDistance +=
                        vrpData.disMatrix[pickupPrevNode->nodeId][pickupNode->nodeId] +
                        vrpData.disMatrix[pickupNode->nodeId]
                                         [pickupPrevNextNode == nullptr ? 0 : pickupPrevNextNode->nodeId] -
                        vrpData.disMatrix[pickupPrevNode->nodeId]
                                         [pickupPrevNextNode == nullptr ? 0 : pickupPrevNextNode->nodeId];
                }

                insertionData.deltaDistance +=
                    vrpData.disMatrix[deliveryPrevNode->nodeId][deliveryNode->nodeId] +
                    vrpData.disMatrix[deliveryNode->nodeId][tmpNextNode == nullptr ? 0 : tmpNextNode->nodeId] -
                    vrpData.disMatrix[deliveryPrevNode->nodeId][tmpNextNode == nullptr ? 0 : tmpNextNode->nodeId];
            }

            insertionData.newVehicle = false;
            if (nullptr == tmpNextNode) {
                break;
            }

            currentArriveTime = prevLeaveTime + vrpData.disMatrix[deliveryPrevNode->nodeId][tmpNextNode->nodeId];
            if (!useMatrix || kVerifyInsertionMatrix) {
                for (int i = 0; i < vrpData.numScenarios; ++i) {
                    prevLoad[i] += vrpData.nodeList[tmpNextNode->nodeId].demand[i];
                    prevAccumulatedCapacityViolations += std::max(0.0, prevLoad[i] - vrpData.vehicleCapacity);
                }
            }
            currentForwardTwPenaltySlack =
                prevForwardTwPenaltySlack +
                std::max(0.0, currentArriveTime - vrpData.nodeList[tmpNextNode->nodeId].timeWindow.second);
            currentStartServiceTime =
                std::min(std::max(currentArriveTime, vrpData.nodeList[tmpNextNode->nodeId].timeWindow.first),
                         vrpData.nodeList[tmpNextNode->nodeId].timeWindow.second);

            prevLeaveTime = currentStartServiceTime + vrpData.nodeList[tmpNextNode->nodeId].serveTime;
            prevForwardTwPenaltySlack = currentForwardTwPenaltySlack;
            deliveryPrevNode = tmpNextNode;
            ++deliveryGap;
        }

        if (nullptr == pickupPrevNextNode) {
            break;
        }

        prevLeaveTime = vrpData.nodeList[0].timeWindow.first;
        prevForwardTwPenaltySlack = 0.0;
        prevAccumulatedCapacityViolations = 0.0;
        if (!useMatrix || kVerifyInsertionMatrix) {
            prevLoad.assign(vrpData.numScenarios, 0.0);
        }

        if (nullptr != pickupPrevNode) {
            prevLeaveTime = pickupPrevNode->leaveTime;
            prevForwardTwPenaltySlack = pickupPrevNode->forwardTwPenaltySlack;
            prevAccumulatedCapacityViolations = pickupPrevNode->accumulatedCapacityViolation;
            if (!useMatrix || kVerifyInsertionMatrix) {
                std::copy_n(pickupPrevNode->loadStatus.begin(), vrpData.numScenarios, prevLoad.begin());
            }
        }

        currentArriveTime =
            prevLeaveTime +
            vrpData.disMatrix[nullptr == pickupPrevNode ? 0 : pickupPrevNode->nodeId][pickupPrevNextNode->nodeId];
        if (!useMatrix || kVerifyInsertionMatrix) {
            for (int i = 0; i < vrpData.numScenarios; ++i) {
                prevLoad[i] += vrpData.nodeList[pickupPrevNextNode->nodeId].demand[i];
                prevAccumulatedCapacityViolations += std::max(0.0, prevLoad[i] - vrpData.vehicleCapacity);
            }
        }
        currentForwardTwPenaltySlack =
            prevForwardTwPenaltySlack +
            std::max(0.0, currentArriveTime - vrpData.nodeList[pickupPrevNextNode->nodeId].timeWindow.second);
        currentStartServiceTime =
            std::min(std::max(currentArriveTime, vrpData.nodeList[pickupPrevNextNode->nodeId].timeWindow.first),
                     vrpData.nodeList[pickupPrevNextNode->nodeId].timeWindow.second);

        prevLeaveTime = currentStartServiceTime + vrpData.nodeList[pickupPrevNextNode->nodeId].serveTime;
        prevForwardTwPenaltySlack = currentForwardTwPenaltySlack;
        pickupPrevNode = pickupPrevNextNode;
        ++pickupGap;
    }
}

void bestInsert(const VrpConfig& vrpConfig, VrpSolution& vrpSolution, bool allowInfeasible) {
    std::unordered_map<int, std::unordered_map<int, std::vector<InsertionData>>> insertDataMap;
    std::unordered_map<int, int> unassignedNodeIndexMap;
    for (int i = 0; i < vrpSolution.unassignedNodeIds.size(); ++i) {
        int unassignedNodeId = vrpSolution.unassignedNodeIds[i].first;
        unassignedNodeIndexMap[unassignedNodeId] = i;

        for (int routeId = 0; routeId < (int)vrpSolution.routes.size(); ++routeId) {
            calInsertionData(
                vrpSolution, &vrpSolution.routeNodes[unassignedNodeId],
                &vrpSolution.routeNodes[vrpSolution.vrpProblem->vrpData.nodeList[unassignedNodeId].corresNodeId],
                routeId, insertDataMap[unassignedNodeId][routeId], allowInfeasible);
        }
    }

    while (!vrpSolution.unassignedNodeIds.empty()) {
        InsertionData* bestInsertionData = nullptr;
        int bestInsertPickupNodeId = -1;
        double minInsertCost = std::numeric_limits<double>::max();

        for (auto& [nodeId, routeInsertDataMap] : insertDataMap) {
            for (auto& [routeId, insertions] : routeInsertDataMap) {
                for (InsertionData& insertionData : insertions) {
                    if (!allowInfeasible) {
                        if (insertionData.deltaTwPenalty > 0.0 || insertionData.deltaCapacityViolations > 0.0) {
                            continue;
                        }
                    }

                    double cost = insertionData.deltaDistance;
                    if (insertionData.newVehicle) {
                        cost += vrpConfig.vehicleCost;
                    }
                    cost += vrpConfig.twPenaltyCoeff * insertionData.deltaTwPenalty;
                    cost += vrpConfig.capacityPenaltyCoeff * insertionData.deltaCapacityViolations;

                    if (cost < minInsertCost) {
                        minInsertCost = cost;
                        bestInsertionData = &insertionData;
                        bestInsertPickupNodeId = nodeId;
                    }
                }
            }
        }

        if (nullptr == bestInsertionData) {
            break;
        } else {
            RouteNode* pickupPrevNode = bestInsertionData->pickupPrevNode;
            RouteNode* deliveryPrevNode = bestInsertionData->deliveryPrevNode;
            int routeId = bestInsertionData->routeId;

            vrpSolution.insertNode(routeId, bestInsertPickupNodeId,
                                   vrpSolution.vrpProblem->vrpData.nodeList[bestInsertPickupNodeId].corresNodeId,
                                   pickupPrevNode, deliveryPrevNode, true);

            int unassignedNodeIndex = unassignedNodeIndexMap[bestInsertPickupNodeId];
            if (unassignedNodeIndex == vrpSolution.unassignedNodeIds.size() - 1) {
                vrpSolution.unassignedNodeIds.pop_back();
            } else {
                int lastUnassignedNodeId = vrpSolution.unassignedNodeIds.back().first;
                vrpSolution.unassignedNodeIds[unassignedNodeIndex] = vrpSolution.unassignedNodeIds.back();
                unassignedNodeIndexMap[lastUnassignedNodeId] = unassignedNodeIndex;
                vrpSolution.unassignedNodeIds.pop_back();
            }

            insertDataMap[bestInsertPickupNodeId].clear();

            for (auto& [nodeId, routeInsertDataMap] : insertDataMap) {
                for (auto& [tmpRouteId, insertions] : routeInsertDataMap) {
                    if (tmpRouteId != routeId) {
                        continue;
                    }
                    insertions.clear();
                    calInsertionData(
                        vrpSolution, &vrpSolution.routeNodes[nodeId],
                        &vrpSolution.routeNodes[vrpSolution.vrpProblem->vrpData.nodeList[nodeId].corresNodeId],
                        tmpRouteId, insertions, false);
                }
            }
        }
    }
}

void greedyInsertWithBlinks(const VrpConfig& vrpConfig, VrpSolution& vrpSolution, bool allowInfeasible) {
    const VrpData& vrpData = vrpSolution.vrpProblem->vrpData;
    double sortProb = GetRandDouble(0.0, 1.0);
    if (sortProb < kSortBasedOnRandomProbability) {
        std::shuffle(vrpSolution.unassignedNodeIds.begin(), vrpSolution.unassignedNodeIds.end(), GetRandomGenerator());
    } else if (sortProb < kSortBasedOnRandomProbability + kSortBasedOnDemandProbability) {
        std::sort(vrpSolution.unassignedNodeIds.begin(), vrpSolution.unassignedNodeIds.end(),
                  [&vrpData](const std::pair<int, int>& node1, const std::pair<int, int>& node2) {
                      return vrpData.nodeList[node1.first].demand > vrpData.nodeList[node2.first].demand;
                  });
    } else if (sortProb <
               kSortBasedOnRandomProbability + kSortBasedOnRandomProbability + kSortBasedOnFarDistanceProbability) {
        std::sort(vrpSolution.unassignedNodeIds.begin(), vrpSolution.unassignedNodeIds.end(),
                  [&vrpData](const std::pair<int, int>& node1, const std::pair<int, int>& node2) {
                      return vrpData.disMatrix[0][node1.first] > vrpData.disMatrix[0][node2.first];
                  });
    } else {
        std::sort(vrpSolution.unassignedNodeIds.begin(), vrpSolution.unassignedNodeIds.end(),
                  [&vrpData](const std::pair<int, int>& node1, const std::pair<int, int>& node2) {
                      return vrpData.disMatrix[0][node1.first] < vrpData.disMatrix[0][node2.first];
                  });
    }

    for (int i = 0; i < vrpSolution.unassignedNodeIds.size();) {
        int unassignedPickupNodeId = vrpSolution.unassignedNodeIds[i].first;
        int unassignedDeliveryNodeId = vrpSolution.unassignedNodeIds[i].second;

        assert(0 > vrpSolution.routeNodes[unassignedPickupNodeId].routeId);
        assert(0 > vrpSolution.routeNodes[unassignedDeliveryNodeId].routeId);

        InsertionData* bestInsertionData = nullptr;
        double minCost = std::numeric_limits<double>::max();

        std::unordered_map<int, std::vector<InsertionData>> insertDataMap;

        // Optionally evaluate every route's candidate capacity-violation deltas for
        // this request on the (GPU) evaluator with a single batched launch, then feed
        // each route's delta matrix into calInsertionData. The CPU enumeration is kept
        // intact and used whenever the GPU insertion path is disabled.
        const int routeCount = (int)vrpSolution.routes.size();
        std::vector<std::vector<double>> routeCapacityDeltas;
        const bool useGpuInsertion = vrpSolution.vrpProblem->useGpuInsertionEval &&
                                     vrpSolution.vrpProblem->capacityEvaluator != nullptr;
        if (useGpuInsertion) {
            InsertionProbeBatch batch;
            batch.routeOffsets.reserve(routeCount + 1);
            batch.pickupIds.reserve(routeCount);
            batch.deliveryIds.reserve(routeCount);
            batch.routeOffsets.push_back(0);
            for (int routeId = 0; routeId < routeCount; ++routeId) {
                for (RouteNode* node = vrpSolution.routes[routeId].startNode; node != nullptr; node = node->nexNode) {
                    batch.nodeIds.push_back(node->nodeId);
                }
                batch.routeOffsets.push_back(static_cast<int>(batch.nodeIds.size()));
                batch.pickupIds.push_back(unassignedPickupNodeId);
                batch.deliveryIds.push_back(unassignedDeliveryNodeId);
            }
            vrpSolution.vrpProblem->capacityEvaluator->EvaluateInsertionDeltasBatch(batch, routeCapacityDeltas);
        }

        for (int routeId = 0; routeId < routeCount; ++routeId) {
            const std::vector<double>* capacityDeltaMatrix =
                useGpuInsertion ? &routeCapacityDeltas[routeId] : nullptr;
            calInsertionData(vrpSolution, &vrpSolution.routeNodes[unassignedPickupNodeId],
                             &vrpSolution.routeNodes[unassignedDeliveryNodeId], routeId, insertDataMap[routeId], true,
                             capacityDeltaMatrix);
        }

        for (auto& [route_index, insertions] : insertDataMap) {
            for (InsertionData& insertionData : insertions) {
                if (!allowInfeasible) {
                    if (insertionData.deltaTwPenalty > 0.0 || insertionData.deltaCapacityViolations > 0.0) {
                        continue;
                    }
                }

                double tmpProb = GetRandDouble(0.0, 1.0);
                if (tmpProb >= 1 - kBlinkRate) {
                    continue;
                }

                double cost = insertionData.deltaDistance;
                if (insertionData.newVehicle) {
                    cost += vrpConfig.vehicleCost;
                }
                cost += insertionData.deltaCapacityViolations * vrpConfig.capacityPenaltyCoeff;
                cost += insertionData.deltaTwPenalty * vrpConfig.twPenaltyCoeff;

                if (cost < minCost) {
                    minCost = cost;
                    bestInsertionData = &insertionData;
                }
            }
        }

        if (nullptr == bestInsertionData) {
            i++;
        } else {
            RouteNode* pickupPrevNode = bestInsertionData->pickupPrevNode;
            RouteNode* deliveryPrevNode = bestInsertionData->deliveryPrevNode;
            int routeId = bestInsertionData->routeId;

#ifndef NDEBUG
            double oldCapacityViolation = 0.0;
            double oldTwViolation = 0.0;
            double oldDistance = 0.0;
            if (0 < vrpSolution.routes[routeId].nodeNum) {
                oldCapacityViolation = vrpSolution.routes[routeId].endNode->accumulatedCapacityViolation;
                oldTwViolation = vrpSolution.routes[routeId].twViolation;
                oldDistance = vrpSolution.routes[routeId].distance;
            }
#endif

            vrpSolution.insertNode(routeId, unassignedPickupNodeId, unassignedDeliveryNodeId, pickupPrevNode,
                                   deliveryPrevNode, true);

#ifndef NDEBUG
            assert(std::fabs(vrpSolution.routes[routeId].endNode->accumulatedCapacityViolation - oldCapacityViolation -
                             bestInsertionData->deltaCapacityViolations) < 1e-4);
            assert(std::fabs(vrpSolution.routes[routeId].twViolation - oldTwViolation -
                             bestInsertionData->deltaTwPenalty) < 1e-4);
            assert(std::fabs(vrpSolution.routes[routeId].distance - oldDistance - bestInsertionData->deltaDistance) <
                   1e-4);
#endif
        }
    }
}