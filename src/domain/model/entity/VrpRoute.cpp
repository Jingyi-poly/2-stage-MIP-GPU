#include "domain/model/entity/VrpRoute.h"

void RouteNode::clearData() {
    routeId = -1;
    preNode = nullptr;
    nexNode = nullptr;
    arriveTime = Utils::kDoubleMax;
    startServiceTime = Utils::kDoubleMax;
    leaveTime = Utils::kDoubleMax;
    loadStatus.assign(loadStatus.size(), 0.0);
    latestStartServiceTime = Utils::kDoubleMax;
    forwardTwPenaltySlack = 0.0;
    backwardTwPenaltySlack = 0.0;
    accumulatedCapacityViolation = 0.0;
    accumulatedDistance = 0.0;
}

void VrpRoute::clearData() {
    nodeNum = 0;
    startNode = nullptr;
    endNode = nullptr;
    nodeIds.clear();
    distance = 0.0;
    twViolation = 0.0;
}