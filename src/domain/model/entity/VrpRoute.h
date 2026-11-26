#ifndef AMDAHL_SRC_DOMAIN_MODEL_ENTITY_VRP_ROUTE_H_
#define AMDAHL_SRC_DOMAIN_MODEL_ENTITY_VRP_ROUTE_H_

#include <vector>

#include "domain/utils/Utils.h"
struct RouteNode {
   public:
    int nodeId;
    int routeId;
    RouteNode* preNode;
    RouteNode* nexNode;
    RouteNode* corresRouteNode;
    double arriveTime;
    double startServiceTime;
    double leaveTime;
    std::vector<double> loadStatus;
    double latestStartServiceTime;
    double forwardTwPenaltySlack;
    double backwardTwPenaltySlack;
    double accumulatedCapacityViolation;
    double accumulatedDistance;
    explicit RouteNode(int id, int num_scenarios) {
        nodeId = id;
        routeId = -1;
        preNode = nullptr;
        nexNode = nullptr;
        corresRouteNode = nullptr;
        arriveTime = Utils::kDoubleMax;
        startServiceTime = Utils::kDoubleMax;
        leaveTime = Utils::kDoubleMax;
        loadStatus.assign(num_scenarios, 0.0);
        latestStartServiceTime = Utils::kDoubleMax;
        forwardTwPenaltySlack = 0.0;
        backwardTwPenaltySlack = 0.0;
        accumulatedCapacityViolation = 0.0;
        accumulatedDistance = 0.0;
    }
    ~RouteNode() {
        preNode = nullptr;
        nexNode = nullptr;
        corresRouteNode = nullptr;
    };

    void clearData();
};

class VrpRoute {
   public:
    int routeId;
    int nodeNum;
    RouteNode* startNode;
    RouteNode* endNode;
    std::vector<int> nodeIds;
    double distance;
    double twViolation;

    explicit VrpRoute(int id) {
        routeId = id;
        nodeNum = 0;
        startNode = nullptr;
        endNode = nullptr;
        distance = 0.0;
        twViolation = 0.0;
    }
    ~VrpRoute() {
        startNode = nullptr;
        endNode = nullptr;
    }

    void clearData();
};

#endif
