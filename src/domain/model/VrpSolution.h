#ifndef AMDAHL_SRC_DOMAIN_MODEL_VRP_SOLUTION_H_
#define AMDAHL_SRC_DOMAIN_MODEL_VRP_SOLUTION_H_

#include <vector>

#include "domain/model/VrpProblem.h"
#include "domain/model/entity/VrpConfig.h"
#include "domain/model/entity/VrpRoute.h"

constexpr double kRouteHashFactor = 0.12345;

class VrpSolution {
   public:
    enum Feasibility {
        FEASIBLE,
        INFEASIBLE,
        NO_JUDGE,
    };

    VrpSolution* preSol;
    VrpSolution* nexSol;
    int vehicleNum;
    double totalDistance;
    std::vector<VrpRoute> routes;
    Feasibility feasibility;
    std::vector<RouteNode> routeNodes;
    std::vector<double> twViolations;
    double totalTwViolation;
    std::vector<double> capacityViolations;
    double totalCapacityViolation;
    double totalPickupDeliveryPrecedenceViolation;
    double totalPickupDeliveryPairingViolation;
    std::vector<std::pair<int, int>> unassignedNodeIds;

    bool totalValueValid{false};
    VrpProblem* vrpProblem{nullptr};

    explicit VrpSolution(VrpProblem* problem);

    void removeRoute(int routeId);
    void removeNode(int routeId, int pickupNodeId, int deliveryNodeId, bool updateData);
    void updateRouteData(int routeId, RouteNode* forwardStartNode, RouteNode* backwardStartNode);
    void insertNode(int routeId, int pickupNodeId, int deliveryNodeId, RouteNode* pickupPreNode,
                    RouteNode* deliveryPreNode, bool updateData);
    void copyRoute(const VrpSolution& fromSol, int fromRouteId, int toRouteId);
    void copySolution(const VrpSolution& fromSol);
    void calTotalValue();
    double getTotalDistance();
    double getTotalTwViolation();
    double getTotalCapacityViolation();
    double getTotalObjectiveValue(const VrpConfig& vrpConfig);
    double getRouteHash();
    bool isSameWith(VrpSolution& otherSol);
    void checkData();
};
#endif