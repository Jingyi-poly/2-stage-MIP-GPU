#ifndef AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_LOCAL_SEARCH_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_LOCAL_SEARCH_H_

#include "domain/model/VrpSolution.h"

constexpr int kExchangeNeighborNum = 20;

bool outRelocateMove(const VrpConfig& vrpConfig, VrpSolution& vrpSolution, int pickupNodeId, int deliveryNodeId);

bool outRelocateMove(const VrpConfig& vrpConfig, VrpSolution& vrpSolution);

bool outExchangeTwoRequests(const VrpConfig& vrpConfig, VrpSolution& vrpSolution, int firstPickupNodeId,
                            int firstDeliveryNodeId, int secondPickupNodeId, int secondDeliveryNodeId);

bool outExchangeMove(const VrpConfig& vrpConfig, VrpSolution& vrpSolution, int pickupNodeId, int deliveryNodeId);

bool outExchangeMove(const VrpConfig& vrpConfig, VrpSolution& vrpSolution);

void localSearch(const VrpConfig& vrpConfig, VrpSolution& vrpSolution);

#endif