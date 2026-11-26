#ifndef AMDAHL_SRC_DOMAIN_MODEL_ENTITY_VRP_DATA_H_
#define AMDAHL_SRC_DOMAIN_MODEL_ENTITY_VRP_DATA_H_

#include <string>
#include <vector>

#include "domain/model/entity/VrpNode.h"

class VrpData {
   public:
    int nodeNum;
    int numScenarios;
    int vehicleUpLimit;
    double vehicleCapacity;
    std::vector<VrpNode> nodeList;
    std::vector<std::vector<double>> disMatrix;
    std::vector<int> pickNodeIds;
    std::string name;

    // Node-major Structure-of-Arrays copy of per-scenario demands, laid out as
    // demandMatrix[nodeId * numScenarios + scenario]. Built once after parsing and
    // consumed by the (GPU/CPU) batched scenario-capacity evaluators.
    std::vector<float> demandMatrix;
};

#endif