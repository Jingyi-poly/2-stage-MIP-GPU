#ifndef AMDAHL_SRC_DOMAIN_MODEL_ENTITY_VRP_NODE_H_
#define AMDAHL_SRC_DOMAIN_MODEL_ENTITY_VRP_NODE_H_

#include <utility>
#include <vector>
#include <set>

#include "domain/utils/Utils.h"

class VrpNode {
   public:
    int id;
    double lat;
    double lon;
    Utils::VrpNodeType nodeType;
    std::vector<double> demand;
    double serveTime;
    std::pair<double, double> timeWindow;
    int corresNodeId;
    std::set<int> candiPreNodes;
    std::set<int> candiNextNodes;
};
#endif