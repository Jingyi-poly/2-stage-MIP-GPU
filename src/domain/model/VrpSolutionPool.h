#ifndef AMDAHL_SRC_DOMAIN_MODEL_VRP_SOLUTION_POOL_H_
#define AMDAHL_SRC_DOMAIN_MODEL_VRP_SOLUTION_POOL_H_

#include "domain/model/VrpSolution.h"

class VrpSolutionPool {
   public:
    VrpSolution* bestSolution;
    VrpSolution* worstSolution;
};
#endif