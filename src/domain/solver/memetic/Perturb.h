#ifndef AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_PERTURB_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_PERTURB_H_

#include "domain/model/VrpSolution.h"
#include "domain/solver/memetic/insert/Insert.h"

constexpr int kRandomMoveNumInPerturb = 5;

int selectInsertion(const VrpConfig& vrpConfig, const std::vector<InsertionData>& candidateInsertions);
void randomMove(const VrpConfig& vrpConfig, VrpSolution& vrpSolution);
void perturb(const VrpConfig& vrpConfig, VrpSolution& vrpSolution);

#endif