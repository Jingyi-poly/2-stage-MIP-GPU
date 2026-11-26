#ifndef AMDAHL_SRC_UTIL_SOL_SIMILARITY_H_
#define AMDAHL_SRC_UTIL_SOL_SIMILARITY_H_

#include "domain/model/VrpSolution.h"

double calculateSimilarity(const VrpSolution& solution1, const VrpSolution& solution2);

#endif