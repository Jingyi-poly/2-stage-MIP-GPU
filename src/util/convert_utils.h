#ifndef AMDAHL_SRC_UTIL_CONVERT_UTILS_H_
#define AMDAHL_SRC_UTIL_CONVERT_UTILS_H_

#include "domain/model/VrpProblem.h"
#include "domain/model/VrpSolution.h"
#include "model/pdptw.h"
#include "model/solution.h"

void ConvertVrpProblem(const PDPTW& pdptw, VrpProblem& vrpProblem);

void ConvertVrpSolution(const Solution& solution, VrpSolution& vrpSolution);

#endif