#ifndef AMDAHL_SRC_UTIL_DISTANCE_UTILS_H_
#define AMDAHL_SRC_UTIL_DISTANCE_UTILS_H_

#include <cmath>

#include "common/constants.h"
static inline real_t GetEuclideanDis(pos_t x1, pos_t y1, pos_t x2, pos_t y2) { return std::hypot(x1 - x2, y1 - y2); }
#endif