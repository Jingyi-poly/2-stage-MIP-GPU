#ifndef AMDAHL_SRC_COMMON_CONSTANTS_H_
#define AMDAHL_SRC_COMMON_CONSTANTS_H_

#include <cstdint>
#include <limits>

typedef int32_t demand_t;
typedef double pos_t;
typedef int32_t idx_t;
typedef double real_t;

constexpr double kDoubleMax = std::numeric_limits<real_t>::max();

#endif
