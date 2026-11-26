#ifndef AMDAHL_SRC_DOMAIN_UTILS_UTILS_H_
#define AMDAHL_SRC_DOMAIN_UTILS_UTILS_H_

#include <limits>

namespace Utils {
enum class VrpNodeType {
    kDepot = 0,
    kPickup,
    kDelivery,
};

constexpr double kDoubleMax = std::numeric_limits<double>::max();

}  // namespace Utils

#endif