#include "util/random_utils.h"

#include <atomic>
#include <cstdint>
#include <random>

namespace {

std::atomic<std::uint32_t> g_randomSeed{kRandomSeed};
std::atomic<std::uint64_t> g_randomSeedEpoch{1};

}  // namespace

void ResetRandomGenerator(std::uint32_t seed) {
    // Solver runs call this while OpenMP workers are idle. Publishing the new
    // epoch after the seed makes every thread lazily reseed on its next draw.
    g_randomSeed.store(seed, std::memory_order_relaxed);
    g_randomSeedEpoch.fetch_add(1, std::memory_order_release);
}

std::mt19937& GetRandomGenerator() {
    static thread_local std::mt19937 generator;
    static thread_local std::uint64_t localSeedEpoch = 0;

    const std::uint64_t publishedEpoch = g_randomSeedEpoch.load(std::memory_order_acquire);
    if (localSeedEpoch != publishedEpoch) {
        generator.seed(g_randomSeed.load(std::memory_order_relaxed));
        localSeedEpoch = publishedEpoch;
    }
    return generator;
}

int GetRandInt(int min, int max) {
    // Guard against empty ranges (e.g. picking an index inside an empty route, which
    // yields GetRandInt(0, -1)): uniform_int_distribution requires min <= max, and
    // violating it is undefined behaviour (observed as a crash/stack overflow).
    if (min >= max) return min;
    std::uniform_int_distribution<int> distribution(min, max);
    return distribution(GetRandomGenerator());
}

double GetRandDouble(double min, double max) {
    std::uniform_real_distribution<double> distribution(min, max);
    return distribution(GetRandomGenerator());
}
