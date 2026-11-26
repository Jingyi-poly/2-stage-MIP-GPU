#ifndef AMDAHL_SRC_UTIL_RANDOM_UTILS_H_
#define AMDAHL_SRC_UTIL_RANDOM_UTILS_H_

#include <cstdint>
#include <random>

constexpr std::uint32_t kRandomSeed = 42;

// Resets the algorithm RNG stream for the calling thread and every worker
// thread on its next use. Call this between solver runs, while no worker is
// using the generator, to make repeated runs start from the requested seed.
void ResetRandomGenerator(std::uint32_t seed = kRandomSeed);

std::mt19937& GetRandomGenerator();

int GetRandInt(int min, int max);

double GetRandDouble(double min, double max);
#endif
