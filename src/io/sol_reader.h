#ifndef AMDAHL_SRC_IO_SOL_READER_H_
#define AMDAHL_SRC_IO_SOL_READER_H_

#include <string>

#include "model/solution.h"

constexpr char kInstanceNameTag[] = "Instance name";
constexpr char kAuthorsTag[] = "Authors";
constexpr char kDateTag[] = "Date";
constexpr char kReferenceTag[] = "Reference";
constexpr char kSolutionTag[] = "Solution";
constexpr char kRouteTag[] = "Route";
constexpr int kTagWidth = 13;

bool ReadSol(const std::string& sol_file_path, Solution& solution);
#endif