#ifndef AMDAHL_SRC_IO_SOL_WRITER_H_
#define AMDAHL_SRC_IO_SOL_WRITER_H_

#include <string>

#include "model/solution.h"

constexpr char kAuthors[] = "Team";
constexpr char kReference[] = "Unpublished";

bool WriteSol(const Solution& solution, const std::string& sol_file_path);

#endif