#ifndef AMDAHL_SRC_IO_VALID_ARCS_READER_H_
#define AMDAHL_SRC_IO_VALID_ARCS_READER_H_

#include <string>
#include <unordered_map>
#include <unordered_set>

bool ReadValidArcs(const std::string& valid_arcs_file_path,
                   std::unordered_map<int, std::unordered_set<int>>& valid_arcs_data);

#endif