#ifndef AMDAHL_SRC_IO_REQUEST_PAIRING_READER_H_
#define AMDAHL_SRC_IO_REQUEST_PAIRING_READER_H_

#include <string>
#include <unordered_map>
#include <unordered_set>

bool ReadRequestPairing(const std::string& request_pairing_file,
                        std::unordered_map<int, std::unordered_set<int>>& request_pairing_data);

#endif