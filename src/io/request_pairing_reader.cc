#include "io/request_pairing_reader.h"

#include <fstream>
#include <iostream>

#include "common/constants.h"
#include "util/string_utils.h"

bool ReadRequestPairing(const std::string& request_pairing_file,
                        std::unordered_map<int, std::unordered_set<int>>& request_pairing_data) {
    std::ifstream ifs;
    ifs.open(request_pairing_file, std::ios::in);

    if (!ifs.is_open()) {
        std::cout << "Failed to open " << request_pairing_file << std::endl;
        ifs.close();
        return false;
    }

    std::string line_data;
    bool success = true;

    while (getline(ifs, line_data)) {
        line_data.erase(line_data.find_last_not_of(kWhiteSpaceChars) + 1);
        size_t pos = line_data.find_first_of(kWhiteSpaceChars);

        char* str_end_ptr = nullptr;

        char char_bak = *(line_data.data() + pos);
        *(line_data.data() + pos) = kNullChar;
        idx_t first_node_index = (idx_t)strtol(line_data.data(), &str_end_ptr, 10);
        *(line_data.data() + pos) = char_bak;

        pos = line_data.find_first_not_of(kWhiteSpaceChars, pos);
        idx_t second_node_index = (idx_t)strtol(line_data.data() + pos, &str_end_ptr, 10);

        request_pairing_data[first_node_index].insert(second_node_index);
    }

    return success;
}