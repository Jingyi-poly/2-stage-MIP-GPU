#include "io/valid_arcs_reader.h"

#include <fstream>
#include <iostream>

#include "common/constants.h"
#include "util/string_utils.h"

bool ReadValidArcs(const std::string& valid_arcs_file_path,
                   std::unordered_map<int, std::unordered_set<int>>& valid_arcs_data) {
    std::ifstream ifs;
    ifs.open(valid_arcs_file_path, std::ios::in);

    if (!ifs.is_open()) {
        std::cout << "Failed to open " << valid_arcs_file_path << std::endl;
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
        idx_t start_node_index = (idx_t)strtol(line_data.data(), &str_end_ptr, 10);
        *(line_data.data() + pos) = char_bak;

        pos = line_data.find_first_not_of(kWhiteSpaceChars, pos);
        idx_t end_node_index = (idx_t)strtol(line_data.data() + pos, &str_end_ptr, 10);

        valid_arcs_data[start_node_index].insert(end_node_index);
    }

    return success;
}