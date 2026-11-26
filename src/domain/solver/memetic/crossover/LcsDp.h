#ifndef AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_CROSSOVER_LCS_DP_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_CROSSOVER_LCS_DP_H_

#include <vector>

#include "domain/utils/Utils.h"
constexpr int kInitialLCSValue = -1;

static inline int CalArcHashValue(int head, int tail) {
    if (head >= tail) {
        return head * head + head + tail;
    } else {
        return head + tail * tail;
    }
}

int CalLCS(const std::vector<int>& first_arc_seq, const std::vector<int>& second_arc_seq,
           int first_valid_lcs_value_length, int second_valid_lcs_value_length, int first_arc_seq_index,
           int second_arc_seq_index, std::vector<int>& lcs_values, int max_arc_seq_length);

#endif