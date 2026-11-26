#include "domain/solver/memetic/crossover/LcsDp.h"

int CalLCS(const std::vector<int>& first_arc_seq, const std::vector<int>& second_arc_seq,
           int first_valid_lcs_value_length, int second_valid_lcs_value_length, int first_arc_seq_index,
           int second_arc_seq_index, std::vector<int>& lcs_values, int max_arc_seq_length) {
    if (0 == first_arc_seq_index) {
        lcs_values[second_arc_seq_index] = 0;
        return 0;
    } else if (0 == second_arc_seq_index) {
        lcs_values[first_arc_seq_index * max_arc_seq_length] = 0;
        return 0;
    }

    int tmp_index = first_arc_seq_index * max_arc_seq_length + second_arc_seq_index;
    int& lcs_value = lcs_values[tmp_index];
    if (kInitialLCSValue != lcs_value) {
        return lcs_value;
    }

    if (first_arc_seq[first_arc_seq_index] == second_arc_seq[second_arc_seq_index]) {
        lcs_value = CalLCS(first_arc_seq, second_arc_seq, first_valid_lcs_value_length, second_valid_lcs_value_length,
                           first_arc_seq_index - 1, second_arc_seq_index - 1, lcs_values, max_arc_seq_length) +
                    1;
        return lcs_value;
    } else {
        int tmp_first_lcs_value =
            CalLCS(first_arc_seq, second_arc_seq, first_valid_lcs_value_length, second_valid_lcs_value_length,
                   first_arc_seq_index - 1, second_arc_seq_index, lcs_values, max_arc_seq_length);
        int tmp_second_lcs_value =
            CalLCS(first_arc_seq, second_arc_seq, first_valid_lcs_value_length, second_valid_lcs_value_length,
                   first_arc_seq_index, second_arc_seq_index - 1, lcs_values, max_arc_seq_length);
        lcs_value = std::max(tmp_first_lcs_value, tmp_second_lcs_value);
        return lcs_value;
    }
}