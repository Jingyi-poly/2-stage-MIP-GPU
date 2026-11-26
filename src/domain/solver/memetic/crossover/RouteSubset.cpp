#include "domain/solver/memetic/crossover/RouteSubset.h"

#include <algorithm>

#include "util/random_utils.h"

void RouteSubset::CalArcSequences(const VrpSolution& sol, const std::vector<int>& route_indices,
                                  std::vector<int>& arc_sequence, int& arc_sequence_length) {
    arc_sequence_length = 0;
    for (int route_index : route_indices) {
        AddArcSequences(sol, route_index, arc_sequence, arc_sequence_length);
    }
}

void RouteSubset::AddArcSequences(const VrpSolution& sol, int route_index, std::vector<int>& arc_sequence,
                                  int& arc_sequence_length) {
    RouteNode* prevNode = nullptr;
    RouteNode* currentNode = sol.routes[route_index].startNode;

    while (nullptr != currentNode) {
        int arc_hash = CalArcHashValue(nullptr == prevNode ? 0 : prevNode->nodeId, currentNode->nodeId);
        arc_sequence[arc_sequence_length] = arc_hash;
        arc_sequence_length += 1;

        prevNode = currentNode;

        currentNode = currentNode->nexNode;
    }

    arc_sequence[arc_sequence_length] = CalArcHashValue(prevNode->nodeId, 0);
    arc_sequence_length += 1;
}

void RouteSubset::CalRouteIndicesHashValue(const std::vector<int>& route_indices, double& hash_value) {
    hash_value = 0.0;
    for (int route_index : route_indices) {
        hash_value += (route_index + 1.0) / max_arc_seq_length_;
    }
}

void RouteSubset::CalArcSequences() {
    CalArcSequences(first_sol_, first_route_indices_, first_arc_sequence_, first_arc_sequence_length_);
    CalArcSequences(second_sol_, second_route_indices_, second_arc_sequence_, second_arc_sequence_length_);
}

void RouteSubset::CalLCSValues() {
    lcs_value_ =
        CalLCS(first_arc_sequence_, second_arc_sequence_, first_arc_sequence_length_, second_arc_sequence_length_,
               first_arc_sequence_length_ - 1, second_arc_sequence_length_ - 1, lcs_values_, max_arc_seq_length_);
}

void RouteSubset::SelectRandomInitialRoutes() {
    int min_route_num = (int)std::min(first_sol_.routes.size(), second_sol_.routes.size());
    int select_route_num = GetRandInt(1, min_route_num);

    if (select_route_num == (int)first_sol_.routes.size()) {
        first_route_indices_.resize(select_route_num);
        std::iota(first_route_indices_.begin(), first_route_indices_.end(), 0);

        first_remaining_route_indices_.clear();
    } else {
        first_route_indices_.resize(first_sol_.routes.size());
        std::iota(first_route_indices_.begin(), first_route_indices_.end(), 0);
        std::shuffle(first_route_indices_.begin(), first_route_indices_.end(), GetRandomGenerator());

        first_remaining_route_indices_.assign(first_route_indices_.begin() + select_route_num,
                                              first_route_indices_.end());
        first_route_indices_.resize(select_route_num);
    }

    if (select_route_num == (int)second_sol_.routes.size()) {
        second_route_indices_.resize(select_route_num);
        std::iota(second_route_indices_.begin(), second_route_indices_.end(), 0);

        second_remaining_route_indices_.clear();
    } else {
        second_route_indices_.resize(second_sol_.routes.size());
        std::iota(second_route_indices_.begin(), second_route_indices_.end(), 0);
        std::shuffle(second_route_indices_.begin(), second_route_indices_.end(), GetRandomGenerator());

        second_remaining_route_indices_.assign(second_route_indices_.begin() + select_route_num,
                                               second_route_indices_.end());
        second_route_indices_.resize(select_route_num);
    }

    CalArcSequences();
    CalLCSValues();
}

void RouteSubset::SearchNeighborhood() {
    while (!first_remaining_route_indices_.empty() && !second_remaining_route_indices_.empty()) {
        int old_first_arc_sequence_length = first_arc_sequence_length_;
        int old_second_arc_sequence_length = second_arc_sequence_length_;

        int first_route_to_add = GetRandInt(0, (int)first_remaining_route_indices_.size() - 1);
        int first_route_index_to_add = first_remaining_route_indices_[first_route_to_add];

        int second_route_to_added = GetRandInt(0, (int)second_remaining_route_indices_.size() - 1);
        int second_route_index_to_add = second_remaining_route_indices_[second_route_to_added];

        AddArcSequences(first_sol_, first_route_index_to_add, first_arc_sequence_, first_arc_sequence_length_);
        AddArcSequences(second_sol_, second_route_index_to_add, second_arc_sequence_, second_arc_sequence_length_);

        int new_lcs_value =
            CalLCS(first_arc_sequence_, second_arc_sequence_, first_arc_sequence_length_, second_arc_sequence_length_,
                   first_arc_sequence_length_ - 1, second_arc_sequence_length_ - 1, lcs_values_, max_arc_seq_length_);
        if (new_lcs_value > lcs_value_) {
            lcs_value_ = new_lcs_value;
            first_route_indices_.push_back(first_route_index_to_add);
            second_route_indices_.push_back(second_route_index_to_add);

            first_remaining_route_indices_[first_route_to_add] = first_remaining_route_indices_.back();
            first_remaining_route_indices_.resize(first_remaining_route_indices_.size() - 1);

            second_remaining_route_indices_[second_route_to_added] = second_remaining_route_indices_.back();
            second_remaining_route_indices_.resize(second_remaining_route_indices_.size() - 1);
        } else {
            first_arc_sequence_length_ = old_first_arc_sequence_length;
            second_arc_sequence_length_ = old_second_arc_sequence_length;
            break;
        }
    }

    std::sort(first_route_indices_.begin(), first_route_indices_.end());
    std::sort(second_route_indices_.begin(), second_route_indices_.end());
    CalRouteIndicesHashValue(first_route_indices_, first_route_indices_hash_value_);
    CalRouteIndicesHashValue(second_route_indices_, second_route_indices_hash_value_);
}

bool RouteSubset::IsDuplicatedWith(const RouteSubset& other_route_subset) {
    if (first_route_indices_.size() != other_route_subset.first_route_indices_.size() ||
        second_route_indices_.size() != other_route_subset.second_route_indices_.size()) {
        return false;
    }
    if (first_route_indices_hash_value_ != other_route_subset.first_route_indices_hash_value_ ||
        second_route_indices_hash_value_ != other_route_subset.second_route_indices_hash_value_) {
        return false;
    }

    for (size_t i = 0; i < first_route_indices_.size(); ++i) {
        if (first_route_indices_[i] != other_route_subset.first_route_indices_[i]) {
            return false;
        }
    }

    for (size_t i = 0; i < second_route_indices_.size(); ++i) {
        if (second_route_indices_[i] != other_route_subset.second_route_indices_[i]) {
            return false;
        }
    }

    return true;
}

void RouteSubset::clearLcsValues() { lcs_values_.clear(); }

void RouteSubset::clearArcSequence() {
    first_arc_sequence_.clear();
    second_arc_sequence_.clear();

    first_arc_sequence_length_ = 0;
    second_arc_sequence_length_ = 0;
}