#ifndef AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_CROSSOVER_ROUTE_SUBSET_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_CROSSOVER_ROUTE_SUBSET_H_

#include "domain/model/VrpProblem.h"
#include "domain/model/VrpSolution.h"
#include "domain/solver/memetic/crossover/LcsDp.h"
#include "domain/utils/Utils.h"

class RouteSubset {
   public:
    RouteSubset(const VrpSolution& firstSol, const VrpSolution& secondSol)
        : first_sol_(firstSol), second_sol_(secondSol) {
        int task_num = firstSol.vrpProblem->vrpData.nodeNum;
        int vehicle_num = (int)firstSol.routes.size();
        max_arc_seq_length_ = task_num + vehicle_num;
        first_arc_sequence_.resize(max_arc_seq_length_, -1);
        second_arc_sequence_.resize(max_arc_seq_length_, -1);

        first_arc_sequence_.resize(max_arc_seq_length_, -1);
        second_arc_sequence_.resize(max_arc_seq_length_, -1);

        first_arc_sequence_length_ = 0;
        second_arc_sequence_length_ = 0;
        lcs_values_.resize(max_arc_seq_length_ * max_arc_seq_length_, kInitialLCSValue);
        lcs_value_ = 0;
    }
    const VrpSolution& first_sol_;
    const VrpSolution& second_sol_;

    std::vector<int> first_arc_sequence_;
    std::vector<int> second_arc_sequence_;

    int first_arc_sequence_length_{0};
    int second_arc_sequence_length_{0};

    std::vector<int> lcs_values_;
    int lcs_value_{0};
    int max_arc_seq_length_{0};

    std::vector<int> first_route_indices_;
    std::vector<int> second_route_indices_;
    std::vector<int> first_remaining_route_indices_;
    std::vector<int> second_remaining_route_indices_;

    double first_route_indices_hash_value_{Utils::kDoubleMax};
    double second_route_indices_hash_value_{Utils::kDoubleMax};

    void SelectRandomInitialRoutes();

    void CalArcSequences();
    static void CalArcSequences(const VrpSolution& sol, const std::vector<int>& route_indices,
                                std::vector<int>& arc_sequence, int& arc_sequence_length);
    void CalLCSValues();

    void SearchNeighborhood();

    static void AddArcSequences(const VrpSolution& sol, int route_index, std::vector<int>& arc_sequence,
                                int& arc_sequence_length);
    void CalRouteIndicesHashValue(const std::vector<int>& route_indices, double& hash_value);

    bool IsDuplicatedWith(const RouteSubset& other_route_subset);

    void clearLcsValues();
    void clearArcSequence();
};

#endif