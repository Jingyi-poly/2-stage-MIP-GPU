#ifndef AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_CROSSOVER_EDGE_ASSEMBLY_CROSSOVER_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_CROSSOVER_EDGE_ASSEMBLY_CROSSOVER_H_

#include <cstdint>

#include "domain/model/VrpSolution.h"

constexpr double kEaxCircleSelectedBlockStrategyProb = 0.5;

struct EaxEdge {
    int head{-1};
    int tail{-1};
    uint8_t source{0};
    int index{-1};
};

struct EaxGraph {
    std::vector<EaxEdge> edges;
    std::vector<EaxEdge*> nodeHeadEdge;
    std::vector<EaxEdge*> nodeTailEdge;
    std::vector<EaxEdge*> depotHeadEdges;
    std::vector<EaxEdge*> depotTailEdges;
};

struct GraphCircle {
    std::vector<const EaxEdge*> graphDiffEdges;
    std::vector<int> circleStart;
    std::vector<uint8_t> firstGraphEdgeDiffFlags;
    std::vector<uint8_t> secondGraphEdgeDiffFlags;
};

struct SubtourInsertionData {
    int routeId{-1};
    int prevNodeId{-1};
    double deltaTwPenalty{0.0};
    double deltaCapacityViolations{0.0};
    double deltaDistance{0.0};
    bool newVehicle{false};
};

void eaxInitGraph(const VrpSolution& sol, EaxGraph& graph, uint8_t source);

bool calGraphCircle(EaxGraph& firstGraph, EaxGraph& secondGraph, GraphCircle& graphCircle);

void calInterGraph(const VrpSolution& firstSol, const EaxGraph& firstGraph, const GraphCircle& graphCircle,
                   std::vector<const EaxEdge*>& interGraphEdges, std::vector<const EaxEdge*>& interGraphNodeHeadEdges,
                   std::vector<const EaxEdge*>& interGraphNodeTailEdges,
                   std::vector<const EaxEdge*>& interGraphDepotHeadEdges,
                   std::vector<const EaxEdge*>& interGraphDepotTailEdges);

void constructInterSol(std::vector<const EaxEdge*>& interGraphEdges,
                       std::vector<const EaxEdge*>& interGraphNodeHeadEdges,
                       std::vector<const EaxEdge*>& interGraphNodeTailEdges,
                       std::vector<const EaxEdge*>& interGraphDepotHeadEdges,
                       std::vector<const EaxEdge*>& interGraphDepotTailEdges, VrpSolution& interSol,
                       std::vector<std::vector<int>>& subtours);

void calSubtourInsertionData(const VrpSolution& sol, const std::vector<int>& subtour, int breakPos, int routeId,
                             std::vector<SubtourInsertionData>& insertions);

void connectSubtours(const VrpConfig& vrpConfig, VrpSolution& interSol, std::vector<std::vector<int>>& subtours);

void eax(const VrpConfig& vrpConfig, const VrpSolution& firstSol, const VrpSolution& secondSol, VrpSolution& childSol);

#endif