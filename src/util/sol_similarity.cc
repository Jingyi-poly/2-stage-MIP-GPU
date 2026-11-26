#include "util/sol_similarity.h"

#include <cassert>

double calculateSimilarity(const VrpSolution& solution1, const VrpSolution& solution2) {
    int samePrevNodeNum = 0;
    for (int i = 1; i < solution1.vrpProblem->vrpData.nodeNum; ++i) {
        assert(solution1.routeNodes[i].preNode != nullptr);
        assert(solution2.routeNodes[i].preNode != nullptr);

        int firstPrevNodeId = 0;
        if (nullptr != solution1.routeNodes[i].preNode) {
            firstPrevNodeId = solution1.routeNodes[i].preNode->nodeId;
        }

        int secondPrevNodeId = 0;
        if (nullptr != solution2.routeNodes[i].preNode) {
            secondPrevNodeId = solution2.routeNodes[i].preNode->nodeId;
        }

        if (firstPrevNodeId == secondPrevNodeId) {
            samePrevNodeNum += 1;
        }
    }

    return samePrevNodeNum * 1.0 / (solution1.vrpProblem->vrpData.nodeNum - 1);
}
