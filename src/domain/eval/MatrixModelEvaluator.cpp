#include "domain/eval/MatrixModelEvaluator.h"

#include <algorithm>
#include <limits>

#include "domain/eval/ScenarioCapacityEvaluator.h"
#include "domain/model/VrpSolution.h"
#include "domain/model/entity/VrpNode.h"

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

MatrixModelEvaluator::MatrixModelEvaluator(const VrpSolution& solution) : solution_(solution) {
    // Build the solution matrix: one ragged row per non-empty route, holding the
    // route's node ids in visiting order (depot is implicit, exactly like the
    // VrpRoute startNode..endNode chain the solver maintains).
    for (const VrpRoute& route : solution_.routes) {
        if (route.nodeNum <= 0) {
            continue;
        }
        std::vector<int> row;
        row.reserve(route.nodeNum);
        for (const RouteNode* node = route.startNode; node != nullptr; node = node->nexNode) {
            row.push_back(node->nodeId);
        }
        maxRouteLen_ = std::max(maxRouteLen_, static_cast<int>(row.size()));
        nodeMat_.push_back(std::move(row));
    }
    routeCount_ = static_cast<int>(nodeMat_.size());
}

MatrixModelResult MatrixModelEvaluator::Evaluate() const {
    MatrixModelResult res = EvaluateDeterministic();
    res.capacityViolation = CapacityInlineHost();
    return res;
}

MatrixModelResult MatrixModelEvaluator::Evaluate(ScenarioCapacityEvaluator& capacityEvaluator) const {
    MatrixModelResult res = EvaluateDeterministic();

    // The capacity term is the same multi-scenario primitive the evaluator owns:
    // pack the solution matrix as a CSR route batch and let the (GPU) backend
    // return the per-route violation totals -- only RouteNum() scalars come back.
    RouteBatch batch;
    batch.routeOffsets.reserve(routeCount_ + 1);
    batch.routeOffsets.push_back(0);
    for (const std::vector<int>& row : nodeMat_) {
        batch.nodeIds.insert(batch.nodeIds.end(), row.begin(), row.end());
        batch.routeOffsets.push_back(static_cast<int>(batch.nodeIds.size()));
    }

    std::vector<double> routeViol;
    capacityEvaluator.EvaluateRouteViolations(batch, routeViol);
    double total = 0.0;
    for (double v : routeViol) {
        total += v;
    }
    res.capacityViolation = total;
    return res;
}

double MatrixModelEvaluator::CapacityInlineHost() const {
    const VrpData& data = solution_.vrpProblem->vrpData;
    const std::vector<VrpNode>& nodes = data.nodeList;
    const double capacity = data.vehicleCapacity;
    const int numScenarios = data.numScenarios;

    double capacityViolation = 0.0;
    std::vector<double> load(numScenarios, 0.0);  // per-scenario running load
    for (const std::vector<int>& row : nodeMat_) {
        // Per scenario, cumsum demand along the row, subtract capacity, relu,
        // reduce. Node-outer / scenario-inner grouping matches the solver's order.
        std::fill(load.begin(), load.end(), 0.0);
        for (int nodeId : row) {
            const std::vector<double>& demand = nodes[nodeId].demand;
            double nodeViol = 0.0;
            for (int s = 0; s < numScenarios; ++s) {
                load[s] += demand[s];
                nodeViol += std::max(0.0, load[s] - capacity);
            }
            capacityViolation += nodeViol;
        }
    }
    return capacityViolation;
}

MatrixModelResult MatrixModelEvaluator::EvaluateDeterministic() const {
    const VrpData& data = solution_.vrpProblem->vrpData;
    const std::vector<VrpNode>& nodes = data.nodeList;
    const std::vector<std::vector<double>>& dis = data.disMatrix;

    const double depotTwl = nodes[0].timeWindow.first;
    const double depotTwr = nodes[0].timeWindow.second;
    const double depotLeave = depotTwl + nodes[0].serveTime;

    MatrixModelResult res;
    res.routeCount = routeCount_;
    res.maxRouteLen = maxRouteLen_;

    for (const std::vector<int>& row : nodeMat_) {
        const int len = static_cast<int>(row.size());

        // ---- Distance: reduce-sum over the row's arc-in distances + return arc.
        {
            int prev = 0;  // depot
            for (int j = 0; j < len; ++j) {
                res.distance += dis[prev][row[j]];
                prev = row[j];
            }
            res.distance += dis[prev][0];
        }

        // ---- Time window, solver-exact (capped at twr): left-to-right scan.
        {
            int prev = 0;
            double prevLeave = depotLeave;
            double penalty = 0.0;
            for (int j = 0; j < len; ++j) {
                const VrpNode& nd = nodes[row[j]];
                const double arrive = prevLeave + dis[prev][row[j]];
                const double start = std::min(std::max(arrive, nd.timeWindow.first), nd.timeWindow.second);
                penalty += std::max(0.0, arrive - nd.timeWindow.second);
                prevLeave = start + nd.serveTime;
                prev = row[j];
            }
            penalty += std::max(0.0, prevLeave + dis[prev][0] - depotTwr);
            res.twViolationCapped += penalty;
        }

        // ---- Time window, pure slide-8 pipeline (no upper cap):
        //   A_j   = cumsum of (travel + predecessor service) from the depot
        //   D_j   = A_j + cummax_{k<=j} max(0, twl_k - A_k)   (waiting closed form)
        //   pen  += relu(D_j - twr_j),  plus the return-to-depot term.
        {
            int prev = 0;
            double a = depotLeave;     // running A (earliest arrival, no waiting)
            double runMax = 0.0;       // cummax of (twl_k - A_k), floored at 0
            double dLast = 0.0;        // earliest start with waiting at last node
            double penalty = 0.0;
            for (int j = 0; j < len; ++j) {
                const VrpNode& nd = nodes[row[j]];
                a += dis[prev][row[j]] + (j == 0 ? 0.0 : nodes[prev].serveTime);
                runMax = std::max(runMax, nd.timeWindow.first - a);
                const double d = a + runMax;
                penalty += std::max(0.0, d - nd.timeWindow.second);
                dLast = d;
                prev = row[j];
            }
            const double returnArrive = dLast + nodes[prev].serveTime + dis[prev][0];
            penalty += std::max(0.0, returnArrive - depotTwr);
            res.twViolationSlides += penalty;
        }
    }

    return res;
}
