#include "PCTSPLevelBBSCIP.h"

#include <scip/cons_linear.h>
#include <scip/scip.h>
#include <scip/scipdefplugins.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

void scipCheck(SCIP_RETCODE code, const char* operation)
{
    if (code == SCIP_OKAY)
        return;
    std::ostringstream message;
    message << operation << " failed with SCIP return code " << code;
    throw std::runtime_error(message.str());
}

struct SCIPOwner {
    SCIP* value = nullptr;

    SCIPOwner() = default;
    SCIPOwner(const SCIPOwner&) = delete;
    SCIPOwner& operator=(const SCIPOwner&) = delete;

    ~SCIPOwner()
    {
        if (value != nullptr)
            (void)SCIPfree(&value);
    }
};

double relativeGap(double lowerBound, double upperBound)
{
    if (!std::isfinite(lowerBound) || !std::isfinite(upperBound))
        return std::numeric_limits<double>::infinity();
    return std::max(0.0, upperBound - lowerBound)
        / std::max(1.0, std::abs(upperBound));
}

double gapTolerance(const PCTSPLevelBBOptions& options, double upperBound)
{
    return std::max(
        options.absoluteGapTolerance,
        options.relativeGapTolerance
            * std::max(1.0, std::abs(upperBound)));
}

bool gapClosed(
    const PCTSPLevelBBOptions& options,
    double lowerBound,
    double upperBound)
{
    if (!std::isfinite(lowerBound) || !std::isfinite(upperBound))
        return false;
    return upperBound - lowerBound <= gapTolerance(options, upperBound);
}

double secondsSince(Clock::time_point start)
{
    return std::chrono::duration<double>(Clock::now() - start).count();
}

struct BinaryLayout {
    explicit BinaryLayout(const Params& params)
        : nodeCount(params.nbClients + 1),
          arcIndex(nodeCount * nodeCount, -1),
          selectedIndex(nodeCount, -1)
    {
        int next = 0;
        for (int i = 0; i < nodeCount; ++i) {
            for (int j = 0; j < nodeCount; ++j) {
                if (i == j)
                    continue;
                arcIndex[index(i, j)] = next++;
                baseLower.push_back(0.0);
                baseUpper.push_back(1.0);
            }
        }
        for (int i = 1; i < nodeCount; ++i) {
            selectedIndex[i] = next++;
            baseLower.push_back(params.ap.optionalVisit ? 0.0 : 1.0);
            baseUpper.push_back(1.0);
        }
        activeIndex = next++;
        baseLower.push_back(params.ap.optionalVisit ? 0.0 : 1.0);
        baseUpper.push_back(1.0);
        variableCount = next;
    }

    int index(int i, int j) const { return i * nodeCount + j; }

    bool structurallyFree(int variable) const
    {
        return baseUpper[variable] - baseLower[variable] > 0.5;
    }

    int nodeCount;
    std::vector<int> arcIndex;
    std::vector<int> selectedIndex;
    int activeIndex = -1;
    int variableCount = 0;
    std::vector<double> baseLower;
    std::vector<double> baseUpper;
};

std::vector<double> makeAnchorCosts(const Params& params)
{
    const int nodeCount = params.nbClients + 1;
    std::vector<double> anchor(nodeCount * nodeCount, 0.0);
    for (int i = 0; i < nodeCount; ++i) {
        for (int j = 0; j < nodeCount; ++j) {
            if (i == j)
                continue;
            double value = params.timeCost[i][j];
            if (i != 0 && j != 0) {
                value = std::min(
                    value,
                    params.timeCost[i][0] + params.timeCost[0][j]);
            }
            if (!std::isfinite(value) || value < 0.0) {
                throw std::runtime_error(
                    "Level B&B distance anchor requires finite nonnegative costs");
            }
            anchor[i * nodeCount + j] = value;
        }
    }
    return anchor;
}

double tourAnchor(
    const std::vector<int>& tour,
    const std::vector<double>& anchor,
    int nodeCount)
{
    double value = 0.0;
    int from = 0;
    for (int to : tour) {
        value += anchor[from * nodeCount + to];
        from = to;
    }
    value += anchor[from * nodeCount];
    return value;
}

double totalSkipPenalty(const Params& params)
{
    double total = 0.0;
    for (int client = 1; client <= params.nbClients; ++client) {
        const double penalty = params.cli[client].skipPenalty;
        if (!std::isfinite(penalty) || penalty < 0.0) {
            throw std::runtime_error(
                "Level B&B requires finite nonnegative skip penalties");
        }
        total += penalty;
    }
    return total;
}

double skippedCost(
    const Params& params,
    const std::vector<int>& tour,
    double allSkippedCost)
{
    double value = allSkippedCost;
    std::vector<bool> seen(params.nbClients + 1, false);
    for (int client : tour) {
        if (client <= 0 || client > params.nbClients || seen[client])
            throw std::runtime_error("Level B&B received an invalid tour");
        seen[client] = true;
        value -= params.cli[client].skipPenalty;
    }
    return std::max(0.0, value);
}

struct TourCut {
    std::vector<int> tour;
    double residualLowerBound = 0.0;
};

struct BBNode {
    std::uint64_t id = 0;
    int depth = 0;
    std::vector<signed char> fixings;
    double inheritedLowerBound = 0.0;
};

enum class NodeLPStatus {
    Optimal,
    Infeasible,
    Interrupted,
};

struct NodeLPResult {
    NodeLPStatus status = NodeLPStatus::Interrupted;
    bool solveStarted = false;
    bool hitTimeLimit = false;
    double certifiedLowerBound = 0.0;
    double rawDualBound = -std::numeric_limits<double>::infinity();
    double appliedBoundSlack = 0.0;
    double solveSeconds = 0.0;
    double theta = 0.0;
    double residual = 0.0;
    double skipCost = 0.0;
    std::vector<double> binaryValues;
};

class ContinuousMaster {
public:
    ContinuousMaster(
        const Params& params,
        const BinaryLayout& layout,
        const std::vector<double>& anchor,
        const std::vector<TourCut>& cuts,
        const BBNode& node)
        : params(params), layout(layout), anchor(anchor), node(node)
    {
        scipCheck(SCIPcreate(&owner.value), "SCIPcreate(level LP)");
        scip = owner.value;
        scipCheck(
            SCIPincludeDefaultPlugins(scip),
            "SCIPincludeDefaultPlugins(level LP)");
        scipCheck(
            SCIPcreateProbBasic(scip, "pctsp_level_bb_node_lp"),
            "SCIPcreateProbBasic(level LP)");
        scipCheck(
            SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE),
            "SCIPsetObjsense(level LP)");
        scipCheck(
            SCIPsetIntParam(scip, "display/verblevel", 0),
            "SCIPsetIntParam(display/verblevel)");
        // The outer level wave owns parallelism.  Keep every independent SCIP
        // node-LP instance single-threaded to avoid hidden oversubscription.
        if (SCIPgetParam(scip, "parallel/maxnthreads") != nullptr) {
            scipCheck(
                SCIPsetIntParam(scip, "parallel/maxnthreads", 1),
                "SCIPsetIntParam(parallel/maxnthreads)");
        }
        if (SCIPgetParam(scip, "lp/threads") != nullptr) {
            scipCheck(
                SCIPsetIntParam(scip, "lp/threads", 1),
                "SCIPsetIntParam(lp/threads)");
        }
        buildStaticModel();
        for (std::size_t cutIndex = 0; cutIndex < cuts.size(); ++cutIndex)
            addTourCut(cuts[cutIndex], cutIndex);
    }

    NodeLPResult solve(
        const PCTSPLevelBBOptions& options,
        bool limited,
        Clock::time_point deadline)
    {
        NodeLPResult result;
        result.certifiedLowerBound = node.inheritedLowerBound;
        if (limited) {
            const double remaining = std::chrono::duration<double>(
                deadline - Clock::now()).count();
            if (remaining <= 1e-6) {
                result.hitTimeLimit = true;
                return result;
            }
            scipCheck(
                SCIPsetRealParam(scip, "limits/time", remaining),
                "SCIPsetRealParam(level LP time)");
        }

        const auto solveStart = Clock::now();
        result.solveStarted = true;
        scipCheck(SCIPsolve(scip), "SCIPsolve(level LP)");
        result.solveSeconds = secondsSince(solveStart);

        const SCIP_STATUS status = SCIPgetStatus(scip);
        if (status == SCIP_STATUS_INFEASIBLE) {
            result.status = NodeLPStatus::Infeasible;
            return result;
        }
        if (status == SCIP_STATUS_TIMELIMIT) {
            result.status = NodeLPStatus::Interrupted;
            result.hitTimeLimit = true;
            return result;
        }
        if (status == SCIP_STATUS_UNBOUNDED
            || status == SCIP_STATUS_INFORUNBD) {
            throw std::runtime_error(
                "Level B&B node LP is unexpectedly unbounded");
        }
        if (status != SCIP_STATUS_OPTIMAL) {
            std::ostringstream message;
            message << "Level B&B node LP stopped with SCIP status " << status;
            throw std::runtime_error(message.str());
        }

        SCIP_SOL* solution = SCIPgetBestSol(scip);
        if (solution == nullptr)
            throw std::runtime_error("Optimal level B&B node LP has no solution");

        result.rawDualBound = SCIPgetDualbound(scip);
        if (!std::isfinite(result.rawDualBound))
            throw std::runtime_error("Level B&B node LP has a non-finite bound");
        result.appliedBoundSlack = options.lpBoundSafetySlack
            * std::max(1.0, std::abs(result.rawDualBound));
        result.certifiedLowerBound = std::max(
            node.inheritedLowerBound,
            result.rawDualBound - result.appliedBoundSlack);
        result.theta = SCIPgetSolVal(scip, solution, theta);
        result.residual = SCIPgetSolVal(scip, solution, residual);
        result.skipCost = SCIPgetSolVal(scip, solution, skipCostVariable);
        result.binaryValues.resize(layout.variableCount);
        for (int variable = 0; variable < layout.variableCount; ++variable) {
            result.binaryValues[variable] =
                SCIPgetSolVal(scip, solution, binaryVariables[variable]);
        }
        result.status = NodeLPStatus::Optimal;
        return result;
    }

private:
    SCIP_VAR* addContinuousVariable(
        const std::string& name,
        double lowerBound,
        double upperBound,
        double objective)
    {
        SCIP_VAR* captured = nullptr;
        scipCheck(
            SCIPcreateVarBasic(
                scip, &captured, name.c_str(), lowerBound, upperBound,
                objective, SCIP_VARTYPE_CONTINUOUS),
            "SCIPcreateVarBasic(level LP)");
        try {
            scipCheck(SCIPaddVar(scip, captured), "SCIPaddVar(level LP)");
        } catch (...) {
            (void)SCIPreleaseVar(scip, &captured);
            throw;
        }
        SCIP_VAR* raw = captured;
        scipCheck(
            SCIPreleaseVar(scip, &captured),
            "SCIPreleaseVar(level LP builder capture)");
        return raw;
    }

    void addLinear(
        const std::string& name,
        const std::vector<SCIP_VAR*>& variables,
        const std::vector<SCIP_Real>& coefficients,
        SCIP_Real lhs,
        SCIP_Real rhs)
    {
        if (variables.size() != coefficients.size())
            throw std::logic_error("Level B&B linear constraint size mismatch");
        SCIP_CONS* constraint = nullptr;
        scipCheck(
            SCIPcreateConsBasicLinear(
                scip, &constraint, name.c_str(),
                static_cast<int>(variables.size()),
                const_cast<SCIP_VAR**>(variables.data()),
                const_cast<SCIP_Real*>(coefficients.data()), lhs, rhs),
            "SCIPcreateConsBasicLinear(level LP)");
        try {
            scipCheck(
                SCIPaddCons(scip, constraint),
                "SCIPaddCons(level LP)");
        } catch (...) {
            (void)SCIPreleaseCons(scip, &constraint);
            throw;
        }
        scipCheck(
            SCIPreleaseCons(scip, &constraint),
            "SCIPreleaseCons(level LP)");
    }

    std::pair<double, double> binaryBounds(int variable) const
    {
        double lower = layout.baseLower[variable];
        double upper = layout.baseUpper[variable];
        const signed char fixing = node.fixings[variable];
        if (fixing == 0 || fixing == 1) {
            lower = std::max(lower, static_cast<double>(fixing));
            upper = std::min(upper, static_cast<double>(fixing));
        }
        if (lower > upper) {
            throw std::logic_error(
                "Level B&B generated a node conflicting with structural bounds");
        }
        return {lower, upper};
    }

    void buildStaticModel()
    {
        const int nodeCount = layout.nodeCount;
        const int clientCount = nodeCount - 1;
        arc.assign(nodeCount * nodeCount, nullptr);
        flow.assign(nodeCount * nodeCount, nullptr);
        selected.assign(nodeCount, nullptr);
        binaryVariables.assign(layout.variableCount, nullptr);

        for (int i = 0; i < nodeCount; ++i) {
            for (int j = 0; j < nodeCount; ++j) {
                if (i == j)
                    continue;
                const int binary = layout.arcIndex[layout.index(i, j)];
                const auto bounds = binaryBounds(binary);
                arc[layout.index(i, j)] = addContinuousVariable(
                    "x_" + std::to_string(i) + '_' + std::to_string(j),
                    bounds.first, bounds.second, 0.0);
                binaryVariables[binary] = arc[layout.index(i, j)];
                flow[layout.index(i, j)] = addContinuousVariable(
                    "f_" + std::to_string(i) + '_' + std::to_string(j),
                    0.0, static_cast<double>(clientCount), 0.0);
            }
        }

        for (int i = 1; i < nodeCount; ++i) {
            const int binary = layout.selectedIndex[i];
            const auto bounds = binaryBounds(binary);
            selected[i] = addContinuousVariable(
                "y_" + std::to_string(i),
                bounds.first, bounds.second, 0.0);
            binaryVariables[binary] = selected[i];
        }
        {
            const auto bounds = binaryBounds(layout.activeIndex);
            active = addContinuousVariable(
                "active_tour", bounds.first, bounds.second, 0.0);
            binaryVariables[layout.activeIndex] = active;
        }
        theta = addContinuousVariable(
            "theta", 0.0, SCIPinfinity(scip), 1.0);
        residual = addContinuousVariable(
            "split_residual", 0.0, SCIPinfinity(scip), 0.0);
        const double allSkippedCost = totalSkipPenalty(params);
        skipCostVariable = addContinuousVariable(
            "skip_cost", 0.0, allSkippedCost, 1.0);

        for (int i = 1; i < nodeCount; ++i) {
            std::vector<SCIP_VAR*> outgoing;
            std::vector<SCIP_VAR*> incoming;
            std::vector<SCIP_Real> outgoingValues;
            std::vector<SCIP_Real> incomingValues;
            for (int j = 0; j < nodeCount; ++j) {
                if (i == j)
                    continue;
                outgoing.push_back(arc[layout.index(i, j)]);
                outgoingValues.push_back(1.0);
                incoming.push_back(arc[layout.index(j, i)]);
                incomingValues.push_back(1.0);
            }
            outgoing.push_back(selected[i]);
            outgoingValues.push_back(-1.0);
            incoming.push_back(selected[i]);
            incomingValues.push_back(-1.0);
            addLinear(
                "out_degree_" + std::to_string(i),
                outgoing, outgoingValues, 0.0, 0.0);
            addLinear(
                "in_degree_" + std::to_string(i),
                incoming, incomingValues, 0.0, 0.0);
            addLinear(
                "selection_implies_active_" + std::to_string(i),
                std::vector<SCIP_VAR*>{selected[i], active},
                std::vector<SCIP_Real>{1.0, -1.0},
                -SCIPinfinity(scip), 0.0);
        }

        std::vector<SCIP_VAR*> depotOutgoing;
        std::vector<SCIP_VAR*> depotIncoming;
        std::vector<SCIP_Real> depotOutgoingValues;
        std::vector<SCIP_Real> depotIncomingValues;
        for (int j = 1; j < nodeCount; ++j) {
            depotOutgoing.push_back(arc[layout.index(0, j)]);
            depotOutgoingValues.push_back(1.0);
            depotIncoming.push_back(arc[layout.index(j, 0)]);
            depotIncomingValues.push_back(1.0);
        }
        depotOutgoing.push_back(active);
        depotOutgoingValues.push_back(-1.0);
        depotIncoming.push_back(active);
        depotIncomingValues.push_back(-1.0);
        addLinear(
            "depot_out_degree", depotOutgoing, depotOutgoingValues, 0.0, 0.0);
        addLinear(
            "depot_in_degree", depotIncoming, depotIncomingValues, 0.0, 0.0);

        for (int i = 0; i < nodeCount; ++i) {
            std::vector<SCIP_VAR*> variables;
            std::vector<SCIP_Real> coefficients;
            for (int j = 0; j < nodeCount; ++j) {
                if (i == j)
                    continue;
                variables.push_back(flow[layout.index(i, j)]);
                coefficients.push_back(i == 0 ? 1.0 : -1.0);
                variables.push_back(flow[layout.index(j, i)]);
                coefficients.push_back(i == 0 ? -1.0 : 1.0);
            }
            if (i == 0) {
                for (int client = 1; client < nodeCount; ++client) {
                    variables.push_back(selected[client]);
                    coefficients.push_back(-1.0);
                }
            } else {
                variables.push_back(selected[i]);
                coefficients.push_back(-1.0);
            }
            addLinear(
                "flow_balance_" + std::to_string(i),
                variables, coefficients, 0.0, 0.0);
        }

        {
            std::vector<SCIP_VAR*> variables{skipCostVariable};
            std::vector<SCIP_Real> coefficients{1.0};
            for (int i = 1; i < nodeCount; ++i) {
                variables.push_back(selected[i]);
                coefficients.push_back(params.cli[i].skipPenalty);
            }
            addLinear(
                "skip_cost_definition", variables, coefficients,
                allSkippedCost, allSkippedCost);
        }

        for (int i = 0; i < nodeCount; ++i) {
            for (int j = 0; j < nodeCount; ++j) {
                if (i == j)
                    continue;
                addLinear(
                    "flow_link_" + std::to_string(i) + '_'
                        + std::to_string(j),
                    std::vector<SCIP_VAR*>{
                        flow[layout.index(i, j)], arc[layout.index(i, j)]},
                    std::vector<SCIP_Real>{
                        1.0, -static_cast<double>(clientCount)},
                    -SCIPinfinity(scip), 0.0);
            }
        }

        {
            std::vector<SCIP_VAR*> variables{theta, residual};
            std::vector<SCIP_Real> coefficients{1.0, -1.0};
            for (int i = 0; i < nodeCount; ++i) {
                for (int j = 0; j < nodeCount; ++j) {
                    if (i == j)
                        continue;
                    variables.push_back(arc[layout.index(i, j)]);
                    coefficients.push_back(-anchor[layout.index(i, j)]);
                }
            }
            addLinear(
                "distance_anchor", variables, coefficients, 0.0, 0.0);
        }
    }

    void addTourCut(const TourCut& cut, std::size_t cutIndex)
    {
        if (cut.residualLowerBound <= 0.0)
            return;
        const int nodeCount = layout.nodeCount;
        const double residualCost = cut.residualLowerBound;
        std::vector<SCIP_VAR*> variables{residual};
        std::vector<SCIP_Real> coefficients{1.0};
        std::vector<bool> inTour(nodeCount, false);
        int from = 0;
        for (int to : cut.tour) {
            if (to <= 0 || to >= nodeCount || inTour[to])
                throw std::logic_error("Invalid tour in level B&B cut pool");
            inTour[to] = true;
            variables.push_back(arc[layout.index(from, to)]);
            coefficients.push_back(-residualCost);
            from = to;
        }
        variables.push_back(arc[layout.index(from, 0)]);
        coefficients.push_back(-residualCost);
        for (int client = 1; client < nodeCount; ++client) {
            if (!inTour[client]) {
                variables.push_back(selected[client]);
                coefficients.push_back(residualCost);
            }
        }
        const int tourArcCount = static_cast<int>(cut.tour.size()) + 1;
        const double lhs = residualCost * (1 - tourArcCount);
        addLinear(
            "level_tour_cut_" + std::to_string(cutIndex),
            variables, coefficients, lhs, SCIPinfinity(scip));
    }

    const Params& params;
    const BinaryLayout& layout;
    const std::vector<double>& anchor;
    const BBNode& node;
    SCIPOwner owner;
    SCIP* scip = nullptr;
    std::vector<SCIP_VAR*> arc;
    std::vector<SCIP_VAR*> flow;
    std::vector<SCIP_VAR*> selected;
    std::vector<SCIP_VAR*> binaryVariables;
    SCIP_VAR* active = nullptr;
    SCIP_VAR* theta = nullptr;
    SCIP_VAR* residual = nullptr;
    SCIP_VAR* skipCostVariable = nullptr;
};

NodeLPResult solveOneNode(
    const Params& params,
    const BinaryLayout& layout,
    const std::vector<double>& anchor,
    const std::vector<TourCut>& cuts,
    const BBNode& node,
    const PCTSPLevelBBOptions& options,
    bool limited,
    Clock::time_point deadline)
{
    if (limited && Clock::now() >= deadline) {
        NodeLPResult result;
        result.hitTimeLimit = true;
        result.certifiedLowerBound = node.inheritedLowerBound;
        return result;
    }
    ContinuousMaster master(params, layout, anchor, cuts, node);
    return master.solve(options, limited, deadline);
}

std::vector<NodeLPResult> solveNodeWave(
    const Params& params,
    const BinaryLayout& layout,
    const std::vector<double>& anchor,
    const std::vector<TourCut>& cuts,
    const std::vector<BBNode>& nodes,
    const PCTSPLevelBBOptions& options,
    bool limited,
    Clock::time_point deadline)
{
    std::vector<NodeLPResult> results(nodes.size());
    if (nodes.empty())
        return results;

    const int threadCount = std::max(
        1, std::min(options.nodeParallelism, static_cast<int>(nodes.size())));
    if (threadCount == 1) {
        for (std::size_t index = 0; index < nodes.size(); ++index) {
            results[index] = solveOneNode(
                params, layout, anchor, cuts, nodes[index], options,
                limited, deadline);
        }
        return results;
    }

    std::atomic<std::size_t> nextIndex{0};
    std::vector<std::exception_ptr> errors(nodes.size());
    std::vector<std::thread> workers;
    workers.reserve(threadCount);
    for (int worker = 0; worker < threadCount; ++worker) {
        workers.emplace_back([&]() {
            while (true) {
                const std::size_t index = nextIndex.fetch_add(1);
                if (index >= nodes.size())
                    return;
                try {
                    results[index] = solveOneNode(
                        params, layout, anchor, cuts, nodes[index], options,
                        limited, deadline);
                } catch (...) {
                    errors[index] = std::current_exception();
                }
            }
        });
    }
    for (std::thread& worker : workers)
        worker.join();
    for (const std::exception_ptr& error : errors) {
        if (error)
            std::rethrow_exception(error);
    }
    return results;
}

bool firstStageIntegral(
    const std::vector<double>& values,
    double tolerance)
{
    for (double value : values) {
        if (std::abs(value) > tolerance
            && std::abs(value - 1.0) > tolerance) {
            return false;
        }
    }
    return true;
}

int selectFractionalBranchVariable(
    const BBNode& node,
    const BinaryLayout& layout,
    const std::vector<double>& values,
    double tolerance)
{
    int best = -1;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (int variable = 0; variable < layout.variableCount; ++variable) {
        if (!layout.structurallyFree(variable)
            || node.fixings[variable] != -1) {
            continue;
        }
        const double value = values[variable];
        if (std::abs(value) <= tolerance
            || std::abs(value - 1.0) <= tolerance) {
            continue;
        }
        const double distance = std::abs(value - 0.5);
        if (distance < bestDistance - 1e-15
            || (std::abs(distance - bestDistance) <= 1e-15
                && (best < 0 || variable < best))) {
            best = variable;
            bestDistance = distance;
        }
    }
    return best;
}

int selectUnfixedVariable(
    const BBNode& node,
    const BinaryLayout& layout)
{
    for (int variable = 0; variable < layout.variableCount; ++variable) {
        if (layout.structurallyFree(variable)
            && node.fixings[variable] == -1) {
            return variable;
        }
    }
    return -1;
}

struct IntegralCandidate {
    std::vector<int> tour;
    double skipCost = 0.0;
    double anchor = 0.0;
    double masterTheta = 0.0;
};

IntegralCandidate extractIntegralCandidate(
    const Params& params,
    const BinaryLayout& layout,
    const std::vector<double>& anchorCosts,
    double allSkippedCost,
    const NodeLPResult& result)
{
    const int nodeCount = layout.nodeCount;
    std::vector<bool> selected(nodeCount, false);
    int selectedCount = 0;
    for (int client = 1; client < nodeCount; ++client) {
        if (result.binaryValues[layout.selectedIndex[client]] > 0.5) {
            selected[client] = true;
            ++selectedCount;
        }
    }
    const bool active =
        result.binaryValues[layout.activeIndex] > 0.5;
    if (active != (selectedCount > 0)) {
        throw std::runtime_error(
            "Integral level B&B LP has inconsistent tour activation");
    }

    std::vector<int> successor(nodeCount, -1);
    for (int i = 0; i < nodeCount; ++i) {
        for (int j = 0; j < nodeCount; ++j) {
            if (i == j)
                continue;
            if (result.binaryValues[layout.arcIndex[layout.index(i, j)]] > 0.5) {
                if (successor[i] != -1) {
                    throw std::runtime_error(
                        "Integral level B&B LP has multiple outgoing arcs");
                }
                successor[i] = j;
            }
        }
    }

    IntegralCandidate candidate;
    candidate.masterTheta = result.theta;
    candidate.tour.reserve(selectedCount);
    if (selectedCount == 0) {
        for (int successorNode : successor) {
            if (successorNode != -1) {
                throw std::runtime_error(
                    "Integral empty level B&B LP unexpectedly uses a tour arc");
            }
        }
    } else {
        std::vector<bool> seen(nodeCount, false);
        seen[0] = true;
        int current = 0;
        for (int step = 0; step <= selectedCount; ++step) {
            const int next = successor[current];
            if (next < 0 || next >= nodeCount) {
                throw std::runtime_error(
                    "Integral level B&B LP is missing a tour arc");
            }
            if (step < selectedCount) {
                if (next == 0 || !selected[next] || seen[next]) {
                    throw std::runtime_error(
                        "Integral level B&B LP has a disconnected tour");
                }
                seen[next] = true;
                candidate.tour.push_back(next);
            } else if (next != 0) {
                throw std::runtime_error(
                    "Integral level B&B tour does not return to the depot");
            }
            current = next;
        }
    }
    if (static_cast<int>(candidate.tour.size()) != selectedCount)
        throw std::runtime_error("Integral level B&B tour omits a client");

    candidate.anchor = tourAnchor(
        candidate.tour, anchorCosts, nodeCount);
    candidate.skipCost = skippedCost(
        params, candidate.tour, allSkippedCost);
    const double skipTolerance = 1e-7 * std::max(1.0, allSkippedCost);
    if (std::abs(candidate.skipCost - result.skipCost) > skipTolerance) {
        throw std::runtime_error(
            "Integral level B&B LP has an inconsistent skip cost");
    }
    return candidate;
}

struct OracleRecord {
    double pointEstimate = 0.0;
    double lowerBound = 0.0;
    double upperBound = 0.0;
    double appliedSlack = 0.0;
    double anchor = 0.0;
    double skipCost = 0.0;
    bool cutAdded = false;
};

double frontierLowerBound(
    const std::vector<BBNode>& frontier,
    double upperBound)
{
    if (frontier.empty())
        return std::isfinite(upperBound)
            ? upperBound
            : std::numeric_limits<double>::infinity();
    double lowerBound = std::numeric_limits<double>::infinity();
    for (const BBNode& node : frontier)
        lowerBound = std::min(lowerBound, node.inheritedLowerBound);
    return lowerBound;
}

void validateOptions(const PCTSPLevelBBOptions& options)
{
    if (options.timeLimit < 0.0
        || !std::isfinite(options.timeLimit)) {
        throw std::invalid_argument(
            "Level B&B time limit must be finite and nonnegative");
    }
    if (options.evaluationBatchSize < 1)
        throw std::invalid_argument("Level B&B batch size must be positive");
    if (options.nodeParallelism < 1)
        throw std::invalid_argument("Level B&B parallelism must be positive");
    const double nonnegativeValues[] = {
        options.oracleAbsoluteSlack,
        options.oracleRelativeSlack,
        options.absoluteGapTolerance,
        options.relativeGapTolerance,
        options.integralityTolerance,
        options.lpBoundSafetySlack,
    };
    for (double value : nonnegativeValues) {
        if (!std::isfinite(value) || value < 0.0) {
            throw std::invalid_argument(
                "Level B&B tolerances/slacks must be finite and nonnegative");
        }
    }
    if (options.integralityTolerance >= 0.5)
        throw std::invalid_argument(
            "Level B&B integrality tolerance must be below 0.5");
}

} // namespace

PCTSPLevelBBSCIP::PCTSPLevelBBSCIP(
    const Params& params,
    PCTSPLevelBBOptions options)
    : params(params), options(std::move(options))
{
    validateOptions(this->options);
    if (params.nbClients < 1)
        throw std::invalid_argument("Level B&B requires at least one client");
    if (params.n_scenarios <= 0)
        throw std::invalid_argument(
            "Level B&B requires generated stochastic demand scenarios");
    if (params.isDurationConstraint) {
        throw std::invalid_argument(
            "Level B&B currently supports capacity-penalized Split without duration constraints");
    }
}

PCTSPResult PCTSPLevelBBSCIP::solveFullStochasticMILPBatch(
    TourBatchEvalFunc batchEval)
{
    if (!batchEval)
        throw std::invalid_argument("Level B&B requires a tour batch evaluator");

    const auto start = Clock::now();
    const bool limited = options.timeLimit > 0.0;
    const Clock::time_point deadline = limited
        ? start + std::chrono::duration_cast<Clock::duration>(
              std::chrono::duration<double>(options.timeLimit))
        : Clock::time_point::max();

    lastStats = PCTSPLevelBBStats{};
    lastTrace.clear();
    lastStats.requestedBatchSize = options.evaluationBatchSize;
    lastStats.nodeParallelism = options.nodeParallelism;
    lastStats.oracleAbsoluteSlack = options.oracleAbsoluteSlack;
    lastStats.oracleRelativeSlack = options.oracleRelativeSlack;

    const BinaryLayout layout(params);
    const std::vector<double> anchorCosts = makeAnchorCosts(params);
    const double allSkippedCost = totalSkipPenalty(params);

    std::vector<TourCut> cutPool;
    std::map<std::vector<int>, OracleRecord> oracleCache;
    // The empty selected set has analytically exact zero recourse and does not
    // consume evaluator capacity.
    oracleCache.emplace(
        std::vector<int>{},
        OracleRecord{0.0, 0.0, 0.0, 0.0, 0.0, allSkippedCost, true});

    double bestUpperBound = std::numeric_limits<double>::infinity();
    std::vector<int> bestTour;
    bool bestFound = false;
    if (params.ap.optionalVisit) {
        bestUpperBound = allSkippedCost;
        bestTour.clear();
        bestFound = true;
    }

    std::uint64_t nextNodeId = 1;
    BBNode root;
    root.id = 0;
    root.depth = 0;
    root.fixings.assign(layout.variableCount, static_cast<signed char>(-1));
    root.inheritedLowerBound = 0.0;
    std::vector<BBNode> frontier{std::move(root)};
    lastStats.nodesCreated = 1;

    auto refreshBounds = [&]() {
        lastStats.openNodes = frontier.size();
        lastStats.lowerBound = frontierLowerBound(frontier, bestUpperBound);
        lastStats.upperBound = bestUpperBound;
        lastStats.relativeGap = relativeGap(
            lastStats.lowerBound, lastStats.upperBound);
    };
    auto recordTrace = [&](int depth) {
        refreshBounds();
        lastTrace.push_back(PCTSPLevelBBTracePoint{
            secondsSince(start),
            depth,
            lastStats.cutEpochs,
            lastStats.openNodes,
            lastStats.nodeLPSolves,
            lastStats.evaluatedTours,
            lastStats.optimalityCuts,
            lastStats.lowerBound,
            lastStats.upperBound,
            lastStats.relativeGap,
            lastStats.levelLPWallTime,
            lastStats.evaluationTime,
        });
    };

    bool proofReached = false;
    bool stop = false;
    while (!frontier.empty() && !stop) {
        std::sort(
            frontier.begin(), frontier.end(),
            [](const BBNode& left, const BBNode& right) {
                return left.id < right.id;
            });
        const int depth = frontier.front().depth;
        for (const BBNode& node : frontier) {
            if (node.depth != depth) {
                throw std::logic_error(
                    "Level B&B frontier contains more than one depth");
            }
        }
        lastStats.currentDepth = depth;

        if (limited && Clock::now() >= deadline) {
            lastStats.hitTimeLimit = true;
            recordTrace(depth);
            break;
        }

        ++lastStats.cutEpochs;
        const auto waveStart = Clock::now();
        std::vector<NodeLPResult> results = solveNodeWave(
            params, layout, anchorCosts, cutPool, frontier, options,
            limited, deadline);
        lastStats.levelLPWallTime += secondsSince(waveStart);

        bool incompleteWave = false;
        std::vector<bool> removed(frontier.size(), false);
        for (std::size_t index = 0; index < frontier.size(); ++index) {
            NodeLPResult& result = results[index];
            if (result.solveStarted) {
                ++lastStats.nodeLPSolves;
                lastStats.nodeLPSolveTime += result.solveSeconds;
            }
            lastStats.maxAppliedLPBoundSlack = std::max(
                lastStats.maxAppliedLPBoundSlack,
                result.appliedBoundSlack);
            if (result.status == NodeLPStatus::Infeasible) {
                removed[index] = true;
                ++lastStats.nodesInfeasible;
                continue;
            }
            if (result.status == NodeLPStatus::Interrupted) {
                ++lastStats.incompleteNodeLPSolves;
                incompleteWave = true;
                lastStats.hitTimeLimit =
                    lastStats.hitTimeLimit || result.hitTimeLimit;
                continue;
            }
            frontier[index].inheritedLowerBound = std::max(
                frontier[index].inheritedLowerBound,
                result.certifiedLowerBound);
            // This is an exact dominance prune, not a tolerance prune.  Gap
            // tolerance is handled separately so the reported LB stays honest.
            if (std::isfinite(bestUpperBound)
                && result.certifiedLowerBound >= bestUpperBound) {
                removed[index] = true;
                ++lastStats.nodesBoundPruned;
            }
        }

        if (incompleteWave) {
            std::vector<BBNode> kept;
            kept.reserve(frontier.size());
            for (std::size_t index = 0; index < frontier.size(); ++index) {
                if (!removed[index])
                    kept.push_back(std::move(frontier[index]));
            }
            frontier = std::move(kept);
            recordTrace(depth);
            stop = true;
            break;
        }

        std::vector<bool> isIntegral(frontier.size(), false);
        std::vector<IntegralCandidate> nodeCandidate(frontier.size());
        std::map<std::vector<int>, IntegralCandidate> candidates;
        for (std::size_t index = 0; index < frontier.size(); ++index) {
            if (removed[index])
                continue;
            isIntegral[index] = firstStageIntegral(
                results[index].binaryValues, options.integralityTolerance);
            if (!isIntegral[index])
                continue;
            nodeCandidate[index] = extractIntegralCandidate(
                params, layout, anchorCosts, allSkippedCost, results[index]);
            candidates.emplace(
                nodeCandidate[index].tour, nodeCandidate[index]);
        }

        std::vector<std::vector<int>> unseenTours;
        unseenTours.reserve(candidates.size());
        for (const auto& entry : candidates) {
            if (oracleCache.find(entry.first) == oracleCache.end())
                unseenTours.push_back(entry.first);
        }

        bool evaluationInterrupted = false;
        for (std::size_t begin = 0; begin < unseenTours.size();
             begin += static_cast<std::size_t>(options.evaluationBatchSize)) {
            if (limited && Clock::now() >= deadline) {
                lastStats.hitTimeLimit = true;
                evaluationInterrupted = true;
                break;
            }
            const std::size_t end = std::min(
                unseenTours.size(),
                begin + static_cast<std::size_t>(options.evaluationBatchSize));
            std::vector<std::vector<int>> batch(
                unseenTours.begin() + static_cast<std::ptrdiff_t>(begin),
                unseenTours.begin() + static_cast<std::ptrdiff_t>(end));
            const auto evaluationStart = Clock::now();
            const std::vector<double> pointEstimates = batchEval(batch);
            lastStats.evaluationTime += secondsSince(evaluationStart);
            if (pointEstimates.size() != batch.size()) {
                throw std::runtime_error(
                    "Level B&B batch evaluator returned the wrong number of costs");
            }
            ++lastStats.evaluationBatches;
            lastStats.evaluatedTours += static_cast<int>(batch.size());
            lastStats.maxActualBatchSize = std::max(
                lastStats.maxActualBatchSize,
                static_cast<int>(batch.size()));

            for (std::size_t local = 0; local < batch.size(); ++local) {
                const double point = pointEstimates[local];
                if (!std::isfinite(point) || point < 0.0) {
                    throw std::runtime_error(
                        "Level B&B evaluator returned a non-finite or negative cost");
                }
                const auto candidatePosition = candidates.find(batch[local]);
                if (candidatePosition == candidates.end())
                    throw std::logic_error("Level B&B lost an evaluator candidate");
                const IntegralCandidate& candidate = candidatePosition->second;
                const double slack = options.oracleAbsoluteSlack
                    + options.oracleRelativeSlack
                        * std::max(1.0, std::abs(point));
                // The anchor is a lower bound, not a feasible upper bound.
                // Never use it to mask an oracle interval that falls below
                // the anchor: that could make the incumbent non-feasible.
                if (point + slack < candidate.anchor) {
                    throw std::runtime_error(
                        "Level B&B oracle interval violates the distance anchor");
                }
                OracleRecord record;
                record.pointEstimate = point;
                record.appliedSlack = slack;
                record.anchor = candidate.anchor;
                record.skipCost = candidate.skipCost;
                record.lowerBound = std::max(candidate.anchor, point - slack);
                record.upperBound = std::max(candidate.anchor, point + slack);
                oracleCache.emplace(batch[local], record);
                lastStats.maxAppliedOracleSlack = std::max(
                    lastStats.maxAppliedOracleSlack, slack);
                lastStats.totalAppliedOracleSlack += slack;
            }
            if (limited && Clock::now() >= deadline) {
                lastStats.hitTimeLimit = true;
                // A non-cancellable oracle call may finish slightly after the
                // deadline.  Retain its valid incumbent/cuts, but stop this
                // solve instead of claiming a proof obtained after the budget.
                evaluationInterrupted = true;
                break;
            }
        }

        bool addedCut = false;
        for (const auto& entry : candidates) {
            auto recordPosition = oracleCache.find(entry.first);
            if (recordPosition == oracleCache.end())
                continue;
            OracleRecord& record = recordPosition->second;
            const double candidateUpper = record.skipCost + record.upperBound;
            const double tieTolerance = 1e-12
                * std::max(1.0, std::abs(candidateUpper));
            if (!bestFound
                || candidateUpper < bestUpperBound - tieTolerance
                || (candidateUpper <= bestUpperBound
                    && std::abs(candidateUpper - bestUpperBound) <= tieTolerance
                    && entry.first < bestTour)) {
                bestUpperBound = candidateUpper;
                bestTour = entry.first;
                bestFound = true;
            }

            const double residualLower = std::max(
                0.0, record.lowerBound - record.anchor);
            if (!record.cutAdded && residualLower > 0.0) {
                cutPool.push_back(TourCut{entry.first, residualLower});
                record.cutAdded = true;
                ++lastStats.optimalityCuts;
                addedCut = true;
            }
        }

        // A stronger incumbent found by the batch can prune additional nodes.
        for (std::size_t index = 0; index < frontier.size(); ++index) {
            if (removed[index])
                continue;
            if (std::isfinite(bestUpperBound)
                && results[index].certifiedLowerBound >= bestUpperBound) {
                removed[index] = true;
                if (isIntegral[index])
                    ++lastStats.nodesFathomed;
                else
                    ++lastStats.nodesBoundPruned;
            }
        }

        std::vector<BBNode> survivors;
        std::vector<NodeLPResult> survivorResults;
        std::vector<bool> survivorIntegral;
        survivors.reserve(frontier.size());
        survivorResults.reserve(frontier.size());
        survivorIntegral.reserve(frontier.size());
        for (std::size_t index = 0; index < frontier.size(); ++index) {
            if (removed[index])
                continue;
            survivors.push_back(std::move(frontier[index]));
            survivorResults.push_back(std::move(results[index]));
            survivorIntegral.push_back(isIntegral[index]);
        }
        frontier = std::move(survivors);
        results = std::move(survivorResults);
        isIntegral = std::move(survivorIntegral);

        if (evaluationInterrupted) {
            recordTrace(depth);
            stop = true;
            break;
        }

        refreshBounds();
        if (frontier.empty()
            || gapClosed(options, lastStats.lowerBound, bestUpperBound)) {
            proofReached = bestFound;
            lastStats.levelsCompleted = std::max(
                lastStats.levelsCompleted, depth + 1);
            recordTrace(depth);
            break;
        }

        if (addedCut) {
            // Every surviving node at this depth is re-solved against the new
            // global cut epoch before any branching decision is accepted.
            recordTrace(depth);
            continue;
        }

        std::vector<BBNode> children;
        children.reserve(2 * frontier.size());
        std::vector<BBNode> unresolvedLeaves;
        for (std::size_t index = 0; index < frontier.size(); ++index) {
            int branchVariable = -1;
            if (isIntegral[index]) {
                // With a nonzero oracle interval, an integral LP can remain
                // below its certified feasible upper value.  Branching on an
                // otherwise free binary variable explores alternatives without
                // accepting that point as exact.  A fully fixed unresolved leaf
                // is retained in the certificate and stops with a partial gap.
                branchVariable = selectUnfixedVariable(frontier[index], layout);
                if (branchVariable < 0) {
                    unresolvedLeaves.push_back(std::move(frontier[index]));
                    lastStats.oracleSlackPreventedProof = true;
                    continue;
                }
            } else {
                branchVariable = selectFractionalBranchVariable(
                    frontier[index], layout, results[index].binaryValues,
                    options.integralityTolerance);
                if (branchVariable < 0) {
                    throw std::runtime_error(
                        "Fractional level B&B LP has no eligible branch variable");
                }
            }

            for (signed char value = 0; value <= 1; ++value) {
                BBNode child;
                child.id = nextNodeId++;
                child.depth = depth + 1;
                child.fixings = frontier[index].fixings;
                child.fixings[branchVariable] = value;
                child.inheritedLowerBound =
                    results[index].certifiedLowerBound;
                children.push_back(std::move(child));
            }
            ++lastStats.nodesBranched;
            lastStats.nodesCreated += 2;
        }

        ++lastStats.levelsCompleted;
        if (!unresolvedLeaves.empty()) {
            // Keep all unresolved regions represented in the final certificate.
            frontier = std::move(unresolvedLeaves);
            frontier.insert(
                frontier.end(),
                std::make_move_iterator(children.begin()),
                std::make_move_iterator(children.end()));
            recordTrace(depth);
            stop = true;
            break;
        }

        frontier = std::move(children);
        recordTrace(depth);
    }

    refreshBounds();
    if (!proofReached
        && gapClosed(options, lastStats.lowerBound, lastStats.upperBound)) {
        proofReached = bestFound;
    }

    PCTSPResult result{};
    result.objValue = bestUpperBound;
    result.totalPrize = 0.0;
    result.totalDistance = 0.0;
    result.solveTime = secondsSince(start);
    result.optimal = proofReached;
    if (bestFound) {
        result.tour.push_back(0);
        result.tour.insert(result.tour.end(), bestTour.begin(), bestTour.end());
        result.tour.push_back(0);
        result.selectedNodes.push_back(0);
        for (int client : bestTour) {
            result.selectedNodes.push_back(client);
            result.totalPrize += params.cli[client].skipPenalty;
        }
        for (std::size_t position = 1; position < result.tour.size(); ++position) {
            result.totalDistance +=
                params.timeCost[result.tour[position - 1]][result.tour[position]];
        }
    }

    if (options.verbose) {
        std::cout << "\n===== level-synchronous SCIP B&B result =====\n"
                  << std::fixed << std::setprecision(6)
                  << "  UB (certified):      " << lastStats.upperBound << '\n'
                  << "  LB (open nodes):     " << lastStats.lowerBound << '\n'
                  << "  relative gap:        "
                  << 100.0 * lastStats.relativeGap << "%\n"
                  << "  levels completed:    " << lastStats.levelsCompleted << '\n'
                  << "  cut epochs:          " << lastStats.cutEpochs << '\n'
                  << "  node LP solves:      " << lastStats.nodeLPSolves << '\n'
                  << "  open nodes:          " << lastStats.openNodes << '\n'
                  << "  evaluated tours:     " << lastStats.evaluatedTours << '\n'
                  << "  global tour cuts:    " << lastStats.optimalityCuts << '\n'
                  << "  oracle slack (max):  "
                  << lastStats.maxAppliedOracleSlack << '\n'
                  << "  hit time limit:      "
                  << (lastStats.hitTimeLimit ? "yes" : "no") << '\n'
                  << "  tolerance optimal:   "
                  << (result.optimal ? "yes" : "no") << '\n';
    }
    return result;
}
