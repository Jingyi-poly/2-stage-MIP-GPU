#include "PCTSPExactSCIP.h"

#include <scip/cons_linear.h>
#include <scip/scip.h>
#include <scip/scipdefplugins.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

constexpr double kAbsGapTolerance = 1e-5;
constexpr double kRelGapTolerance = 1e-6;

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

    ~SCIPOwner()
    {
        if (value != nullptr)
            SCIPfree(&value);
    }
};

double relativeGap(double lowerBound, double upperBound)
{
    if (!std::isfinite(lowerBound) || !std::isfinite(upperBound))
        return std::numeric_limits<double>::infinity();
    return std::max(0.0, upperBound - lowerBound)
        / std::max(1.0, std::abs(upperBound));
}

bool gapClosed(double lowerBound, double upperBound)
{
    if (!std::isfinite(lowerBound) || !std::isfinite(upperBound))
        return false;
    const double tolerance = std::max(
        kAbsGapTolerance,
        kRelGapTolerance * std::max(1.0, std::abs(upperBound)));
    return upperBound - lowerBound <= tolerance;
}

double boundTolerance(double upperBound)
{
    return std::max(
        kAbsGapTolerance,
        kRelGapTolerance * std::max(1.0, std::abs(upperBound)));
}

const char* statusName(SCIP_STATUS status)
{
    switch (status) {
    case SCIP_STATUS_UNKNOWN: return "unknown";
    case SCIP_STATUS_USERINTERRUPT: return "user interrupt";
    case SCIP_STATUS_NODELIMIT: return "node limit";
    case SCIP_STATUS_TOTALNODELIMIT: return "total node limit";
    case SCIP_STATUS_STALLNODELIMIT: return "stall node limit";
    case SCIP_STATUS_TIMELIMIT: return "time limit";
    case SCIP_STATUS_MEMLIMIT: return "memory limit";
    case SCIP_STATUS_GAPLIMIT: return "gap limit";
    case SCIP_STATUS_SOLLIMIT: return "solution limit";
    case SCIP_STATUS_BESTSOLLIMIT: return "best solution limit";
    case SCIP_STATUS_RESTARTLIMIT: return "restart limit";
    case SCIP_STATUS_OPTIMAL: return "optimal";
    case SCIP_STATUS_INFEASIBLE: return "infeasible";
    case SCIP_STATUS_UNBOUNDED: return "unbounded";
    case SCIP_STATUS_INFORUNBD: return "infeasible or unbounded";
    case SCIP_STATUS_TERMINATE: return "terminate";
    default: return "other";
    }
}

} // namespace

PCTSPExactSCIP::PCTSPExactSCIP(
    const Params& params, double timeLimit, bool verbose)
    : params(params),
      timeLimit(timeLimit),
      verbose(verbose),
      n(params.nbClients + 1)
{
    // Correlated stress instances have a small set of repeated demand vectors.
    // Other samplers keep the OpenMP oracle over the original scenario rows.
    if (params.ap.demandDistribution != SCENARIO_DEMAND_CORRELATED_STRESS)
        return;
    std::map<std::vector<double>, std::size_t> typeIndex;
    for (int scenario = 0; scenario < params.n_scenarios; ++scenario) {
        std::vector<double> demand;
        demand.reserve(params.nbClients);
        for (int client = 1; client <= params.nbClients; ++client)
            demand.push_back(params.cli[client].demands_scenarios[scenario]);
        const auto [position, inserted] = typeIndex.emplace(demand, scenarioTypes.size());
        if (inserted)
            scenarioTypes.push_back({scenario, 1});
        else
            ++scenarioTypes[position->second].second;
        if (scenarioTypes.size() > 512) {
            scenarioTypes.clear();
            return;
        }
    }
    long long count = 0;
    for (const auto& row : scenarioTypes) count += row.second;
    if (count != params.n_scenarios)
        throw std::runtime_error("weighted-scenario multiplicities do not sum to N");
    std::cout << "SCENARIO_TYPE_COMPRESSION unique=" << scenarioTypes.size()
              << " original=" << params.n_scenarios << '\n';
}

double PCTSPExactSCIP::splitDP(
    const std::vector<int>& perm, int scenario) const
{
    const int size = static_cast<int>(perm.size());
    if (size == 0)
        return 0.0;

    const double infinity = 1e30;
    std::vector<double> cost(size + 1, infinity);
    cost[0] = 0.0;

    for (int end = 1; end <= size; ++end) {
        double load = 0.0;
        double routeSuffix = 0.0;
        for (int begin = end; begin >= 1; --begin) {
            const int client = perm[begin - 1];
            load += params.cli[client].demands_scenarios[scenario];
            if (begin == end)
                routeSuffix = params.timeCost[client][0];
            else
                routeSuffix += params.timeCost[client][perm[begin]];

            const double routeDistance =
                params.timeCost[0][client] + routeSuffix;
            const double excess =
                std::max(0.0, load - params.vehicleCapacity);
            const double segmentCost =
                routeDistance + params.penaltyCapacity * excess;
            cost[end] = std::min(cost[end], cost[begin - 1] + segmentCost);
        }
    }
    return cost[size];
}

double PCTSPExactSCIP::evaluateTourCPU(const std::vector<int>& perm) const
{
    if (params.n_scenarios <= 0)
        throw std::runtime_error("No stochastic demand scenarios were generated");
    if (scenarioTypes.empty())
        return evaluateTourCPUUncompressed(perm);

    long double total = 0.0;
    for (const auto& [scenario, multiplicity] : scenarioTypes)
        total += static_cast<long double>(multiplicity) * splitDP(perm, scenario);
    return static_cast<double>(total / params.n_scenarios);
}

double PCTSPExactSCIP::evaluateTourCPUUncompressed(const std::vector<int>& perm) const
{
    double total = 0.0;
#pragma omp parallel for reduction(+ : total) schedule(static)
    for (int scenario = 0; scenario < params.n_scenarios; ++scenario)
        total += splitDP(perm, scenario);
    return total / params.n_scenarios;
}

void PCTSPExactSCIP::validateWeightedScenarioOracle() const
{
    if (scenarioTypes.empty())
        return;
    std::vector<int> fullTour;
    for (int client = 1; client <= params.nbClients; ++client)
        fullTour.push_back(client);
    const double weighted = evaluateTourCPU(fullTour);
    const double raw = evaluateTourCPUUncompressed(fullTour);
    const double diff = std::abs(weighted - raw);
    const double tolerance = 1e-7 * std::max(1.0, std::abs(raw));
    if (diff > tolerance)
        throw std::runtime_error("weighted/uncompressed million-row oracle mismatch");
    std::cout << std::setprecision(17)
              << "SCENARIO_TYPE_PARITY weighted=" << weighted
              << " raw=" << raw << " absolute_difference=" << diff
              << " tolerance=" << tolerance << '\n';
}

PCTSPResult PCTSPExactSCIP::solveFullStochasticMILP(
    TourEvalFunc tourEval, int batchSize)
{
    TourBatchEvalFunc batchEval;
    if (tourEval) {
        batchEval = [tourEval = std::move(tourEval)](
                        const std::vector<std::vector<int>>& tours) {
            std::vector<double> costs;
            costs.reserve(tours.size());
            for (const auto& tour : tours)
                costs.push_back(tourEval(tour));
            return costs;
        };
    } else {
        batchEval = [this](const std::vector<std::vector<int>>& tours) {
            std::vector<double> costs;
            costs.reserve(tours.size());
            for (const auto& tour : tours)
                costs.push_back(evaluateTourCPU(tour));
            return costs;
        };
    }
    return solveFullStochasticMILPBatch(std::move(batchEval), batchSize);
}

PCTSPResult PCTSPExactSCIP::solveFullStochasticMILPBatch(
    TourBatchEvalFunc batchEval, int batchSize)
{
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();

    PCTSPResult result{};
    result.objValue = std::numeric_limits<double>::infinity();
    result.totalPrize = 0.0;
    result.totalDistance = 0.0;
    result.solveTime = 0.0;
    result.optimal = false;
    lastStats = SCIPBendersStats{};
    lastTrace.clear();

    if (!batchEval)
        throw std::invalid_argument("SCIP Benders requires a tour evaluator");
    if (batchSize < 1)
        throw std::invalid_argument("SCIP Benders batch size must be positive");
    if (n < 2)
        throw std::invalid_argument("SCIP Benders requires at least one client");

    lastStats.requestedBatchSize = batchSize;

    const int nodeCount = n;
    const int clientCount = n - 1;
    SCIPOwner owner;
    scipCheck(SCIPcreate(&owner.value), "SCIPcreate");
    SCIP* scip = owner.value;
    scipCheck(SCIPincludeDefaultPlugins(scip), "SCIPincludeDefaultPlugins");
    scipCheck(SCIPcreateProbBasic(
                  scip, params.ap.optionalVisit
                      ? "stochastic_optional_cvrp_benders"
                      : "stochastic_cvrp_benders"),
              "SCIPcreateProbBasic");
    scipCheck(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE),
              "SCIPsetObjsense");
    scipCheck(SCIPsetIntParam(
                  scip, "display/verblevel", verbose ? 4 : 0),
              "SCIPsetIntParam(display/verblevel)");

    auto index = [nodeCount](int i, int j) { return i * nodeCount + j; };
    std::vector<SCIP_VAR*> arc(nodeCount * nodeCount, nullptr);
    std::vector<SCIP_VAR*> flow(nodeCount * nodeCount, nullptr);
    std::vector<SCIP_VAR*> selected(nodeCount, nullptr);

    for (int i = 0; i < nodeCount; ++i) {
        for (int j = 0; j < nodeCount; ++j) {
            if (i == j)
                continue;

            std::ostringstream arcName;
            arcName << "x_" << i << '_' << j;
            scipCheck(SCIPcreateVarBasic(
                          scip, &arc[index(i, j)], arcName.str().c_str(),
                          0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY),
                      "SCIPcreateVarBasic(arc)");
            scipCheck(SCIPaddVar(scip, arc[index(i, j)]), "SCIPaddVar(arc)");

            std::ostringstream flowName;
            flowName << "f_" << i << '_' << j;
            scipCheck(SCIPcreateVarBasic(
                          scip, &flow[index(i, j)], flowName.str().c_str(),
                          0.0, static_cast<double>(clientCount), 0.0,
                          SCIP_VARTYPE_CONTINUOUS),
                      "SCIPcreateVarBasic(flow)");
            scipCheck(SCIPaddVar(scip, flow[index(i, j)]),
                      "SCIPaddVar(flow)");
        }
    }

    for (int i = 1; i < nodeCount; ++i) {
        std::ostringstream name;
        name << "y_" << i;
        const double lowerBound = params.ap.optionalVisit ? 0.0 : 1.0;
        scipCheck(SCIPcreateVarBasic(
                      scip, &selected[i], name.str().c_str(), lowerBound, 1.0,
                      0.0, SCIP_VARTYPE_BINARY),
                  "SCIPcreateVarBasic(selected)");
        scipCheck(SCIPaddVar(scip, selected[i]), "SCIPaddVar(selected)");
    }

    SCIP_VAR* active = nullptr;
    scipCheck(SCIPcreateVarBasic(
                  scip, &active, "active_tour",
                  params.ap.optionalVisit ? 0.0 : 1.0, 1.0, 0.0,
                  SCIP_VARTYPE_BINARY),
              "SCIPcreateVarBasic(active_tour)");
    scipCheck(SCIPaddVar(scip, active), "SCIPaddVar(active_tour)");

    SCIP_VAR* theta = nullptr;
    SCIP_VAR* residual = nullptr;
    SCIP_VAR* skipCostVariable = nullptr;
    scipCheck(SCIPcreateVarBasic(
                  scip, &theta, "theta", 0.0, SCIPinfinity(scip), 1.0,
                  SCIP_VARTYPE_CONTINUOUS),
              "SCIPcreateVarBasic(theta)");
    scipCheck(SCIPaddVar(scip, theta), "SCIPaddVar(theta)");
    scipCheck(SCIPcreateVarBasic(
                  scip, &residual, "split_residual", 0.0,
                  SCIPinfinity(scip), 0.0, SCIP_VARTYPE_CONTINUOUS),
              "SCIPcreateVarBasic(split_residual)");
    scipCheck(SCIPaddVar(scip, residual), "SCIPaddVar(split_residual)");

    double totalSkipPenalty = 0.0;
    for (int i = 1; i < nodeCount; ++i) {
        const double penalty = params.cli[i].skipPenalty;
        if (!std::isfinite(penalty) || penalty < 0.0)
            throw std::runtime_error(
                "Optional-visit master requires finite nonnegative skip penalties");
        totalSkipPenalty += penalty;
    }
    scipCheck(SCIPcreateVarBasic(
                  scip, &skipCostVariable, "skip_cost", 0.0,
                  totalSkipPenalty, 1.0, SCIP_VARTYPE_CONTINUOUS),
              "SCIPcreateVarBasic(skip_cost)");
    scipCheck(SCIPaddVar(scip, skipCostVariable), "SCIPaddVar(skip_cost)");

    auto addLinear = [&](const std::string& name,
                         std::vector<SCIP_VAR*>& vars,
                         std::vector<SCIP_Real>& values,
                         SCIP_Real lhs,
                         SCIP_Real rhs) {
        SCIP_CONS* constraint = nullptr;
        scipCheck(SCIPcreateConsBasicLinear(
                      scip, &constraint, name.c_str(),
                      static_cast<int>(vars.size()), vars.data(), values.data(),
                      lhs, rhs),
                  "SCIPcreateConsBasicLinear");
        scipCheck(SCIPaddCons(scip, constraint), "SCIPaddCons");
        scipCheck(SCIPreleaseCons(scip, &constraint), "SCIPreleaseCons");
    };

    // A selected client has exactly one incoming and one outgoing giant-tour
    // arc.  The depot has one of each iff the selected set is non-empty.
    for (int i = 1; i < nodeCount; ++i) {
        std::vector<SCIP_VAR*> outgoing;
        std::vector<SCIP_VAR*> incoming;
        std::vector<SCIP_Real> outgoingValues;
        std::vector<SCIP_Real> incomingValues;
        outgoing.reserve(nodeCount);
        incoming.reserve(nodeCount);
        outgoingValues.reserve(nodeCount);
        incomingValues.reserve(nodeCount);
        for (int j = 0; j < nodeCount; ++j) {
            if (i == j)
                continue;
            outgoing.push_back(arc[index(i, j)]);
            outgoingValues.push_back(1.0);
            incoming.push_back(arc[index(j, i)]);
            incomingValues.push_back(1.0);
        }
        outgoing.push_back(selected[i]);
        outgoingValues.push_back(-1.0);
        incoming.push_back(selected[i]);
        incomingValues.push_back(-1.0);
        addLinear("out_degree_" + std::to_string(i), outgoing,
                  outgoingValues, 0.0, 0.0);
        addLinear("in_degree_" + std::to_string(i), incoming,
                  incomingValues, 0.0, 0.0);

        std::vector<SCIP_VAR*> activationVars{selected[i], active};
        std::vector<SCIP_Real> activationValues{1.0, -1.0};
        addLinear("selection_implies_active_" + std::to_string(i),
                  activationVars, activationValues,
                  -SCIPinfinity(scip), 0.0);
    }

    std::vector<SCIP_VAR*> depotOutgoing;
    std::vector<SCIP_VAR*> depotIncoming;
    std::vector<SCIP_Real> depotOutgoingValues;
    std::vector<SCIP_Real> depotIncomingValues;
    depotOutgoing.reserve(nodeCount);
    depotIncoming.reserve(nodeCount);
    depotOutgoingValues.reserve(nodeCount);
    depotIncomingValues.reserve(nodeCount);
    for (int j = 1; j < nodeCount; ++j) {
        depotOutgoing.push_back(arc[index(0, j)]);
        depotOutgoingValues.push_back(1.0);
        depotIncoming.push_back(arc[index(j, 0)]);
        depotIncomingValues.push_back(1.0);
    }
    depotOutgoing.push_back(active);
    depotOutgoingValues.push_back(-1.0);
    depotIncoming.push_back(active);
    depotIncomingValues.push_back(-1.0);
    addLinear("depot_out_degree", depotOutgoing, depotOutgoingValues, 0.0, 0.0);
    addLinear("depot_in_degree", depotIncoming, depotIncomingValues, 0.0, 0.0);

    // Static single-commodity flow formulation.  The depot sends one unit for
    // every selected client and each selected client consumes one unit.  This
    // eliminates disconnected cycles while allowing the all-zero empty tour.
    for (int i = 0; i < nodeCount; ++i) {
        std::vector<SCIP_VAR*> vars;
        std::vector<SCIP_Real> values;
        vars.reserve(2 * (nodeCount - 1) + clientCount);
        values.reserve(2 * (nodeCount - 1) + clientCount);
        for (int j = 0; j < nodeCount; ++j) {
            if (i == j)
                continue;
            vars.push_back(flow[index(i, j)]);
            values.push_back(i == 0 ? 1.0 : -1.0);
            vars.push_back(flow[index(j, i)]);
            values.push_back(i == 0 ? -1.0 : 1.0);
        }
        if (i == 0) {
            for (int client = 1; client < nodeCount; ++client) {
                vars.push_back(selected[client]);
                values.push_back(-1.0);
            }
        } else {
            vars.push_back(selected[i]);
            values.push_back(-1.0);
        }
        addLinear("flow_balance_" + std::to_string(i),
                  vars, values, 0.0, 0.0);
    }

    // skip_cost = sum_i p_i (1-y_i).  Modeling the constant through an
    // explicit variable keeps SCIP's dual bound in the full objective scale.
    std::vector<SCIP_VAR*> skipVars{skipCostVariable};
    std::vector<SCIP_Real> skipValues{1.0};
    skipVars.reserve(nodeCount);
    skipValues.reserve(nodeCount);
    for (int i = 1; i < nodeCount; ++i) {
        skipVars.push_back(selected[i]);
        skipValues.push_back(params.cli[i].skipPenalty);
    }
    addLinear("skip_cost_definition", skipVars, skipValues,
              totalSkipPenalty, totalSkipPenalty);
    for (int i = 0; i < nodeCount; ++i) {
        for (int j = 0; j < nodeCount; ++j) {
            if (i == j)
                continue;
            std::vector<SCIP_VAR*> vars{
                flow[index(i, j)], arc[index(i, j)]};
            std::vector<SCIP_Real> values{
                1.0, -static_cast<double>(clientCount)};
            addLinear("flow_link_" + std::to_string(i) + '_' +
                          std::to_string(j),
                      vars, values, -SCIPinfinity(scip), 0.0);
        }
    }

    // A universally valid distance anchor.  For a client-client giant-tour
    // arc (i,j), Split either keeps d(i,j), or cuts there and pays
    // d(i,0)+d(0,j).  Its contribution is therefore at least their minimum.
    // Depot arcs are always paid.  On metric instances this is exactly the
    // ordinary giant-tour distance D(x).
    std::vector<double> anchorCost(nodeCount * nodeCount, 0.0);
    std::vector<SCIP_VAR*> anchorVars{theta, residual};
    std::vector<SCIP_Real> anchorValues{1.0, -1.0};
    anchorVars.reserve(nodeCount * (nodeCount - 1) + 2);
    anchorValues.reserve(nodeCount * (nodeCount - 1) + 2);
    for (int i = 0; i < nodeCount; ++i) {
        for (int j = 0; j < nodeCount; ++j) {
            if (i == j)
                continue;
            double value = params.timeCost[i][j];
            if (i != 0 && j != 0)
                value = std::min(value,
                                 params.timeCost[i][0] + params.timeCost[0][j]);
            if (!std::isfinite(value) || value < 0.0)
                throw std::runtime_error(
                    "Distance anchor requires finite nonnegative travel costs");
            anchorCost[index(i, j)] = value;
            anchorVars.push_back(arc[index(i, j)]);
            anchorValues.push_back(-value);
        }
    }
    // theta = B(x) + residual, residual >= 0.  Keeping residual explicit makes
    // every exact-tour cut sparse while remaining algebraically identical to
    // theta >= B(x) + (Q(T)-B(T))*(sum_{e in T} x_e-(|T|-1)).
    addLinear("distance_anchor", anchorVars, anchorValues, 0.0, 0.0);

    if (verbose) {
        std::cout << "\n===== SCIP outer-loop Benders =====\n"
                  << "  clients: " << clientCount << '\n'
                  << "  scenarios: " << params.n_scenarios << '\n'
                  << "  optional visits: "
                  << (params.ap.optionalVisit ? "yes (empty set allowed)" : "no")
                  << '\n'
                  << "  recourse: unlimited-route-count, capacity-penalized Split\n"
                  << "  objective: skip penalties + stochastic recourse\n"
                  << "  connectivity: optional static single-commodity flow\n"
                  << "  evaluator batch capacity: " << batchSize << '\n'
                  << "  cut: distance-anchored exact-tour cut\n"
                  << "===================================\n";
    }

    double bestUpperBound = std::numeric_limits<double>::infinity();
    double certifiedLowerBound = 0.0;
    std::vector<int> bestPermutation;
    bool bestFound = false;
    std::set<std::vector<int>> evaluatedTours;
    int iteration = 0;
    auto recordTrace = [&]() {
        if (!std::isfinite(bestUpperBound) ||
            !std::isfinite(certifiedLowerBound))
            return;
        lastTrace.push_back(SCIPBendersTracePoint{
            std::chrono::duration<double>(Clock::now() - start).count(),
            iteration,
            lastStats.evaluatedTours,
            bestUpperBound,
            certifiedLowerBound,
            relativeGap(certifiedLowerBound, bestUpperBound),
        });
    };

    while (true) {
        const double elapsed =
            std::chrono::duration<double>(Clock::now() - start).count();
        const double remaining = timeLimit > 0.0
            ? timeLimit - elapsed
            : std::numeric_limits<double>::infinity();
        if (remaining <= 1e-3) {
            lastStats.hitTimeLimit = true;
            break;
        }
        if (std::isfinite(remaining))
            scipCheck(SCIPsetRealParam(scip, "limits/time", remaining),
                      "SCIPsetRealParam(limits/time)");

        const auto masterStart = Clock::now();
        scipCheck(SCIPsolve(scip), "SCIPsolve");
        lastStats.masterSolveTime += std::chrono::duration<double>(
            Clock::now() - masterStart).count();
        ++iteration;

        const SCIP_STATUS status = SCIPgetStatus(scip);
        const double iterationLowerBound = SCIPgetDualbound(scip);
        if (std::isfinite(iterationLowerBound))
            certifiedLowerBound = std::max(certifiedLowerBound,
                                           iterationLowerBound);

        SCIP_SOL* solution = SCIPgetBestSol(scip);
        if (solution == nullptr) {
            if (verbose) {
                std::cout << "  [SCIP] iteration " << iteration
                          << ": no master incumbent, status="
                          << statusName(status) << '\n';
            }
            if (status == SCIP_STATUS_TIMELIMIT)
                lastStats.hitTimeLimit = true;
            break;
        }

        std::vector<bool> selectedMask(nodeCount, false);
        int selectedCount = 0;
        double candidateSkipCost = totalSkipPenalty;
        for (int i = 1; i < nodeCount; ++i) {
            if (SCIPgetSolVal(scip, solution, selected[i]) > 0.5) {
                selectedMask[i] = true;
                ++selectedCount;
                candidateSkipCost -= params.cli[i].skipPenalty;
            }
        }
        // Avoid a tiny negative value from floating-point cancellation.
        candidateSkipCost = std::max(0.0, candidateSkipCost);
        const double masterSkipCost =
            SCIPgetSolVal(scip, solution, skipCostVariable);
        if (std::abs(masterSkipCost - candidateSkipCost) >
            1e-7 * std::max(1.0, totalSkipPenalty))
            throw std::runtime_error(
                "SCIP incumbent has an inconsistent skip-cost value");

        const bool masterActive =
            SCIPgetSolVal(scip, solution, active) > 0.5;
        if (masterActive != (selectedCount > 0))
            throw std::runtime_error(
                "SCIP optional master has inconsistent tour activation");

        std::vector<int> successor(nodeCount, -1);
        for (int i = 0; i < nodeCount; ++i) {
            for (int j = 0; j < nodeCount; ++j) {
                if (i != j &&
                    SCIPgetSolVal(scip, solution, arc[index(i, j)]) > 0.5) {
                    if (successor[i] != -1)
                        throw std::runtime_error(
                            "SCIP incumbent has multiple outgoing tour arcs");
                    successor[i] = j;
                }
            }
        }

        std::vector<int> permutation;
        std::vector<bool> seen(nodeCount, false);
        permutation.reserve(selectedCount);
        seen[0] = true;
        int current = 0;
        double tourAnchor = 0.0;
        if (selectedCount == 0) {
            for (SCIP_VAR* arcVariable : arc) {
                if (arcVariable != nullptr &&
                    SCIPgetSolVal(scip, solution, arcVariable) > 0.5)
                    throw std::runtime_error(
                        "SCIP empty-set incumbent unexpectedly uses a tour arc");
            }
        } else {
            for (int step = 0; step <= selectedCount; ++step) {
                const int next = successor[current];
                if (next < 0 || next >= nodeCount)
                    throw std::runtime_error(
                        "SCIP incumbent is missing a selected-tour arc");
                tourAnchor += anchorCost[index(current, next)];
                if (step < selectedCount) {
                    if (next == 0 || !selectedMask[next] || seen[next])
                        throw std::runtime_error(
                            "SCIP flow master returned a disconnected selected tour");
                    seen[next] = true;
                    permutation.push_back(next);
                } else if (next != 0) {
                    throw std::runtime_error(
                        "SCIP selected tour does not return to the depot");
                }
                current = next;
            }
        }
        if (static_cast<int>(permutation.size()) != selectedCount)
            throw std::runtime_error(
                "SCIP incumbent omits one or more selected clients");

        // The certified master incumbent is always the first candidate.  Fill
        // the remainder of the GPU launch deterministically with unseen 2-opt
        // and swap neighbors.  These extra tours only strengthen the master;
        // the proof never relies on them being a systematic K-best pool.
        if (evaluatedTours.find(permutation) != evaluatedTours.end()) {
            // The re-solve may have tightened the certified dual bound all the
            // way to the incumbent.  Preserve that final proof event even
            // though no additional tour evaluation is needed.
            recordTrace();
            if (gapClosed(certifiedLowerBound, bestUpperBound))
                break;
            if (verbose) {
                std::cerr
                    << "  [SCIP] stopping with a partial bound because a "
                    << "previously evaluated master tour reappeared before "
                    << "the numerical gap closed\n";
            }
            break;
        }

        std::vector<std::vector<int>> evaluationBatch;
        evaluationBatch.reserve(batchSize);
        std::set<std::vector<int>> scheduledTours = evaluatedTours;
        auto scheduleTour = [&](std::vector<int> candidate) {
            if (static_cast<int>(evaluationBatch.size()) >= batchSize)
                return;
            if (scheduledTours.insert(candidate).second)
                evaluationBatch.push_back(std::move(candidate));
        };
        scheduleTour(permutation);

        for (int left = 0;
             left + 1 < selectedCount &&
             static_cast<int>(evaluationBatch.size()) < batchSize;
             ++left) {
            for (int right = left + 1;
                 right < selectedCount &&
                 static_cast<int>(evaluationBatch.size()) < batchSize;
                 ++right) {
                std::vector<int> neighbor = permutation;
                std::reverse(neighbor.begin() + left,
                             neighbor.begin() + right + 1);
                scheduleTour(std::move(neighbor));
            }
        }
        for (int first = 0;
             first + 1 < selectedCount &&
             static_cast<int>(evaluationBatch.size()) < batchSize;
             ++first) {
            for (int second = first + 1;
                 second < selectedCount &&
                 static_cast<int>(evaluationBatch.size()) < batchSize;
                 ++second) {
                std::vector<int> neighbor = permutation;
                std::swap(neighbor[first], neighbor[second]);
                scheduleTour(std::move(neighbor));
            }
        }

        const auto evaluationStart = Clock::now();
        const std::vector<double> evaluatedCosts = batchEval(evaluationBatch);
        lastStats.evaluationTime += std::chrono::duration<double>(
            Clock::now() - evaluationStart).count();
        if (evaluatedCosts.size() != evaluationBatch.size())
            throw std::runtime_error(
                "Tour batch evaluator returned the wrong number of costs");
        ++lastStats.evaluationBatches;
        lastStats.evaluatedTours += static_cast<int>(evaluationBatch.size());
        lastStats.maxActualBatchSize = std::max(
            lastStats.maxActualBatchSize,
            static_cast<int>(evaluationBatch.size()));

        std::vector<double> tourAnchors(evaluationBatch.size(), 0.0);
        std::vector<double> candidateSkipCosts(evaluationBatch.size(), 0.0);
        std::vector<double> candidateTotalCosts(evaluationBatch.size(), 0.0);
        for (std::size_t candidateIndex = 0;
             candidateIndex < evaluationBatch.size(); ++candidateIndex) {
            const std::vector<int>& candidate = evaluationBatch[candidateIndex];
            const double recourseCost = evaluatedCosts[candidateIndex];
            if (!std::isfinite(recourseCost) || recourseCost < 0.0)
                throw std::runtime_error(
                    "Tour evaluator returned a non-finite or negative cost");

            double candidateAnchor = 0.0;
            double skipCost = totalSkipPenalty;
            std::vector<bool> candidateSelected(nodeCount, false);
            int from = 0;
            for (int to : candidate) {
                if (to <= 0 || to >= nodeCount || candidateSelected[to])
                    throw std::runtime_error(
                        "Tour evaluator batch contains an invalid selected tour");
                candidateSelected[to] = true;
                skipCost -= params.cli[to].skipPenalty;
                candidateAnchor += anchorCost[index(from, to)];
                from = to;
            }
            candidateAnchor += anchorCost[index(from, 0)];
            tourAnchors[candidateIndex] = candidateAnchor;
            candidateSkipCosts[candidateIndex] = std::max(0.0, skipCost);
            candidateTotalCosts[candidateIndex] =
                recourseCost + candidateSkipCosts[candidateIndex];

            const double anchorTolerance =
                1e-7 * std::max(1.0, std::abs(candidateAnchor));
            if (candidate.empty() && recourseCost > kAbsGapTolerance)
                throw std::runtime_error(
                    "Tour evaluator must return zero recourse for the empty selected set");
            if (recourseCost + anchorTolerance < candidateAnchor) {
                std::ostringstream message;
                message << "Tour evaluator recourse cost " << recourseCost
                        << " violates the rigorous distance anchor "
                        << candidateAnchor;
                throw std::runtime_error(message.str());
            }
            if (candidateTotalCosts[candidateIndex] < bestUpperBound) {
                bestUpperBound = candidateTotalCosts[candidateIndex];
                bestPermutation = candidate;
                bestFound = true;
            }
            evaluatedTours.insert(candidate);
        }

        if (certifiedLowerBound >
            bestUpperBound + boundTolerance(bestUpperBound)) {
            std::ostringstream message;
            message << "Bound inconsistency: SCIP lower bound "
                    << certifiedLowerBound << " exceeds evaluated upper bound "
                    << bestUpperBound
                    << "; check the tour oracle and cut coefficients";
            throw std::runtime_error(message.str());
        }

        if (timeLimit > 0.0 &&
            std::chrono::duration<double>(Clock::now() - start).count() >=
                timeLimit) {
            lastStats.hitTimeLimit = true;
        }

        if (verbose) {
            std::cout << std::fixed << std::setprecision(6)
                      << "  [SCIP] iter=" << iteration
                      << " status=" << statusName(status)
                      << " LB=" << certifiedLowerBound
                      << " UB=" << bestUpperBound
                      << " gap="
                      << 100.0 * relativeGap(certifiedLowerBound,
                                             bestUpperBound)
                      << "% total(master)=" << candidateTotalCosts.front()
                      << " [skip=" << candidateSkipCosts.front()
                      << ", Q=" << evaluatedCosts.front() << ']'
                      << " batch=" << evaluationBatch.size() << '\n';
        }

        recordTrace();

        if (gapClosed(certifiedLowerBound, bestUpperBound))
            break;
        // Only an optimal master solve may seed another exact outer round.
        // For time/node/memory/user/gap/solution limits, keep the incumbent and
        // rigorous dual bound just obtained and return a partial certificate.
        if (status != SCIP_STATUS_OPTIMAL) {
            if (status == SCIP_STATUS_TIMELIMIT)
                lastStats.hitTimeLimit = true;
            break;
        }

        // Constraints can be added to the original problem after freeing the
        // transformed solve.  This deliberately simple outer loop preserves a
        // rigorous dual bound and avoids callbacks invoking CUDA from SCIP.
        scipCheck(SCIPfreeTransform(scip), "SCIPfreeTransform");

        for (std::size_t candidateIndex = 0;
             candidateIndex < evaluationBatch.size(); ++candidateIndex) {
            const double residualCost = std::max(
                0.0, evaluatedCosts[candidateIndex] - tourAnchors[candidateIndex]);
            // A zero-residual tour is already represented exactly by the
            // distance anchor; adding its algebraic no-op cut is unnecessary.
            if (residualCost <= kAbsGapTolerance)
                continue;

            std::vector<SCIP_VAR*> cutVars;
            std::vector<SCIP_Real> cutValues;
            cutVars.reserve(2 * nodeCount + 1);
            cutValues.reserve(2 * nodeCount + 1);
            cutVars.push_back(residual);
            cutValues.push_back(1.0);
            std::vector<bool> candidateSelected(nodeCount, false);
            int from = 0;
            for (int to : evaluationBatch[candidateIndex]) {
                candidateSelected[to] = true;
                cutVars.push_back(arc[index(from, to)]);
                cutValues.push_back(-residualCost);
                from = to;
            }
            cutVars.push_back(arc[index(from, 0)]);
            cutValues.push_back(-residualCost);

            // Condition the cut on the exact selected subset as well as its
            // exact cycle.  If any absent client is selected, the +r*y_i term
            // relaxes the RHS by one residual unit.  If any tour arc is absent,
            // its -r*x_e term does the same.  Consequently the RHS is positive
            // only for this exact subset/order and is nonpositive everywhere
            // else, where residual >= 0 makes the cut globally valid.
            for (int client = 1; client < nodeCount; ++client) {
                if (!candidateSelected[client]) {
                    cutVars.push_back(selected[client]);
                    cutValues.push_back(residualCost);
                }
            }

            const int tourArcCount =
                static_cast<int>(evaluationBatch[candidateIndex].size()) + 1;
            const double lhs = residualCost * (1 - tourArcCount);
            addLinear("tour_cut_" + std::to_string(iteration) + '_' +
                          std::to_string(candidateIndex),
                      cutVars, cutValues, lhs, SCIPinfinity(scip));
            ++lastStats.optimalityCuts;
        }
    }

    lastStats.iterations = iteration;
    lastStats.lowerBound = certifiedLowerBound;
    lastStats.upperBound = bestUpperBound;
    lastStats.relativeGap = relativeGap(certifiedLowerBound, bestUpperBound);

    result.solveTime =
        std::chrono::duration<double>(Clock::now() - start).count();
    if (bestFound) {
        result.objValue = bestUpperBound;
        result.optimal = gapClosed(certifiedLowerBound, bestUpperBound);
        result.tour.push_back(0);
        result.tour.insert(result.tour.end(),
                           bestPermutation.begin(), bestPermutation.end());
        result.tour.push_back(0);
        result.selectedNodes.push_back(0);
        for (int node : bestPermutation) {
            result.selectedNodes.push_back(node);
            result.totalPrize += params.cli[node].skipPenalty;
        }
        for (std::size_t position = 1; position < result.tour.size(); ++position) {
            result.totalDistance += params.timeCost[result.tour[position - 1]]
                                                   [result.tour[position]];
        }
    }

    if (verbose) {
        std::cout << "\n===== SCIP Benders result =====\n"
                  << std::fixed << std::setprecision(6)
                  << "  UB (skip + recourse): " << lastStats.upperBound << '\n'
                  << "  LB (SCIP dual):      " << lastStats.lowerBound << '\n'
                  << "  relative gap:        "
                  << 100.0 * lastStats.relativeGap << "%\n"
                  << "  evaluated tours:     " << lastStats.evaluatedTours << '\n'
                  << "  evaluation batches:  " << lastStats.evaluationBatches << '\n'
                  << "  max actual batch:    " << lastStats.maxActualBatchSize
                  << '/' << lastStats.requestedBatchSize << '\n'
                  << "  exact-tour cuts:     " << lastStats.optimalityCuts << '\n'
                  << "  master solve time:   " << lastStats.masterSolveTime
                  << " s\n"
                  << "  evaluator time:      " << lastStats.evaluationTime
                  << " s\n"
                  << "  solve time:          " << result.solveTime << " s\n"
                  << "  proven optimal:      "
                  << (result.optimal ? "yes" : "no") << '\n';
    }

    // Release the captures owned by this builder.  The original problem keeps
    // its own captures until SCIPfree() in SCIPOwner's destructor.
    if (SCIPgetStage(scip) >= SCIP_STAGE_TRANSFORMED)
        scipCheck(SCIPfreeTransform(scip), "SCIPfreeTransform(final)");
    for (SCIP_VAR*& variable : arc) {
        if (variable != nullptr)
            scipCheck(SCIPreleaseVar(scip, &variable), "SCIPreleaseVar(arc)");
    }
    for (SCIP_VAR*& variable : flow) {
        if (variable != nullptr)
            scipCheck(SCIPreleaseVar(scip, &variable), "SCIPreleaseVar(flow)");
    }
    for (SCIP_VAR*& variable : selected) {
        if (variable != nullptr)
            scipCheck(SCIPreleaseVar(scip, &variable),
                      "SCIPreleaseVar(selected)");
    }
    scipCheck(SCIPreleaseVar(scip, &active),
              "SCIPreleaseVar(active_tour)");
    scipCheck(SCIPreleaseVar(scip, &theta), "SCIPreleaseVar(theta)");
    scipCheck(SCIPreleaseVar(scip, &residual),
              "SCIPreleaseVar(split_residual)");
    scipCheck(SCIPreleaseVar(scip, &skipCostVariable),
              "SCIPreleaseVar(skip_cost)");

    return result;
}
