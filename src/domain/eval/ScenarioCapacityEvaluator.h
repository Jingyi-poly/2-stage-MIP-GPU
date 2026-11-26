#ifndef AMDAHL_SRC_DOMAIN_EVAL_SCENARIO_CAPACITY_EVALUATOR_H_
#define AMDAHL_SRC_DOMAIN_EVAL_SCENARIO_CAPACITY_EVALUATOR_H_

#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

// A CSR-style batch of routes to be evaluated together.
//
// `nodeIds` holds the concatenated node ids of every route (the depot is NOT
// included, exactly like VrpRoute's startNode..endNode chain). `routeOffsets`
// is a CSR offset array of size RouteNum()+1 into `nodeIds`, so route r spans
// [routeOffsets[r], routeOffsets[r + 1]). Each route's prefix load starts from
// an empty vehicle (load 0) leaving the depot.
struct RouteBatch {
    std::vector<int> nodeIds;
    std::vector<int> routeOffsets;

    int RouteNum() const { return static_cast<int>(routeOffsets.size()) - 1; }
    int TotalNodes() const { return static_cast<int>(nodeIds.size()); }
};

// A batch of independent insertion probes (one request per probe against one
// route), evaluated together in a single GPU launch so the kernel-launch latency
// is amortised across all probes - the realistic greedy-insert sweep shape.
struct InsertionProbeBatch {
    std::vector<int> nodeIds;       // concatenated route node ids of every probe
    std::vector<int> routeOffsets;  // CSR offsets into nodeIds, size ProbeNum()+1
    std::vector<int> pickupIds;     // size ProbeNum()
    std::vector<int> deliveryIds;   // size ProbeNum()

    int ProbeNum() const { return static_cast<int>(pickupIds.size()); }
    int RouteLen(int probe) const { return routeOffsets[probe + 1] - routeOffsets[probe]; }
};

// Computes, for every node of every route in a batch, the multi-scenario state
// the memetic solver relies on:
//   * outLoads        : node-major [TotalNodes * numScenarios], the per-scenario
//                       prefix load at each node (== RouteNode::loadStatus).
//   * outNodeAccViol  : [TotalNodes], the prefix-accumulated capacity violation
//                       summed over all scenarios at each node
//                       (== RouteNode::accumulatedCapacityViolation).
//
// This is the single multi-scenario primitive shared by updateRouteData /
// calInsertionData / removal / EAX; it is the part that scales with the number
// of scenarios and therefore the target for GPU offloading.
class ScenarioCapacityEvaluator {
   public:
    virtual ~ScenarioCapacityEvaluator() = default;

    virtual void Evaluate(const RouteBatch& batch, std::vector<double>& outLoads,
                          std::vector<double>& outNodeAccViol) = 0;

    // Lean path: returns only the total capacity violation of each route
    // (outRouteViol has size RouteNum()), keeping the large per-node/per-scenario
    // state on the device. This is what fitness evaluation / calTotalValue needs,
    // and it avoids transferring the O(nodes * scenarios) load array back to the
    // host - making it the high-throughput target for very large scenario counts.
    virtual void EvaluateRouteViolations(const RouteBatch& batch, std::vector<double>& outRouteViol) = 0;

    // Device-resident, transfer-light candidate evaluation. For a single route
    // (node ids r[0..L-1]) and one request (pickup/delivery node ids), returns the
    // capacity-violation DELTA of inserting that request at every position pair
    // (pickupGap a, deliveryGap b) with 0 <= a <= b <= L.
    //
    // outDeltas is row-major [(L+1) * (L+1)]; entry a*(L+1)+b holds the delta for
    // b >= a and +inf otherwise. Only the resident demand matrix is read and only
    // O(L^2) scalars are produced - nothing of size L * numScenarios is transferred,
    // which is what lets the GPU win once the scenario count is large.
    virtual void EvaluateInsertionDeltas(const std::vector<int>& routeNodeIds, int pickupNodeId, int deliveryNodeId,
                                         std::vector<double>& outDeltas) = 0;

    // Batched form of EvaluateInsertionDeltas: outDeltas[p] is probe p's
    // (RouteLen(p)+1)^2 row-major delta matrix. A single GPU launch covers the
    // whole batch, which is what makes the device path win in practice.
    virtual void EvaluateInsertionDeltasBatch(const InsertionProbeBatch& batch,
                                              std::vector<std::vector<double>>& outDeltas) = 0;

    // 2-opt segment-reversal capacity primitive (the heavy step inside local
    // search's get_reversed_cost). For every route r of length L in the batch and
    // every reversal segment [a, b] with 1 <= a <= b <= L-1, computes
    //   RV(a,b) = sum over the reversed visit order (b, b-1, ..., a), over
    //             scenarios, of max(0, prefixLoad[a-1] + running demand - capacity)
    // where prefixLoad[a-1] is the per-scenario load just before position a.
    // outViol[r] is row-major L*L; entry a*L+b holds RV(a,b) for b>=a>=1, else 0.
    // Only the resident demand matrix is read and only O(L^2) scalars per route are
    // produced, so the device path amortises across the whole 2-opt sweep.
    virtual void EvaluateReversalViolationsBatch(const RouteBatch& batch,
                                                 std::vector<std::vector<double>>& outViol) = 0;

    virtual int NumScenarios() const = 0;
    virtual const char* Backend() const = 0;
};

namespace scenario_eval_detail {

// Combines the per-prefix violation arrays (length L+1 each, indexed by the
// number of leading route nodes) into the full insertion-delta matrix. This is
// the cheap O(L^2) tail shared by both backends; the expensive O(L * scenarios)
// work that fills v0/vp/vpd is what each backend computes (on CPU or GPU).
//   v0[i]  = sum_s max(0, prefixLoad_i[s]                       - cap)
//   vp[i]  = sum_s max(0, prefixLoad_i[s] + pickupDemand[s]     - cap)
//   vpd[i] = sum_s max(0, prefixLoad_i[s] + pickupDemand[s] + deliveryDemand[s] - cap)
inline void CombineInsertionDeltas(const std::vector<double>& v0, const std::vector<double>& vp,
                                   const std::vector<double>& vpd, std::vector<double>& outDeltas) {
    const int m = static_cast<int>(v0.size());  // == L + 1
    const int routeLen = m - 1;

    std::vector<double> prefix0(m, 0.0), prefixP(m, 0.0), prefixPd(m, 0.0);
    for (int i = 1; i < m; ++i) {
        prefix0[i] = prefix0[i - 1] + v0[i];
        prefixP[i] = prefixP[i - 1] + vp[i];
        prefixPd[i] = prefixPd[i - 1] + vpd[i];
    }
    const double oldViolation = prefix0[routeLen];

    outDeltas.assign(static_cast<size_t>(m) * m, std::numeric_limits<double>::infinity());
    for (int a = 0; a <= routeLen; ++a) {
        for (int b = a; b <= routeLen; ++b) {
            const double newViolation = prefix0[a] + vp[a] + (prefixP[b] - prefixP[a]) + vpd[b] +
                                        (prefixPd[routeLen] - prefixPd[b]);
            outDeltas[static_cast<size_t>(a) * m + b] = newViolation - oldViolation;
        }
    }
}

}  // namespace scenario_eval_detail

enum class EvaluatorBackend : std::uint8_t { kAuto = 0, kCpu = 1, kCuda = 2 };

// Builds an evaluator bound to a node-major demand matrix
// (demandMatrix[node * numScenarios + scenario], length nodeNum * numScenarios).
// The matrix is copied/uploaded internally, so the caller may free it afterwards.
//
// kAuto picks CUDA when the binary was built with USE_CUDA and a device is
// present, otherwise CPU. kCuda falls back to CPU (with a warning) when CUDA is
// unavailable, so callers can always rely on a non-null result.
std::unique_ptr<ScenarioCapacityEvaluator> MakeScenarioCapacityEvaluator(
    const float* demandMatrix, int nodeNum, int numScenarios, double vehicleCapacity,
    EvaluatorBackend backend = EvaluatorBackend::kAuto);

#endif
