#ifndef AMDAHL_SRC_DOMAIN_EVAL_MATRIX_MODEL_EVALUATOR_H_
#define AMDAHL_SRC_DOMAIN_EVAL_MATRIX_MODEL_EVALUATOR_H_

#include <vector>

class VrpSolution;
class ScenarioCapacityEvaluator;

// Aggregated result of a whole-solution "matrix model" cost evaluation.
struct MatrixModelResult {
    double distance{0.0};
    double capacityViolation{0.0};
    // Time-window penalty reproduced with the solver's capped-at-twr semantics
    // (matches VrpSolution::getTotalTwViolation()).
    double twViolationCapped{0.0};
    // Time-window penalty from the *pure* slide-8 cumsum/cummax/relu pipeline
    // (no upper cap: lateness propagates downstream). Equals twViolationCapped
    // only when no route ever arrives late; otherwise it is an upper bound and
    // measures how far the slide formula diverges from this solver's model.
    double twViolationSlides{0.0};
    int routeCount{0};
    int maxRouteLen{0};
};

// Whole-solution cost evaluation via the "NPU matrix model" formulation
// (see "NPU model for PDPTW"): represent the solution as a padded M x N matrix
// (rows = routes, columns = node positions, virtual nodes padded with -1) and
// compute every objective with dense tensor primitives -- prefix-sum (cumsum),
// running-max (cummax), relu = max(.,0) and reduce -- instead of the solver's
// incremental forward/backward slack scan.
//
//   distance  = reduce-sum(DISM) + return arcs                          (slide 6)
//   capacity  = sum_s reduce( relu( cumsum_row(DM_s) - capacity ) )     (slide 7, per scenario)
//   tw(slide) = reduce( relu( (A=cumsum_row(TM)) + cummax_row(TWL - A) - TWR ) )  (slide 8)
//   tw(capped)= row scan reproducing the solver's clamp-at-twr penalty
//
// Padding is handled implicitly via ragged rows, which is mathematically
// identical to the slides' fixed M x N matrix with sentinel virtual nodes
// (DMD(-1)=0, TWR(-1)=+inf, Capa(-1)=+inf) -- those sentinels are defined
// precisely so padded cells contribute nothing to any reduction.
//
// The capacity step is exactly the multi-scenario primitive already offloaded
// to the GPU (ScenarioCapacityEvaluator::EvaluateRouteViolations); this class is
// the single-source, NPU-style full re-evaluation built on top of the same math,
// and doubles as an independent oracle for the incremental evaluator.
class MatrixModelEvaluator {
   public:
    explicit MatrixModelEvaluator(const VrpSolution& solution);

    // Capacity step computed inline on the host (CPU matrix model).
    MatrixModelResult Evaluate() const;

    // Capacity step (the only term that scales with the scenario count) delegated
    // to the given scenario evaluator -- pass a CUDA-backed evaluator to run the
    // multi-scenario cumsum/relu/reduce on the GPU. Distance and time-window are
    // deterministic O(M*N) and always stay on the host.
    MatrixModelResult Evaluate(ScenarioCapacityEvaluator& capacityEvaluator) const;

   private:
    // Distance + both time-window variants (no capacity term).
    MatrixModelResult EvaluateDeterministic() const;
    // Inline host capacity violation (per-scenario cumsum/relu/reduce).
    double CapacityInlineHost() const;

    const VrpSolution& solution_;
    std::vector<std::vector<int>> nodeMat_;  // M ragged rows of route node ids (depot excluded)
    int routeCount_{0};
    int maxRouteLen_{0};
};

#endif
