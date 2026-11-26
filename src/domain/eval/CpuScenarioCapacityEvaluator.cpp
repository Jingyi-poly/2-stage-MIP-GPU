#include <algorithm>
#include <iostream>

#include "domain/eval/ScenarioCapacityEvaluator.h"

namespace {

class CpuScenarioCapacityEvaluator : public ScenarioCapacityEvaluator {
   public:
    CpuScenarioCapacityEvaluator(const float* demandMatrix, int nodeNum, int numScenarios, double vehicleCapacity)
        : demandMatrix_(demandMatrix, demandMatrix + static_cast<size_t>(nodeNum) * numScenarios),
          nodeNum_(nodeNum),
          numScenarios_(numScenarios),
          vehicleCapacity_(vehicleCapacity) {}

    void Evaluate(const RouteBatch& batch, std::vector<double>& outLoads,
                  std::vector<double>& outNodeAccViol) override {
        const int totalNodes = batch.TotalNodes();
        const int routeNum = batch.RouteNum();
        outLoads.assign(static_cast<size_t>(totalNodes) * numScenarios_, 0.0);
        outNodeAccViol.assign(totalNodes, 0.0);

        std::vector<double> load(numScenarios_, 0.0);
        for (int route = 0; route < routeNum; ++route) {
            const int begin = batch.routeOffsets[route];
            const int end = batch.routeOffsets[route + 1];

            std::fill(load.begin(), load.end(), 0.0);
            double accViol = 0.0;
            for (int k = begin; k < end; ++k) {
                const int nodeId = batch.nodeIds[k];
                const size_t demandOffset = static_cast<size_t>(nodeId) * numScenarios_;
                const size_t loadOffset = static_cast<size_t>(k) * numScenarios_;

                double nodeViol = 0.0;
                for (int s = 0; s < numScenarios_; ++s) {
                    load[s] += static_cast<double>(demandMatrix_[demandOffset + s]);
                    outLoads[loadOffset + s] = load[s];
                    nodeViol += std::max(0.0, load[s] - vehicleCapacity_);
                }
                accViol += nodeViol;
                outNodeAccViol[k] = accViol;
            }
        }
    }

    void EvaluateRouteViolations(const RouteBatch& batch, std::vector<double>& outRouteViol) override {
        const int routeNum = batch.RouteNum();
        outRouteViol.assign(routeNum, 0.0);

        std::vector<double> load(numScenarios_, 0.0);
        for (int route = 0; route < routeNum; ++route) {
            const int begin = batch.routeOffsets[route];
            const int end = batch.routeOffsets[route + 1];

            std::fill(load.begin(), load.end(), 0.0);
            double routeViol = 0.0;
            for (int k = begin; k < end; ++k) {
                const size_t demandOffset = static_cast<size_t>(batch.nodeIds[k]) * numScenarios_;
                for (int s = 0; s < numScenarios_; ++s) {
                    load[s] += static_cast<double>(demandMatrix_[demandOffset + s]);
                    routeViol += std::max(0.0, load[s] - vehicleCapacity_);
                }
            }
            outRouteViol[route] = routeViol;
        }
    }

    void EvaluateInsertionDeltas(const std::vector<int>& routeNodeIds, int pickupNodeId, int deliveryNodeId,
                                 std::vector<double>& outDeltas) override {
        const int routeLen = static_cast<int>(routeNodeIds.size());
        const int m = routeLen + 1;
        const size_t pickupOffset = static_cast<size_t>(pickupNodeId) * numScenarios_;
        const size_t deliveryOffset = static_cast<size_t>(deliveryNodeId) * numScenarios_;

        std::vector<double> v0(m, 0.0), vp(m, 0.0), vpd(m, 0.0);
        std::vector<double> load(numScenarios_, 0.0);
        for (int i = 0; i < m; ++i) {
            double c0 = 0.0, cp = 0.0, cpd = 0.0;
            for (int s = 0; s < numScenarios_; ++s) {
                const double l = load[s];
                const double dp = static_cast<double>(demandMatrix_[pickupOffset + s]);
                const double dd = static_cast<double>(demandMatrix_[deliveryOffset + s]);
                c0 += std::max(0.0, l - vehicleCapacity_);
                cp += std::max(0.0, l + dp - vehicleCapacity_);
                cpd += std::max(0.0, l + dp + dd - vehicleCapacity_);
            }
            v0[i] = c0;
            vp[i] = cp;
            vpd[i] = cpd;

            if (i < routeLen) {
                const size_t nodeOffset = static_cast<size_t>(routeNodeIds[i]) * numScenarios_;
                for (int s = 0; s < numScenarios_; ++s) {
                    load[s] += static_cast<double>(demandMatrix_[nodeOffset + s]);
                }
            }
        }

        scenario_eval_detail::CombineInsertionDeltas(v0, vp, vpd, outDeltas);
    }

    void EvaluateInsertionDeltasBatch(const InsertionProbeBatch& batch,
                                      std::vector<std::vector<double>>& outDeltas) override {
        const int probeNum = batch.ProbeNum();
        outDeltas.resize(probeNum);
        for (int p = 0; p < probeNum; ++p) {
            const int begin = batch.routeOffsets[p];
            const int routeLen = batch.RouteLen(p);
            std::vector<int> route(batch.nodeIds.begin() + begin, batch.nodeIds.begin() + begin + routeLen);
            EvaluateInsertionDeltas(route, batch.pickupIds[p], batch.deliveryIds[p], outDeltas[p]);
        }
    }

    void EvaluateReversalViolationsBatch(const RouteBatch& batch,
                                         std::vector<std::vector<double>>& outViol) override {
        const int routeNum = batch.RouteNum();
        outViol.assign(routeNum, {});

        std::vector<double> startLoad(numScenarios_, 0.0);  // prefix load up to position a-1
        std::vector<double> work(numScenarios_, 0.0);       // running load along the reversed segment
        for (int r = 0; r < routeNum; ++r) {
            const int begin = batch.routeOffsets[r];
            const int len = batch.routeOffsets[r + 1] - begin;
            outViol[r].assign(static_cast<size_t>(len) * len, 0.0);

            std::fill(startLoad.begin(), startLoad.end(), 0.0);
            for (int a = 1; a < len; ++a) {
                // Advance prefix so startLoad == load just before position a.
                const size_t prevOffset = static_cast<size_t>(batch.nodeIds[begin + a - 1]) * numScenarios_;
                for (int s = 0; s < numScenarios_; ++s) {
                    startLoad[s] += static_cast<double>(demandMatrix_[prevOffset + s]);
                }
                for (int b = a; b < len; ++b) {
                    std::copy(startLoad.begin(), startLoad.end(), work.begin());
                    double viol = 0.0;
                    // Reversed visit order: position b, b-1, ..., a.
                    for (int j = b; j >= a; --j) {
                        const size_t off = static_cast<size_t>(batch.nodeIds[begin + j]) * numScenarios_;
                        for (int s = 0; s < numScenarios_; ++s) {
                            work[s] += static_cast<double>(demandMatrix_[off + s]);
                            viol += std::max(0.0, work[s] - vehicleCapacity_);
                        }
                    }
                    outViol[r][static_cast<size_t>(a) * len + b] = viol;
                }
            }
        }
    }

    int NumScenarios() const override { return numScenarios_; }
    const char* Backend() const override { return "cpu"; }

   private:
    std::vector<float> demandMatrix_;
    int nodeNum_;
    int numScenarios_;
    double vehicleCapacity_;
};

}  // namespace

#ifdef USE_CUDA
// Implemented in CudaScenarioCapacityEvaluator.cu.
bool CudaScenarioCapacityEvaluatorAvailable();
std::unique_ptr<ScenarioCapacityEvaluator> CreateCudaScenarioCapacityEvaluator(const float* demandMatrix, int nodeNum,
                                                                               int numScenarios,
                                                                               double vehicleCapacity);
#endif

std::unique_ptr<ScenarioCapacityEvaluator> MakeScenarioCapacityEvaluator(const float* demandMatrix, int nodeNum,
                                                                         int numScenarios, double vehicleCapacity,
                                                                         EvaluatorBackend backend) {
#ifdef USE_CUDA
    const bool cudaAvailable = CudaScenarioCapacityEvaluatorAvailable();
    if (backend == EvaluatorBackend::kCuda || backend == EvaluatorBackend::kAuto) {
        if (cudaAvailable) {
            return CreateCudaScenarioCapacityEvaluator(demandMatrix, nodeNum, numScenarios, vehicleCapacity);
        }
        if (backend == EvaluatorBackend::kCuda) {
            std::cerr << "[ScenarioCapacityEvaluator] CUDA requested but no device available; falling back to CPU."
                      << std::endl;
        }
    }
#else
    if (backend == EvaluatorBackend::kCuda) {
        std::cerr << "[ScenarioCapacityEvaluator] Built without USE_CUDA; falling back to CPU." << std::endl;
    }
#endif
    return std::make_unique<CpuScenarioCapacityEvaluator>(demandMatrix, nodeNum, numScenarios, vehicleCapacity);
}
