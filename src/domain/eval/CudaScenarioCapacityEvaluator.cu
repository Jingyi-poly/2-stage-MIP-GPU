#include <cuda_runtime.h>

#include <algorithm>
#include <cstdio>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "domain/eval/ScenarioCapacityEvaluator.h"

namespace {

#define CUDA_CHECK(call)                                                                                 \
    do {                                                                                                 \
        cudaError_t status = (call);                                                                     \
        if (status != cudaSuccess) {                                                                     \
            std::fprintf(stderr, "[CUDA] %s:%d %s\n", __FILE__, __LINE__, cudaGetErrorString(status));   \
            std::abort();                                                                                \
        }                                                                                                \
    } while (0)

constexpr int kBlockThreads = 256;

// One block per route; the threads of a block cooperatively sweep the scenario
// dimension. Each thread owns a grid-strided subset of scenarios and keeps that
// subset's running prefix load while walking the (short) route sequentially.
// The per-scenario load dimension is therefore fully parallel; only the
// per-node violation sum needs a block-wide reduction.
__global__ void RouteScenarioKernel(const float* __restrict__ demand, const int* __restrict__ nodeIds,
                                     const int* __restrict__ routeOffsets, int numScenarios, double vehicleCapacity,
                                     double* __restrict__ outLoads, double* __restrict__ outNodeAccViol) {
    const int route = blockIdx.x;
    const int begin = routeOffsets[route];
    const int end = routeOffsets[route + 1];

    extern __shared__ double sdata[];

    double runningAccViol = 0.0;  // only thread 0's copy is authoritative
    for (int k = begin; k < end; ++k) {
        const int nodeId = nodeIds[k];
        const long demandOffset = static_cast<long>(nodeId) * numScenarios;
        const long loadOffset = static_cast<long>(k) * numScenarios;
        const long prevLoadOffset = static_cast<long>(k - 1) * numScenarios;

        double localViol = 0.0;
        for (int s = threadIdx.x; s < numScenarios; s += blockDim.x) {
            const double prevLoad = (k == begin) ? 0.0 : outLoads[prevLoadOffset + s];
            const double load = prevLoad + static_cast<double>(demand[demandOffset + s]);
            outLoads[loadOffset + s] = load;
            const double over = load - vehicleCapacity;
            if (over > 0.0) {
                localViol += over;
            }
        }

        sdata[threadIdx.x] = localViol;
        __syncthreads();
        for (int stride = blockDim.x / 2; stride > 0; stride >>= 1) {
            if (threadIdx.x < stride) {
                sdata[threadIdx.x] += sdata[threadIdx.x + stride];
            }
            __syncthreads();
        }

        if (threadIdx.x == 0) {
            runningAccViol += sdata[0];
            outNodeAccViol[k] = runningAccViol;
        }
        __syncthreads();
    }
}

// Lean kernel: one block per route, one thread per scenario (grid-strided).
// Each thread walks the whole route keeping a single scalar running load for its
// scenario and accumulates that scenario's violation; the block then reduces to
// the route total. No O(nodes * scenarios) state is written to global memory, so
// only RouteNum() doubles are transferred back. Reads of demand[node*S + s] are
// coalesced across the consecutive scenarios held by a warp.
__global__ void RouteViolationKernel(const float* __restrict__ demand, const int* __restrict__ nodeIds,
                                     const int* __restrict__ routeOffsets, int numScenarios, double vehicleCapacity,
                                     double* __restrict__ outRouteViol) {
    const int route = blockIdx.x;
    const int begin = routeOffsets[route];
    const int end = routeOffsets[route + 1];

    extern __shared__ double sdata[];

    double threadViol = 0.0;
    for (int s = threadIdx.x; s < numScenarios; s += blockDim.x) {
        double load = 0.0;
        for (int k = begin; k < end; ++k) {
            load += static_cast<double>(demand[static_cast<long>(nodeIds[k]) * numScenarios + s]);
            const double over = load - vehicleCapacity;
            if (over > 0.0) {
                threadViol += over;
            }
        }
    }

    sdata[threadIdx.x] = threadViol;
    __syncthreads();
    for (int stride = blockDim.x / 2; stride > 0; stride >>= 1) {
        if (threadIdx.x < stride) {
            sdata[threadIdx.x] += sdata[threadIdx.x + stride];
        }
        __syncthreads();
    }
    if (threadIdx.x == 0) {
        outRouteViol[route] = sdata[0];
    }
}

// Reduces one value per thread within the block and atomically folds the block
// partial into *global. `sh` must hold at least blockDim.x doubles. Both
// __syncthreads barriers below are executed by every thread in the block.
__device__ inline void BlockReduceAndAtomicAdd(double* sh, double value, double* global) {
    sh[threadIdx.x] = value;
    __syncthreads();
    for (int stride = blockDim.x / 2; stride > 0; stride >>= 1) {
        if (threadIdx.x < stride) {
            sh[threadIdx.x] += sh[threadIdx.x + stride];
        }
        __syncthreads();
    }
    if (threadIdx.x == 0) {
        atomicAdd(global, sh[0]);
    }
    __syncthreads();
}

// Insertion-delta heavy step: one thread per scenario walks the route once,
// maintaining that scenario's running prefix load. At each of the L+1 prefix
// positions it forms the three violation contributions (no shift / +pickup /
// +pickup+delivery), the block reduces them, and one atomicAdd per block folds
// the partial into the tiny length-(L+1) v0/vp/vpd arrays. The expensive
// O(L * scenarios) reductions stay on the device; only 3*(L+1) doubles return.
__global__ void InsertionVArraysKernel(const float* __restrict__ demand, const int* __restrict__ nodeIds, int routeLen,
                                       int numScenarios, double vehicleCapacity, int pickupNodeId, int deliveryNodeId,
                                       double* __restrict__ v0, double* __restrict__ vp, double* __restrict__ vpd) {
    const int s = blockIdx.x * blockDim.x + threadIdx.x;
    const bool active = s < numScenarios;

    const double dp = active ? static_cast<double>(demand[static_cast<long>(pickupNodeId) * numScenarios + s]) : 0.0;
    const double dd = active ? static_cast<double>(demand[static_cast<long>(deliveryNodeId) * numScenarios + s]) : 0.0;

    extern __shared__ double sh[];

    double load = 0.0;
    for (int i = 0; i <= routeLen; ++i) {
        const double l = load;
        double c0 = active ? fmax(0.0, l - vehicleCapacity) : 0.0;
        double cp = active ? fmax(0.0, l + dp - vehicleCapacity) : 0.0;
        double cpd = active ? fmax(0.0, l + dp + dd - vehicleCapacity) : 0.0;

        BlockReduceAndAtomicAdd(sh, c0, &v0[i]);
        BlockReduceAndAtomicAdd(sh, cp, &vp[i]);
        BlockReduceAndAtomicAdd(sh, cpd, &vpd[i]);

        if (i < routeLen && active) {
            load += static_cast<double>(demand[static_cast<long>(nodeIds[i]) * numScenarios + s]);
        }
    }
}

// Batched insertion-delta kernel: gridDim.y selects the probe, gridDim.x the
// scenario chunk. All scenario-chunk blocks of a probe atomically fold their
// partials into that probe's slice of the packed v-arrays (offset vOffsets[p]).
// One launch therefore covers every probe in the sweep.
__global__ void InsertionVArraysBatchKernel(const float* __restrict__ demand, const int* __restrict__ nodeIds,
                                            const int* __restrict__ routeOffsets, const int* __restrict__ vOffsets,
                                            const int* __restrict__ pickupIds, const int* __restrict__ deliveryIds,
                                            int numScenarios, double vehicleCapacity, double* __restrict__ v0,
                                            double* __restrict__ vp, double* __restrict__ vpd) {
    const int probe = blockIdx.y;
    const int begin = routeOffsets[probe];
    const int routeLen = routeOffsets[probe + 1] - begin;
    const int vBase = vOffsets[probe];
    const int pickupNodeId = pickupIds[probe];
    const int deliveryNodeId = deliveryIds[probe];

    const int s = blockIdx.x * blockDim.x + threadIdx.x;
    const bool active = s < numScenarios;
    const double dp = active ? static_cast<double>(demand[static_cast<long>(pickupNodeId) * numScenarios + s]) : 0.0;
    const double dd = active ? static_cast<double>(demand[static_cast<long>(deliveryNodeId) * numScenarios + s]) : 0.0;

    extern __shared__ double sh[];

    double load = 0.0;
    for (int i = 0; i <= routeLen; ++i) {
        const double l = load;
        double c0 = active ? fmax(0.0, l - vehicleCapacity) : 0.0;
        double cp = active ? fmax(0.0, l + dp - vehicleCapacity) : 0.0;
        double cpd = active ? fmax(0.0, l + dp + dd - vehicleCapacity) : 0.0;

        BlockReduceAndAtomicAdd(sh, c0, &v0[vBase + i]);
        BlockReduceAndAtomicAdd(sh, cp, &vp[vBase + i]);
        BlockReduceAndAtomicAdd(sh, cpd, &vpd[vBase + i]);

        if (i < routeLen && active) {
            load += static_cast<double>(demand[static_cast<long>(nodeIds[begin + i]) * numScenarios + s]);
        }
    }
}

// Batched 2-opt reversal kernel: gridDim.y selects a flattened (route, a, b)
// candidate, gridDim.x the scenario chunk. Each thread reconstructs its scenario's
// prefix load just before position a, then walks the reversed segment (b, b-1, ...,
// a) accumulating the per-scenario violation. The block reduces and atomically
// folds its partial into that candidate's single output slot. One launch covers
// every reversal candidate of every route in the batch.
__global__ void ReversalViolationBatchKernel(const float* __restrict__ demand, const int* __restrict__ nodeIds,
                                             const int* __restrict__ pairBegin, const int* __restrict__ pairA,
                                             const int* __restrict__ pairB, int numScenarios, double vehicleCapacity,
                                             double* __restrict__ outViol) {
    const int pair = blockIdx.y;
    const int begin = pairBegin[pair];
    const int a = pairA[pair];
    const int b = pairB[pair];

    const int s = blockIdx.x * blockDim.x + threadIdx.x;
    const bool active = s < numScenarios;

    double viol = 0.0;
    if (active) {
        double load = 0.0;
        for (int m = 0; m < a; ++m) {  // prefix load up to position a-1
            load += static_cast<double>(demand[static_cast<long>(nodeIds[begin + m]) * numScenarios + s]);
        }
        for (int j = b; j >= a; --j) {  // reversed visit order
            load += static_cast<double>(demand[static_cast<long>(nodeIds[begin + j]) * numScenarios + s]);
            const double over = load - vehicleCapacity;
            if (over > 0.0) {
                viol += over;
            }
        }
    }

    extern __shared__ double sh[];
    BlockReduceAndAtomicAdd(sh, viol, &outViol[pair]);
}

// Per-host-thread device working set. The large read-only demand matrix is shared
// across all threads; every other buffer here is private to one thread and is only
// ever touched on that thread's own CUDA stream, so concurrent OpenMP threads need
// no lock on the hot path - they each set up, launch, and drain their own stream.
struct DeviceWorkspace {
    cudaStream_t stream = nullptr;

    int* dNodeIds = nullptr;
    int* dRouteOffsets = nullptr;
    double* dLoads = nullptr;
    double* dNodeAccViol = nullptr;
    double* dRouteViol = nullptr;
    int nodeCapacity = 0;
    int fullNodeCapacity = 0;
    int routeOffsetCapacity = 0;
    int routeViolCapacity = 0;

    int* dInsNodeIds = nullptr;
    double* dVArrays = nullptr;
    int insNodeCapacity = 0;
    int vArrayCapacity = 0;

    int* dbNodeIds = nullptr;
    int* dbRouteOffsets = nullptr;
    int* dbVOffsets = nullptr;
    int* dbPickupIds = nullptr;
    int* dbDeliveryIds = nullptr;
    double* dbVArrays = nullptr;
    int bNodeCapacity = 0;
    int bProbeCapacity = 0;
    int bVArrayCapacity = 0;

    int* dRevNodeIds = nullptr;
    int* dRevPairBegin = nullptr;
    int* dRevPairA = nullptr;
    int* dRevPairB = nullptr;
    double* dRevOut = nullptr;
    int revNodeCapacity = 0;
    int revPairCapacity = 0;
};

class CudaScenarioCapacityEvaluator : public ScenarioCapacityEvaluator {
   public:
    CudaScenarioCapacityEvaluator(const float* demandMatrix, int nodeNum, int numScenarios, double vehicleCapacity)
        : nodeNum_(nodeNum), numScenarios_(numScenarios), vehicleCapacity_(vehicleCapacity) {
        const size_t demandElems = static_cast<size_t>(nodeNum) * numScenarios;
        CUDA_CHECK(cudaMalloc(&dDemand_, demandElems * sizeof(float)));
        CUDA_CHECK(cudaMemcpy(dDemand_, demandMatrix, demandElems * sizeof(float), cudaMemcpyHostToDevice));
    }

    ~CudaScenarioCapacityEvaluator() override {
        cudaFree(dDemand_);
        for (auto& entry : workspaces_) {
            DeviceWorkspace& ws = *entry.second;
            cudaFree(ws.dNodeIds);
            cudaFree(ws.dRouteOffsets);
            cudaFree(ws.dLoads);
            cudaFree(ws.dNodeAccViol);
            cudaFree(ws.dRouteViol);
            cudaFree(ws.dInsNodeIds);
            cudaFree(ws.dVArrays);
            cudaFree(ws.dbNodeIds);
            cudaFree(ws.dbRouteOffsets);
            cudaFree(ws.dbVOffsets);
            cudaFree(ws.dbPickupIds);
            cudaFree(ws.dbDeliveryIds);
            cudaFree(ws.dbVArrays);
            cudaFree(ws.dRevNodeIds);
            cudaFree(ws.dRevPairBegin);
            cudaFree(ws.dRevPairA);
            cudaFree(ws.dRevPairB);
            cudaFree(ws.dRevOut);
            if (ws.stream != nullptr) {
                cudaStreamDestroy(ws.stream);
            }
        }
    }

    void Evaluate(const RouteBatch& batch, std::vector<double>& outLoads,
                  std::vector<double>& outNodeAccViol) override {
        DeviceWorkspace& ws = GetWorkspace();
        const int totalNodes = batch.TotalNodes();
        const int routeNum = batch.RouteNum();
        outLoads.assign(static_cast<size_t>(totalNodes) * numScenarios_, 0.0);
        outNodeAccViol.assign(totalNodes, 0.0);
        if (routeNum <= 0 || totalNodes <= 0) {
            return;
        }

        EnsureRouteCapacity(ws, totalNodes, routeNum);
        EnsureFullEvaluationCapacity(ws, totalNodes);

        CUDA_CHECK(cudaMemcpyAsync(ws.dNodeIds, batch.nodeIds.data(), totalNodes * sizeof(int), cudaMemcpyHostToDevice,
                                   ws.stream));
        CUDA_CHECK(cudaMemcpyAsync(ws.dRouteOffsets, batch.routeOffsets.data(), (routeNum + 1) * sizeof(int),
                                   cudaMemcpyHostToDevice, ws.stream));

        const int threads = kBlockThreads;
        const size_t sharedBytes = threads * sizeof(double);
        RouteScenarioKernel<<<routeNum, threads, sharedBytes, ws.stream>>>(
            dDemand_, ws.dNodeIds, ws.dRouteOffsets, numScenarios_, vehicleCapacity_, ws.dLoads, ws.dNodeAccViol);
        CUDA_CHECK(cudaGetLastError());

        CUDA_CHECK(cudaMemcpyAsync(outLoads.data(), ws.dLoads,
                                   static_cast<size_t>(totalNodes) * numScenarios_ * sizeof(double),
                                   cudaMemcpyDeviceToHost, ws.stream));
        CUDA_CHECK(cudaMemcpyAsync(outNodeAccViol.data(), ws.dNodeAccViol, totalNodes * sizeof(double),
                                   cudaMemcpyDeviceToHost, ws.stream));
        CUDA_CHECK(cudaStreamSynchronize(ws.stream));
    }

    void EvaluateRouteViolations(const RouteBatch& batch, std::vector<double>& outRouteViol) override {
        DeviceWorkspace& ws = GetWorkspace();
        const int totalNodes = batch.TotalNodes();
        const int routeNum = batch.RouteNum();
        outRouteViol.assign(routeNum, 0.0);
        if (routeNum <= 0 || totalNodes <= 0) {
            return;
        }

        EnsureRouteCapacity(ws, totalNodes, routeNum);
        if (routeNum > ws.routeViolCapacity) {
            cudaFree(ws.dRouteViol);
            CUDA_CHECK(cudaMalloc(&ws.dRouteViol, routeNum * sizeof(double)));
            ws.routeViolCapacity = routeNum;
        }

        CUDA_CHECK(cudaMemcpyAsync(ws.dNodeIds, batch.nodeIds.data(), totalNodes * sizeof(int), cudaMemcpyHostToDevice,
                                   ws.stream));
        CUDA_CHECK(cudaMemcpyAsync(ws.dRouteOffsets, batch.routeOffsets.data(), (routeNum + 1) * sizeof(int),
                                   cudaMemcpyHostToDevice, ws.stream));

        const int threads = kBlockThreads;
        const size_t sharedBytes = threads * sizeof(double);
        RouteViolationKernel<<<routeNum, threads, sharedBytes, ws.stream>>>(
            dDemand_, ws.dNodeIds, ws.dRouteOffsets, numScenarios_, vehicleCapacity_, ws.dRouteViol);
        CUDA_CHECK(cudaGetLastError());
        CUDA_CHECK(cudaMemcpyAsync(outRouteViol.data(), ws.dRouteViol, routeNum * sizeof(double),
                                   cudaMemcpyDeviceToHost, ws.stream));
        CUDA_CHECK(cudaStreamSynchronize(ws.stream));
    }

    void EvaluateInsertionDeltas(const std::vector<int>& routeNodeIds, int pickupNodeId, int deliveryNodeId,
                                 std::vector<double>& outDeltas) override {
        DeviceWorkspace& ws = GetWorkspace();
        const int routeLen = static_cast<int>(routeNodeIds.size());
        const int m = routeLen + 1;

        EnsureInsertionCapacity(ws, routeLen, m);
        if (routeLen > 0) {
            CUDA_CHECK(cudaMemcpyAsync(ws.dInsNodeIds, routeNodeIds.data(), routeLen * sizeof(int),
                                       cudaMemcpyHostToDevice, ws.stream));
        }
        // v0/vp/vpd packed contiguously in dVArrays ([0,m) / [m,2m) / [2m,3m)) so
        // a single memset and a single device-to-host copy cover all three.
        CUDA_CHECK(cudaMemsetAsync(ws.dVArrays, 0, static_cast<size_t>(3) * m * sizeof(double), ws.stream));

        const int threads = kBlockThreads;
        const int blocks = (numScenarios_ + threads - 1) / threads;
        const size_t sharedBytes = threads * sizeof(double);
        InsertionVArraysKernel<<<blocks, threads, sharedBytes, ws.stream>>>(
            dDemand_, ws.dInsNodeIds, routeLen, numScenarios_, vehicleCapacity_, pickupNodeId, deliveryNodeId,
            ws.dVArrays, ws.dVArrays + m, ws.dVArrays + 2 * m);
        CUDA_CHECK(cudaGetLastError());

        std::vector<double> packed(static_cast<size_t>(3) * m);
        CUDA_CHECK(cudaMemcpyAsync(packed.data(), ws.dVArrays, static_cast<size_t>(3) * m * sizeof(double),
                                   cudaMemcpyDeviceToHost, ws.stream));
        CUDA_CHECK(cudaStreamSynchronize(ws.stream));
        std::vector<double> v0(packed.begin(), packed.begin() + m);
        std::vector<double> vp(packed.begin() + m, packed.begin() + 2 * m);
        std::vector<double> vpd(packed.begin() + 2 * m, packed.begin() + 3 * m);

        scenario_eval_detail::CombineInsertionDeltas(v0, vp, vpd, outDeltas);
    }

    void EvaluateInsertionDeltasBatch(const InsertionProbeBatch& batch,
                                      std::vector<std::vector<double>>& outDeltas) override {
        DeviceWorkspace& ws = GetWorkspace();
        const int probeNum = batch.ProbeNum();
        outDeltas.assign(probeNum, {});
        if (probeNum <= 0) {
            return;
        }

        // Packed per-probe v-array layout: probe p owns [vOffsets[p], vOffsets[p+1]).
        std::vector<int> vOffsets(probeNum + 1, 0);
        for (int p = 0; p < probeNum; ++p) {
            vOffsets[p + 1] = vOffsets[p] + (batch.RouteLen(p) + 1);
        }
        const int totalV = vOffsets[probeNum];
        const int totalNodes = static_cast<int>(batch.nodeIds.size());

        EnsureBatchCapacity(ws, totalNodes, probeNum, totalV);

        if (totalNodes > 0) {
            CUDA_CHECK(cudaMemcpyAsync(ws.dbNodeIds, batch.nodeIds.data(), totalNodes * sizeof(int),
                                       cudaMemcpyHostToDevice, ws.stream));
        }
        CUDA_CHECK(cudaMemcpyAsync(ws.dbRouteOffsets, batch.routeOffsets.data(), (probeNum + 1) * sizeof(int),
                                   cudaMemcpyHostToDevice, ws.stream));
        CUDA_CHECK(cudaMemcpyAsync(ws.dbVOffsets, vOffsets.data(), (probeNum + 1) * sizeof(int),
                                   cudaMemcpyHostToDevice, ws.stream));
        CUDA_CHECK(cudaMemcpyAsync(ws.dbPickupIds, batch.pickupIds.data(), probeNum * sizeof(int),
                                   cudaMemcpyHostToDevice, ws.stream));
        CUDA_CHECK(cudaMemcpyAsync(ws.dbDeliveryIds, batch.deliveryIds.data(), probeNum * sizeof(int),
                                   cudaMemcpyHostToDevice, ws.stream));
        CUDA_CHECK(cudaMemsetAsync(ws.dbVArrays, 0, static_cast<size_t>(3) * totalV * sizeof(double), ws.stream));

        const int threads = kBlockThreads;
        const dim3 grid((numScenarios_ + threads - 1) / threads, probeNum);
        const size_t sharedBytes = threads * sizeof(double);
        InsertionVArraysBatchKernel<<<grid, threads, sharedBytes, ws.stream>>>(
            dDemand_, ws.dbNodeIds, ws.dbRouteOffsets, ws.dbVOffsets, ws.dbPickupIds, ws.dbDeliveryIds, numScenarios_,
            vehicleCapacity_, ws.dbVArrays, ws.dbVArrays + totalV, ws.dbVArrays + 2 * totalV);
        CUDA_CHECK(cudaGetLastError());

        std::vector<double> packed(static_cast<size_t>(3) * totalV);
        CUDA_CHECK(cudaMemcpyAsync(packed.data(), ws.dbVArrays, static_cast<size_t>(3) * totalV * sizeof(double),
                                   cudaMemcpyDeviceToHost, ws.stream));
        CUDA_CHECK(cudaStreamSynchronize(ws.stream));

        for (int p = 0; p < probeNum; ++p) {
            const int m = batch.RouteLen(p) + 1;
            const int base = vOffsets[p];
            std::vector<double> v0(packed.begin() + base, packed.begin() + base + m);
            std::vector<double> vp(packed.begin() + totalV + base, packed.begin() + totalV + base + m);
            std::vector<double> vpd(packed.begin() + 2 * totalV + base, packed.begin() + 2 * totalV + base + m);
            scenario_eval_detail::CombineInsertionDeltas(v0, vp, vpd, outDeltas[p]);
        }
    }

    void EvaluateReversalViolationsBatch(const RouteBatch& batch,
                                         std::vector<std::vector<double>>& outViol) override {
        const int routeNum = batch.RouteNum();
        outViol.assign(routeNum, {});
        if (routeNum <= 0) {
            return;
        }

        // Flatten every (route, a, b) reversal candidate into parallel arrays. The
        // node id for global position k of route r is nodeIds[routeOffsets[r] + k];
        // pairBegin stores that base so the kernel can index directly.
        std::vector<int> pairBegin;
        std::vector<int> pairA;
        std::vector<int> pairB;
        std::vector<int> outBaseOfRoute(routeNum, 0);  // first flat-pair index owned by route r
        for (int r = 0; r < routeNum; ++r) {
            const int begin = batch.routeOffsets[r];
            const int len = batch.routeOffsets[r + 1] - begin;
            outViol[r].assign(static_cast<size_t>(len) * len, 0.0);
            outBaseOfRoute[r] = static_cast<int>(pairBegin.size());
            for (int a = 1; a < len; ++a) {
                for (int b = a; b < len; ++b) {
                    pairBegin.push_back(begin);
                    pairA.push_back(a);
                    pairB.push_back(b);
                }
            }
        }
        const int pairNum = static_cast<int>(pairBegin.size());
        const int totalNodes = batch.TotalNodes();
        if (pairNum <= 0 || totalNodes <= 0) {
            return;
        }

        DeviceWorkspace& ws = GetWorkspace();
        EnsureReversalCapacity(ws, totalNodes, pairNum);

        CUDA_CHECK(cudaMemcpyAsync(ws.dRevNodeIds, batch.nodeIds.data(), totalNodes * sizeof(int),
                                   cudaMemcpyHostToDevice, ws.stream));
        CUDA_CHECK(cudaMemcpyAsync(ws.dRevPairBegin, pairBegin.data(), pairNum * sizeof(int), cudaMemcpyHostToDevice,
                                   ws.stream));
        CUDA_CHECK(cudaMemcpyAsync(ws.dRevPairA, pairA.data(), pairNum * sizeof(int), cudaMemcpyHostToDevice,
                                   ws.stream));
        CUDA_CHECK(cudaMemcpyAsync(ws.dRevPairB, pairB.data(), pairNum * sizeof(int), cudaMemcpyHostToDevice,
                                   ws.stream));
        CUDA_CHECK(cudaMemsetAsync(ws.dRevOut, 0, pairNum * sizeof(double), ws.stream));

        const int threads = kBlockThreads;
        const dim3 grid((numScenarios_ + threads - 1) / threads, pairNum);
        const size_t sharedBytes = threads * sizeof(double);
        ReversalViolationBatchKernel<<<grid, threads, sharedBytes, ws.stream>>>(
            dDemand_, ws.dRevNodeIds, ws.dRevPairBegin, ws.dRevPairA, ws.dRevPairB, numScenarios_, vehicleCapacity_,
            ws.dRevOut);
        CUDA_CHECK(cudaGetLastError());

        std::vector<double> flat(pairNum);
        CUDA_CHECK(cudaMemcpyAsync(flat.data(), ws.dRevOut, pairNum * sizeof(double), cudaMemcpyDeviceToHost,
                                   ws.stream));
        CUDA_CHECK(cudaStreamSynchronize(ws.stream));

        // Scatter the flat results back into each route's L*L matrix.
        for (int r = 0; r < routeNum; ++r) {
            const int len = batch.routeOffsets[r + 1] - batch.routeOffsets[r];
            int idx = outBaseOfRoute[r];
            for (int a = 1; a < len; ++a) {
                for (int b = a; b < len; ++b) {
                    outViol[r][static_cast<size_t>(a) * len + b] = flat[idx++];
                }
            }
        }
    }

    int NumScenarios() const override { return numScenarios_; }
    const char* Backend() const override { return "cuda"; }

   private:
    // Returns the calling thread's workspace, creating it (and its stream) on first
    // use. Only the map lookup/insert is locked; the returned workspace lives in a
    // unique_ptr so its address is stable across rehashes and is used lock-free.
    DeviceWorkspace& GetWorkspace() {
        const std::thread::id id = std::this_thread::get_id();
        std::lock_guard<std::mutex> lock(workspaceMutex_);
        auto it = workspaces_.find(id);
        if (it != workspaces_.end()) {
            return *it->second;
        }
        auto ws = std::make_unique<DeviceWorkspace>();
        CUDA_CHECK(cudaStreamCreate(&ws->stream));
        DeviceWorkspace& ref = *ws;
        workspaces_.emplace(id, std::move(ws));
        return ref;
    }

    // Route indices are shared by both evaluator paths. Keep their capacity
    // independent from the O(nodes * scenarios) full-evaluation state so the
    // lean route-violation path does not allocate buffers it never reads.
    void EnsureRouteCapacity(DeviceWorkspace& ws, int totalNodes, int routeNum) {
        if (totalNodes > ws.nodeCapacity) {
            cudaFree(ws.dNodeIds);
            CUDA_CHECK(cudaMalloc(&ws.dNodeIds, totalNodes * sizeof(int)));
            ws.nodeCapacity = totalNodes;
        }
        if (routeNum + 1 > ws.routeOffsetCapacity) {
            cudaFree(ws.dRouteOffsets);
            CUDA_CHECK(cudaMalloc(&ws.dRouteOffsets, (routeNum + 1) * sizeof(int)));
            ws.routeOffsetCapacity = routeNum + 1;
        }
    }

    void EnsureFullEvaluationCapacity(DeviceWorkspace& ws, int totalNodes) {
        if (totalNodes <= ws.fullNodeCapacity) {
            return;
        }
        cudaFree(ws.dLoads);
        cudaFree(ws.dNodeAccViol);
        CUDA_CHECK(cudaMalloc(&ws.dLoads, static_cast<size_t>(totalNodes) * numScenarios_ * sizeof(double)));
        CUDA_CHECK(cudaMalloc(&ws.dNodeAccViol, totalNodes * sizeof(double)));
        ws.fullNodeCapacity = totalNodes;
    }

    void EnsureInsertionCapacity(DeviceWorkspace& ws, int routeLen, int m) {
        if (routeLen > ws.insNodeCapacity) {
            cudaFree(ws.dInsNodeIds);
            CUDA_CHECK(cudaMalloc(&ws.dInsNodeIds, std::max(1, routeLen) * sizeof(int)));
            ws.insNodeCapacity = routeLen;
        }
        if (m > ws.vArrayCapacity) {
            cudaFree(ws.dVArrays);
            CUDA_CHECK(cudaMalloc(&ws.dVArrays, static_cast<size_t>(3) * m * sizeof(double)));
            ws.vArrayCapacity = m;
        }
    }

    void EnsureBatchCapacity(DeviceWorkspace& ws, int totalNodes, int probeNum, int totalV) {
        if (totalNodes > ws.bNodeCapacity) {
            cudaFree(ws.dbNodeIds);
            CUDA_CHECK(cudaMalloc(&ws.dbNodeIds, std::max(1, totalNodes) * sizeof(int)));
            ws.bNodeCapacity = totalNodes;
        }
        if (probeNum > ws.bProbeCapacity) {
            cudaFree(ws.dbRouteOffsets);
            cudaFree(ws.dbVOffsets);
            cudaFree(ws.dbPickupIds);
            cudaFree(ws.dbDeliveryIds);
            CUDA_CHECK(cudaMalloc(&ws.dbRouteOffsets, (probeNum + 1) * sizeof(int)));
            CUDA_CHECK(cudaMalloc(&ws.dbVOffsets, (probeNum + 1) * sizeof(int)));
            CUDA_CHECK(cudaMalloc(&ws.dbPickupIds, probeNum * sizeof(int)));
            CUDA_CHECK(cudaMalloc(&ws.dbDeliveryIds, probeNum * sizeof(int)));
            ws.bProbeCapacity = probeNum;
        }
        if (totalV > ws.bVArrayCapacity) {
            cudaFree(ws.dbVArrays);
            CUDA_CHECK(cudaMalloc(&ws.dbVArrays, static_cast<size_t>(3) * totalV * sizeof(double)));
            ws.bVArrayCapacity = totalV;
        }
    }

    void EnsureReversalCapacity(DeviceWorkspace& ws, int totalNodes, int pairNum) {
        if (totalNodes > ws.revNodeCapacity) {
            cudaFree(ws.dRevNodeIds);
            CUDA_CHECK(cudaMalloc(&ws.dRevNodeIds, std::max(1, totalNodes) * sizeof(int)));
            ws.revNodeCapacity = totalNodes;
        }
        if (pairNum > ws.revPairCapacity) {
            cudaFree(ws.dRevPairBegin);
            cudaFree(ws.dRevPairA);
            cudaFree(ws.dRevPairB);
            cudaFree(ws.dRevOut);
            CUDA_CHECK(cudaMalloc(&ws.dRevPairBegin, pairNum * sizeof(int)));
            CUDA_CHECK(cudaMalloc(&ws.dRevPairA, pairNum * sizeof(int)));
            CUDA_CHECK(cudaMalloc(&ws.dRevPairB, pairNum * sizeof(int)));
            CUDA_CHECK(cudaMalloc(&ws.dRevOut, pairNum * sizeof(double)));
            ws.revPairCapacity = pairNum;
        }
    }

    int nodeNum_;
    int numScenarios_;
    double vehicleCapacity_;

    float* dDemand_ = nullptr;  // shared, read-only after construction

    std::unordered_map<std::thread::id, std::unique_ptr<DeviceWorkspace>> workspaces_;
    std::mutex workspaceMutex_;
};

}  // namespace

bool CudaScenarioCapacityEvaluatorAvailable() {
    int deviceCount = 0;
    cudaError_t status = cudaGetDeviceCount(&deviceCount);
    return status == cudaSuccess && deviceCount > 0;
}

std::unique_ptr<ScenarioCapacityEvaluator> CreateCudaScenarioCapacityEvaluator(const float* demandMatrix, int nodeNum,
                                                                               int numScenarios,
                                                                               double vehicleCapacity) {
    return std::make_unique<CudaScenarioCapacityEvaluator>(demandMatrix, nodeNum, numScenarios, vehicleCapacity);
}
