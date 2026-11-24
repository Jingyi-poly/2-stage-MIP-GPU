#include "InstanceCVRPLIB.h"
#include "Split.h"
#include "SplitCUDA.h"

#include <climits>
#include <cmath>
#include <fstream>
#include <iostream>
#include <omp.h>

int main(int argc, char* argv[])
{
    if (argc != 3) return 2;
    int deviceCount = 0;
    const cudaError_t status = cudaGetDeviceCount(&deviceCount);
    if (status == cudaErrorNoDevice || status == cudaErrorInsufficientDriver
        || (status == cudaSuccess && deviceCount == 0))
    {
        std::cout << "CUDA test requires an available GPU\n";
        return 77;
    }
    if (status != cudaSuccess)
    {
        std::cerr << cudaGetErrorString(status) << '\n';
        return 1;
    }

    omp_set_num_threads(1);
    AlgorithmParameters ap = default_algorithm_parameters();
    ap.seed = 1;
    InstanceCVRPLIB data(argv[1], true);
    Params params(data.x_coords, data.y_coords, data.dist_mtx, data.service_time, data.demands,
                  data.vehicleCapacity, data.durationLimit, INT_MAX, data.isDurationConstraint, false, ap);
    params.generate_scenario_demands(7);
    params.update_max_vehi();

    std::ifstream input(argv[2]);
    std::vector<int> tour;
    for (int visit; input >> visit;)
        tour.push_back(visit);
    if (tour.size() != static_cast<size_t>(params.nbClients))
    {
        std::cerr << "Unexpected fixed tour length\n";
        return 1;
    }

    Individual cpu(params, false);
    cpu.chromT = tour;
    Split split(params);
    for (int s = 0; s < params.n_scenarios; ++s)
    {
        split.generalSplit(cpu, params.nbVehicles, s);
        split.generateChromR(cpu, s);
    }
    cpu.evaluateCompleteCost(params);

    Individual gpu(params, true);
    gpu.chromT = tour;
    std::vector<Individual*> batch{&gpu};
    SplitCUDA cuda(params, 1);
    cuda.reset_batch(1);
    cuda.preprocess_batch(batch, params.nbVehicles);
    cuda.generate_split_batch(1);
    cuda.evaluateOnGPU_batch(batch);

    for (int s = 0; s < params.n_scenarios; ++s)
    {
        const double cpuCost = cpu.eval.penalizedCostScen[s];
        const double gpuCost = gpu.eval.penalizedCostScen[s];
        if (!std::isfinite(gpuCost) || std::abs(cpuCost - gpuCost) > 1e-7)
        {
            std::cerr << "Scenario " << s << ": CPU " << cpuCost << ", CUDA " << gpuCost << '\n';
            return 1;
        }
    }
    std::cout << "CPU and CUDA agree for all " << params.n_scenarios << " fixed-tour scenarios\n";
    return 0;
}
