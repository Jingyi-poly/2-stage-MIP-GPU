
#ifndef SPLIT_CUDA_H
#define SPLIT_CUDA_H

#include "Params.h"
#include "Individual.h"
#include <cuda_runtime.h>
#include <omp.h>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <new>
#include <stdexcept>
#include <string>


struct ClientSplitCUDA
{
	double demand;
	// std::vector<double> demands_scenarios;
	double serviceTime;
	double d0_x;
	double dx_0;
	double dnext;
	ClientSplitCUDA() : demand(0.), serviceTime(0.), d0_x(0.), dx_0(0.), dnext(0.) {};
};
// This is transformed into a matrix:
//     [demand, serviceTime, d0_x, dx_0, dnext]

// Simple Deque which is used for all Linear Split algorithms
struct Trivial_DequeCUDA
{
	std::vector <int> myDeque; // Simply a vector structure to keep the elements of the queue
	int indexFront; // Index of the front element
	int indexBack; // Index of the back element
	inline void pop_front(){indexFront++;} // Removes the front element of the queue D
	inline void pop_back(){indexBack--;} // Removes the back element of the queue D
	inline void push_back(int i){indexBack++; myDeque[indexBack] = i;} // Appends a new element to the back of the queue D
	inline int get_front(){return myDeque[indexFront];}
	inline int get_next_front(){return myDeque[indexFront + 1];}
	inline int get_back(){return myDeque[indexBack];}
	void reset(int firstNode) { myDeque[0] = firstNode; indexBack = 0; indexFront = 0; }
	inline int size(){return indexBack - indexFront + 1;}
	
	Trivial_DequeCUDA(int nbElements, int firstNode)
	{
		myDeque = std::vector <int>(nbElements);
		myDeque[0] = firstNode;
		indexBack = 0;
		indexFront = 0;
	}
};

class SplitCUDA
{

 private:
  const int size_clisplit = 5;
  const int idx_demand = 0;
  const int idx_serviceTime = 1;
  const int idx_d0_x = 2;
  const int idx_dx_0 = 3;
  const int idx_dnext = 4;

  const Params & params ;
  int * maxVehicles = nullptr;
  std::vector<int> maxVehicles_host;

  int n,m,n_scen;
  int maxBatchSize;

  /* Auxiliary data structures to run the Linear Split algorithm */
  // std::vector < ClientSplitCUDA > cliSplit;
  double * cliSplit_demand = nullptr;
  double * cliSplit_serviceTime = nullptr;
  double * cliSplit_d0_x = nullptr;
  double * cliSplit_dx_0 = nullptr;
  double * cliSplit_dnext = nullptr;
  //  std::vector < std::vector < double > > potential;  // Potential vector
  double * potential = nullptr;
  // std::vector < std::vector < int > > pred;  // Indice of the predecessor in an optimal path
  int * pred = nullptr;
  int * pred_host = nullptr;
  // std::vector <double> sumDistance; // sumDistance[i] for i > 1 contains the sum of distances : sum_{k=1}^{i-1} d_{k,k+1}
  double * sumDistance = nullptr;
  // std::vector <double> sumLoad; // sumLoad[i] for i >= 1 contains the sum of loads : sum_{k=1}^{i} q_k
  double * sumLoad = nullptr;
  // std::vector <double> sumService; // sumService[i] for i >= 1 contains the sum of service time : sum_{k=1}^{i} s_k
  double * sumService = nullptr;

  double * totalDemands = nullptr;
  int * myDeque = nullptr;
  double * demand_host = nullptr;
  double * host_d0_x = nullptr;
  double * host_dx_0 = nullptr;
  double * host_sumDistance = nullptr;
  bool predHostPinned = false;
  bool demandHostPinned = false;
  bool d0HostPinned = false;
  bool dxHostPinned = false;
  bool sumDistanceHostPinned = false;
  cudaStream_t stream = nullptr;

  // GPU eval members
  double * d_timeCost = nullptr;      // [m_full * m_full] distance matrix on device
  double * d_allClientDemands = nullptr; // [m_full * n_scen], uploaded once
  int * d_chromT = nullptr;           // [maxBatchSize * nbClients] chromT on device
  int * h_chromT = nullptr;           // pinned host chromT staging
  double * d_evalResults = nullptr;   // [maxBatchSize * n_scen * 4] per-scen results (dist,capEx,durEx,nRoutes)
  double * d_meanCostSums = nullptr;  // [maxBatchSize] exact-solver GPU reductions
  double * h_evalResults = nullptr;   // pinned host for results
  bool chromTHostPinned = false;
  bool evalResultsHostPinned = false;
  int m_full = 0;                     // original m = nbClients+1 (never changes with setActiveClients)
  std::size_t freeBeforeBytes = 0;
  std::size_t freeAfterBytes = 0;
  std::size_t allocatedDeviceBytes = 0;
  std::size_t requestedDeviceBytes = 0;

  template <typename T>
  static void allocateHostBuffer(
      T *& buffer, std::size_t count, bool & isPinned, const char * label)
  {
    const std::size_t bytes = count * sizeof(T);
    void * raw = nullptr;
    const cudaError_t status = cudaMallocHost(&raw, bytes);
    if (status == cudaSuccess)
    {
      buffer = static_cast<T *>(raw);
      isPinned = true;
      return;
    }

    // WSL and container runtimes can impose a much smaller pinned-memory
    // ceiling than either system RAM or device RAM.  Pageable memory remains
    // valid for cudaMemcpyAsync (the runtime stages it as needed), so use it as
    // a correctness-preserving fallback for million-scenario experiments.
    cudaGetLastError();
    buffer = static_cast<T *>(std::malloc(bytes));
    if (buffer == nullptr)
      throw std::bad_alloc();
    isPinned = false;
    std::cerr << "GPU_HOST_MEMORY_FALLBACK buffer=" << label
              << " bytes=" << bytes
              << " cuda_error=\"" << cudaGetErrorString(status) << "\"\n";
  }

  template <typename T>
  static void freeHostBuffer(T *& buffer, bool isPinned) noexcept
  {
    if (buffer == nullptr)
      return;
    if (isPinned)
      cudaFreeHost(buffer);
    else
      std::free(buffer);
    buffer = nullptr;
  }



  //  // To be called with i < j only
  //  // Computes the cost of propagating the label i until j
  inline double propagate(int i, int j, int k);
  //  {
  // 	 return potential[k][i] + sumDistance[j] - sumDistance[i + 1] + cliSplit[i + 1].d0_x + cliSplit[j].dx_0
  // 		 + params.penaltyCapacity * std::max<double>(sumLoad[j] - sumLoad[i] - params.vehicleCapacity, 0.);
  //  }

  //  // Tests if i dominates j as a predecessor for all nodes x >= j+1
  //  // We assume that i < j
   inline bool dominates(int i, int j, int k);
  //  {
  // 	 return potential[k][j] + cliSplit[j + 1].d0_x > potential[k][i] + cliSplit[i + 1].d0_x + sumDistance[j + 1] - sumDistance[i + 1]
  // 		 + params.penaltyCapacity * (sumLoad[j] - sumLoad[i]);
  //  }

  //  // Tests if j dominates i as a predecessor for all nodes x >= j+1
  //  // We assume that i < j
   inline bool dominatesRight(int i, int j, int k);
  //  {
  // 	 return potential[k][j] + cliSplit[j + 1].d0_x < potential[k][i] + cliSplit[i + 1].d0_x + sumDistance[j + 1] - sumDistance[i + 1] + MY_EPSILON;
  //  }

  //   Split for unlimited fleet
    // int splitSimple(Individual & indiv, int idx_scen);

  //   Split for limited fleet
    // int splitLF(Individual & indiv, int idx_scen);


public:

  // General Split function (tests the unlimited fleet, and only if it does not produce a feasible solution, runs the Split algorithm for limited fleet)
  void preprocess(Individual & indiv, int nbMaxVehicles);

  void copy_vec(double * source, double * tar, int n_ele){
    if (!tar){
      cudaMalloc(&tar, n_ele * sizeof(double));
    }
    cudaMemcpy(tar, source, n_ele * sizeof(double), cudaMemcpyHostToDevice);
  }
  void copy_vecVec(std::vector<double> source, double * tar){
    if (!tar){
      cudaMalloc(&tar, source.size() * sizeof(double));
    }
    cudaMemcpy(tar, source.data(), source.size() * sizeof(double), cudaMemcpyHostToDevice);
  }

  static inline void cudaCheck(cudaError_t e, const char* what) {
    if (e != cudaSuccess) throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(e));
  }

  void printMat(const int n, const int m, double * mat){
	std::cout<<"\n\n\n\n\n";
    double h_mat[n*m];
    // double * h_mat = new double[2000]; // allocate on host
    // double * h_mat = (double*)malloc(n*mm*sizeof(double)); // allocate on host
    // Copy from device to host
    cudaMemcpy(&h_mat, mat, n * m * sizeof(double), cudaMemcpyDeviceToHost);
    for (int i = 0; i < n; ++i){
      for (int j = 0; j < m; ++j){
        std::cout<<h_mat[i*m + j]<<" , ";
      }
      std::cout<<"|\n";
    }
  }


  void printPotential(){
    std::cout<<"Potential: n_scen->"<<n_scen<<"\n";

    double h_potential[n];
    // double * h_mat = new double[2000]; // allocate on host
    // double * h_mat = (double*)malloc(n*mm*sizeof(double)); // allocate on host
    // Copy from device to host

    for (int i = 0; i < n_scen; ++i){
      cudaMemcpy(&h_potential, potential,  n * sizeof(double), cudaMemcpyDeviceToHost);
        for (int k = 0; k < n; ++k){
          std::cout<<h_potential[ k]<<", ";
        }
        std::cout<<"\n";
        std::cout<<"--------------\n";
        break;
    }
    std::cout<<"Potential: \n";
  }


  void printPred(){
    std::cout<<n_scen * m * n<<"\n\n\n\n";

    int h_pred[ m * n];
    // double * h_mat = new double[2000]; // allocate on host
    // double * h_mat = (double*)malloc(n*mm*sizeof(double)); // allocate on host
    // Copy from device to host

    for (int iscen = 0; iscen < n_scen;++iscen){
      int p = 0;
      int last = -1;
      int n_segs = 0;
      cudaMemcpy(&h_pred, pred+iscen * (n*m), 1 * m * n * sizeof(int), cudaMemcpyDeviceToHost);
    // int iscen = 0;
      for (int j = 0; j < m; ++j){
        std::cout<<h_pred[ 0 * m + j]<<" . ";
        if (h_pred[ 0 * m + j] != last){
          last = h_pred[ 0 * m + j];
          n_segs +=1;
        }
        p+=1;
      }
      std::cout<<"    size: "<<p<<"    n vehi: "<<n_segs<<"|\n";

    }
  }

  void generate_split();
  void reconstruct_from_pred(Individual & indiv);

  void reset_batch(int batchSize);
  void preprocess_batch(std::vector<Individual*>& indivs, int nbMaxVehicles);
  void generate_split_batch(int batchSize);
  void reconstruct_from_pred_batch(std::vector<Individual*>& indivs);
  int getMaxBatchSize() const { return maxBatchSize; }
  std::size_t getFreeBeforeBytes() const { return freeBeforeBytes; }
  std::size_t getFreeAfterBytes() const { return freeAfterBytes; }
  std::size_t getAllocatedDeviceBytes() const { return allocatedDeviceBytes; }
  std::size_t getRequestedDeviceBytes() const { return requestedDeviceBytes; }

  void evaluateOnGPU_batch(std::vector<Individual*>& indivs);
  // Exact solvers only need one stochastic mean cost per giant tour.  This
  // path performs the scenario reduction on the device and transfers O(batch)
  // values instead of O(batch * scenarios) per-scenario diagnostics.
  std::vector<double> evaluateMeanCostsOnGPU_batch(
      std::vector<Individual*>& indivs);
  void deriveSuccessorsPredecessors_batch(std::vector<Individual*>& indivs);

  int getOriginalM() const { return params.nbClients + 1; }
  void setActiveClients(int nCli) { m = nCli + 1; }

  // Constructor
//   SplitCUDA(const Params & params): params(params)
//     {
//         // Structures of the linear Split
//         // cliSplit = std::vector <ClientSplitCUDA>(params.nbClients + 1);
//         // sumDistance = std::vector <double>(params.nbClients + 1,0.);
//         // sumLoad = std::vector <double>(params.nbClients + 1,0.);
//         // sumService = std::vector <double>(params.nbClients + 1, 0.);
//         // potential = std::vector < std::vector <double> >(params.nbVehicles + 1, std::vector <double>(params.nbClients + 1,1.e30));
//         // pred = std::vector < std::vector <int> >(params.nbVehicles + 1, std::vector <int>(params.nbClients + 1,0));
//     }
    SplitCUDA(const Params & params, int batchSz = 1): params(params)
    {
      if (batchSz < 1)
        throw std::invalid_argument("SplitCUDA batch capacity must be positive");
      n = params.nbVehicles + 1;
      m = params.nbClients + 1;
      n_scen = params.n_scenarios;
      maxBatchSize = batchSz;
      size_t B = (size_t)maxBatchSize;
      size_t nst = B * static_cast<size_t>(n_scen);
      const size_t nodeCount = static_cast<size_t>(m);
      const size_t scenarioNodeEntries = nst * nodeCount;
      const size_t clientCount = nodeCount - 1;
      requestedDeviceBytes =
          scenarioNodeEntries * (3 * sizeof(double) + 2 * sizeof(int)) +
          B * nodeCount * 3 * sizeof(double) +
          nodeCount * 3 * sizeof(double) +
          static_cast<size_t>(n_scen) * sizeof(double) +
          nodeCount * static_cast<size_t>(n_scen) * sizeof(double) +
          nodeCount * nodeCount * sizeof(double) +
          B * clientCount * sizeof(int) +
          nst * 4 * sizeof(double) +
          B * sizeof(double);

      // Initialize the CUDA runtime before taking the baseline.  All device
      // buffers owned by SplitCUDA are persistent and allocated below, so this
      // delta is a deterministic allocation footprint rather than a sampled
      // process-wide peak.
      cudaCheck(cudaFree(0), "cudaFree(0)");
      size_t totalBeforeBytes = 0;
      cudaCheck(cudaMemGetInfo(&freeBeforeBytes, &totalBeforeBytes),
                "cudaMemGetInfo before SplitCUDA allocation");
      cudaCheck(cudaMalloc(&potential, nst * m * sizeof(double)), "cudaMalloc potential");
      cudaCheck(cudaMalloc(&pred, nst * m * sizeof(int)), "cudaMalloc pred");
      allocateHostBuffer(pred_host, nst * m, predHostPinned, "pred");
      allocateHostBuffer(demand_host, (size_t)m * nst, demandHostPinned, "demand");
      std::memset(demand_host, 0, (size_t)m * nst * sizeof(double));
      allocateHostBuffer(host_d0_x, B * m, d0HostPinned, "d0_x");
      allocateHostBuffer(host_dx_0, B * m, dxHostPinned, "dx_0");
      allocateHostBuffer(
          host_sumDistance, B * m, sumDistanceHostPinned, "sumDistance");
      cudaCheck(cudaMalloc(&sumLoad, nst * m * sizeof(double)), "cudaMalloc sumLoad");
      cudaCheck(cudaMalloc(&sumDistance, B * m * sizeof(double)), "cudaMalloc sumDistance");
      cudaCheck(cudaMalloc(&sumService, m * sizeof(double)), "cudaMalloc sumService");
      cudaCheck(cudaMalloc(&cliSplit_demand, nst * m * sizeof(double)), "cudaMalloc Split demand");
      cudaCheck(cudaMalloc(&cliSplit_serviceTime, m * sizeof(double)), "cudaMalloc service time");
      cudaCheck(cudaMalloc(&cliSplit_d0_x, B * m * sizeof(double)), "cudaMalloc d0_x");
      cudaCheck(cudaMalloc(&cliSplit_dx_0, B * m * sizeof(double)), "cudaMalloc dx_0");
      cudaCheck(cudaMalloc(&cliSplit_dnext, m * sizeof(double)), "cudaMalloc dnext");
      cudaCheck(cudaMalloc(&totalDemands, n_scen * sizeof(double)), "cudaMalloc total demands");
      cudaCheck(cudaMemcpy(totalDemands, params.totalDemands,
                           n_scen * sizeof(double), cudaMemcpyHostToDevice),
                "copy total demands");
      maxVehicles_host = std::vector<int>(nst);
      cudaCheck(cudaStreamCreate(&stream), "cudaStreamCreate");
      cudaCheck(cudaMalloc(&myDeque, nst * m * sizeof(int)), "cudaMalloc deque");

      // GPU eval buffers
      m_full = m;
      cudaCheck(cudaMalloc(&d_timeCost, (size_t)m * m * sizeof(double)), "cudaMalloc timeCost");
      // Flatten and upload timeCost
      std::vector<double> flat_tc((size_t)m * m);
      for (int i = 0; i < m; i++)
        for (int j = 0; j < m; j++)
          flat_tc[i * m + j] = params.timeCost[i][j];
      cudaCheck(cudaMemcpy(d_timeCost, flat_tc.data(), m * m * sizeof(double),
                           cudaMemcpyHostToDevice), "copy timeCost");

      cudaCheck(cudaMalloc(
          &d_allClientDemands,
          nodeCount * static_cast<size_t>(n_scen) * sizeof(double)),
          "cudaMalloc all client demands");
      std::vector<double> flatDemands(
          nodeCount * static_cast<size_t>(n_scen), 0.0);
      for (int client = 1; client < m; ++client) {
        const std::vector<double>& source =
            params.cli[client].demands_scenarios;
        if (source.size() != static_cast<std::size_t>(n_scen))
          throw std::runtime_error("SplitCUDA scenario demand size mismatch");
        std::memcpy(
            flatDemands.data() + static_cast<std::size_t>(client) * n_scen,
            source.data(), static_cast<std::size_t>(n_scen) * sizeof(double));
      }
      cudaCheck(cudaMemcpy(
          d_allClientDemands, flatDemands.data(),
          flatDemands.size() * sizeof(double), cudaMemcpyHostToDevice),
          "copy all client demands");

      int nbCli = m - 1;
      cudaCheck(cudaMalloc(&d_chromT, B * nbCli * sizeof(int)), "cudaMalloc chromT");
      allocateHostBuffer(h_chromT, B * nbCli, chromTHostPinned, "chromT");
      cudaCheck(cudaMalloc(&d_evalResults, nst * 4 * sizeof(double)), "cudaMalloc eval results");
      cudaCheck(cudaMalloc(&d_meanCostSums, B * sizeof(double)), "cudaMalloc mean cost sums");
      allocateHostBuffer(
          h_evalResults, nst * 4, evalResultsHostPinned, "evalResults");
      size_t totalAfterBytes = 0;
      cudaCheck(cudaMemGetInfo(&freeAfterBytes, &totalAfterBytes),
                "cudaMemGetInfo after SplitCUDA allocation");
      allocatedDeviceBytes = freeBeforeBytes >= freeAfterBytes
          ? freeBeforeBytes - freeAfterBytes
          : 0;
    }

    ~SplitCUDA() {
      cudaFree(potential);
      cudaFree(pred);
      cudaFree(sumLoad);
      cudaFree(sumDistance);
      cudaFree(sumService);
      cudaFree(cliSplit_demand);
      cudaFree(cliSplit_serviceTime);
      cudaFree(cliSplit_d0_x);
      cudaFree(cliSplit_dx_0);
      cudaFree(cliSplit_dnext);
      cudaFree(totalDemands);
      cudaFree(myDeque);
      cudaFree(d_timeCost);
      cudaFree(d_allClientDemands);
      cudaFree(d_chromT);
      cudaFree(d_evalResults);
      cudaFree(d_meanCostSums);
      freeHostBuffer(pred_host, predHostPinned);
      freeHostBuffer(demand_host, demandHostPinned);
      freeHostBuffer(host_d0_x, d0HostPinned);
      freeHostBuffer(host_dx_0, dxHostPinned);
      freeHostBuffer(host_sumDistance, sumDistanceHostPinned);
      freeHostBuffer(h_chromT, chromTHostPinned);
      freeHostBuffer(h_evalResults, evalResultsHostPinned);
      if (stream != nullptr)
        cudaStreamDestroy(stream);
    }

    void reset();

};
#endif
