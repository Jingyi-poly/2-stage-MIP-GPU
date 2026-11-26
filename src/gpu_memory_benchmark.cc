// Persistent CUDA-memory benchmark for the stochastic PDPTW evaluator.
//
// This source is intentionally not wired into CMake so experiment changes can
// be reviewed independently.  With USE_CUDA enabled, the intended target is:
//
//   add_executable(gpu_memory_benchmark gpu_memory_benchmark.cc)
//   target_link_libraries(gpu_memory_benchmark PRIVATE amdahl_core CUDA::cudart)
//
// Example smoke test:
//
//   ./gpu_memory_benchmark --mode lean --scenarios 1000 --parallelism 1,2
//       --node-num 21 --routes 4
//
// Each grid cell runs in a fresh child process.  This both gives every cell a
// clean CUDA allocation baseline and prevents a cudaMalloc failure (the CUDA
// evaluator currently aborts on allocation errors) from truncating the rest of
// the CSV matrix.

#include <cuda_runtime_api.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <new>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "domain/eval/ScenarioCapacityEvaluator.h"

namespace {

constexpr std::uint64_t kPreambleMagic = 0x414d444d454d5052ULL;  // "AMDMEMPR"
constexpr std::uint64_t kResultMagic = 0x414d444d454d5253ULL;    // "AMDMEMRS"
constexpr std::uint64_t kAllocationHeadroomBytes = 8ULL * 1024ULL * 1024ULL;

enum class BenchmarkMode { kFull, kLean };

struct Options {
    std::vector<int> scenarios{1000, 10000, 100000, 1000000};
    std::vector<int> parallelism{1, 2, 3, 4};
    int nodeNum = 201;  // depot + 200 pickup/delivery nodes
    int routeNum = 20;
    int deviceId = 0;
    double vehicleCapacity = 200.0;
    BenchmarkMode mode = BenchmarkMode::kFull;
};

struct ChildPreamble {
    std::uint64_t magic = kPreambleMagic;
    std::uint64_t freeBeforeBytes = 0;
    std::uint64_t deviceTotalBytes = 0;
    int cudaReady = 0;
    char deviceName[128]{};
};

struct ChildResult {
    std::uint64_t magic = kResultMagic;
    std::uint64_t freeAfterBytes = 0;
    std::uint64_t persistentBytes = 0;
    int oom = 0;
    char backend[16]{};
    char status[32]{};
};

static_assert(std::is_trivially_copyable<ChildPreamble>::value,
              "pipe payload must be trivially copyable");
static_assert(std::is_trivially_copyable<ChildResult>::value,
              "pipe payload must be trivially copyable");

struct CellObservation {
    std::uint64_t persistentBytes = 0;
    std::uint64_t theoreticalBytes = 0;
    std::uint64_t freeBeforeBytes = 0;
    std::uint64_t freeAfterBytes = 0;
    std::uint64_t deviceTotalBytes = 0;
    std::string backend = "unknown";
    std::string status = "child_failed";
    std::string deviceName = "unknown";
    bool oom = false;
};

void CopyText(char* destination, std::size_t capacity, const std::string& text) {
    if (capacity == 0) return;
    std::snprintf(destination, capacity, "%s", text.c_str());
}

std::uint64_t SaturatingAdd(std::uint64_t left, std::uint64_t right) {
    const std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max();
    return left > maximum - right ? maximum : left + right;
}

std::uint64_t SaturatingMultiply(std::uint64_t left, std::uint64_t right) {
    const std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max();
    if (left == 0 || right == 0) return 0;
    return left > maximum / right ? maximum : left * right;
}

const char* ModeName(BenchmarkMode mode) {
    return mode == BenchmarkMode::kFull ? "full" : "lean";
}

// Bytes explicitly requested by CudaScenarioCapacityEvaluator for the selected
// path. CUDA stream/runtime metadata and allocator rounding are deliberately
// excluded; their effect appears in measured bytes.
std::uint64_t TheoreticalPersistentBytes(int nodeNum, int totalNodes,
                                         int routeNum, int scenarios,
                                         int parallelism, BenchmarkMode mode) {
    const std::uint64_t demand = SaturatingMultiply(
        SaturatingMultiply(nodeNum, scenarios), sizeof(float));
    std::uint64_t workspace =
        SaturatingMultiply(totalNodes, sizeof(int));
    workspace = SaturatingAdd(
        workspace, SaturatingMultiply(routeNum + 1, sizeof(int)));
    if (mode == BenchmarkMode::kFull) {
        workspace = SaturatingAdd(
            workspace,
            SaturatingMultiply(SaturatingMultiply(totalNodes, scenarios),
                               sizeof(double)));
        workspace = SaturatingAdd(
            workspace, SaturatingMultiply(totalNodes, sizeof(double)));
    } else {
        workspace = SaturatingAdd(
            workspace, SaturatingMultiply(routeNum, sizeof(double)));
    }
    return SaturatingAdd(
        demand, SaturatingMultiply(parallelism, workspace));
}

std::vector<int> ParsePositiveList(const std::string& text,
                                   const char* optionName) {
    std::vector<int> values;
    std::istringstream stream(text);
    std::string token;
    while (std::getline(stream, token, ',')) {
        if (token.empty()) {
            throw std::invalid_argument(std::string(optionName) +
                                        " contains an empty value");
        }
        std::size_t consumed = 0;
        const long parsed = std::stol(token, &consumed);
        if (consumed != token.size() || parsed <= 0 ||
            parsed > std::numeric_limits<int>::max()) {
            throw std::invalid_argument(std::string(optionName) +
                                        " requires positive integers");
        }
        values.push_back(static_cast<int>(parsed));
    }
    if (values.empty()) {
        throw std::invalid_argument(std::string(optionName) + " is empty");
    }
    return values;
}

int ParsePositiveInt(const std::string& text, const char* optionName) {
    const std::vector<int> values = ParsePositiveList(text, optionName);
    if (values.size() != 1) {
        throw std::invalid_argument(std::string(optionName) +
                                    " requires one positive integer");
    }
    return values.front();
}

Options ParseOptions(int argc, char* argv[]) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--help" || argument == "-h") {
            std::cout
                << "Usage: gpu_memory_benchmark [options]\n"
                << "  --scenarios N[,N...]       default 1000,10000,100000,1000000\n"
                << "  --parallelism P[,P...]     default 1,2,3,4\n"
                << "  --node-num N               default 201 (includes depot)\n"
                << "  --routes R                 default 20\n"
                << "  --device D                 default 0\n"
                << "  --mode full|lean           default full\n"
                << "  --vehicle-capacity C       default 200\n";
            std::exit(0);
        }
        if (i + 1 >= argc) {
            throw std::invalid_argument("missing value after " + argument);
        }
        const std::string value = argv[++i];
        if (argument == "--scenarios") {
            options.scenarios = ParsePositiveList(value, "--scenarios");
        } else if (argument == "--parallelism") {
            options.parallelism = ParsePositiveList(value, "--parallelism");
        } else if (argument == "--node-num") {
            options.nodeNum = ParsePositiveInt(value, "--node-num");
        } else if (argument == "--routes") {
            options.routeNum = ParsePositiveInt(value, "--routes");
        } else if (argument == "--device") {
            std::size_t consumed = 0;
            const long parsed = std::stol(value, &consumed);
            if (consumed != value.size() || parsed < 0 ||
                parsed > std::numeric_limits<int>::max()) {
                throw std::invalid_argument("--device requires a nonnegative integer");
            }
            options.deviceId = static_cast<int>(parsed);
        } else if (argument == "--mode") {
            if (value == "full") {
                options.mode = BenchmarkMode::kFull;
            } else if (value == "lean") {
                options.mode = BenchmarkMode::kLean;
            } else {
                throw std::invalid_argument("--mode must be full or lean");
            }
        } else if (argument == "--vehicle-capacity") {
            std::size_t consumed = 0;
            options.vehicleCapacity = std::stod(value, &consumed);
            if (consumed != value.size() || !(options.vehicleCapacity > 0.0)) {
                throw std::invalid_argument("--vehicle-capacity must be positive");
            }
        } else {
            throw std::invalid_argument("unknown option: " + argument);
        }
    }
    if (options.nodeNum < 2) {
        throw std::invalid_argument("--node-num must include a depot and at least one customer");
    }
    options.routeNum = std::min(options.routeNum, options.nodeNum - 1);
    return options;
}

RouteBatch BuildRouteBatch(int nodeNum, int routeNum) {
    const int totalNodes = nodeNum - 1;
    RouteBatch batch;
    batch.nodeIds.reserve(totalNodes);
    batch.routeOffsets.reserve(routeNum + 1);
    batch.routeOffsets.push_back(0);

    int cursor = 1;
    for (int route = 0; route < routeNum; ++route) {
        const int remainingNodes = nodeNum - cursor;
        const int remainingRoutes = routeNum - route;
        const int count = (remainingNodes + remainingRoutes - 1) / remainingRoutes;
        for (int j = 0; j < count; ++j) batch.nodeIds.push_back(cursor++);
        batch.routeOffsets.push_back(static_cast<int>(batch.nodeIds.size()));
    }
    return batch;
}

std::unique_ptr<float[]> BuildDemandMatrix(int nodeNum, int scenarios) {
    const std::size_t nodes = static_cast<std::size_t>(nodeNum);
    const std::size_t scenarioCount = static_cast<std::size_t>(scenarios);
    const std::size_t maximumElements =
        std::numeric_limits<std::size_t>::max() / sizeof(float);
    if (nodes > maximumElements / scenarioCount) {
        throw std::bad_array_new_length();
    }
    const std::size_t elements = nodes * scenarioCount;
    auto demand = std::make_unique<float[]>(elements);
    std::fill_n(demand.get(), scenarios, 0.0f);  // depot
    for (int node = 1; node < nodeNum; ++node) {
        const int request = (node + 1) / 2;
        const float magnitude = static_cast<float>(1 + request % 7);
        const float value = (node % 2 == 1) ? magnitude : -magnitude;
        std::fill_n(demand.get() + static_cast<std::size_t>(node) * scenarios,
                    scenarios, value);
    }
    return demand;
}

bool WriteAll(int descriptor, const void* data, std::size_t bytes) {
    const char* cursor = static_cast<const char*>(data);
    while (bytes > 0) {
        const ssize_t written = ::write(descriptor, cursor, bytes);
        if (written < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        cursor += written;
        bytes -= static_cast<std::size_t>(written);
    }
    return true;
}

bool ReadAll(int descriptor, void* data, std::size_t bytes) {
    char* cursor = static_cast<char*>(data);
    std::size_t received = 0;
    while (received < bytes) {
        const ssize_t count = ::read(descriptor, cursor + received, bytes - received);
        if (count == 0) return false;
        if (count < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        received += static_cast<std::size_t>(count);
    }
    return true;
}

[[noreturn]] void FinishChild(int descriptor, const ChildResult& result,
                              int exitCode = 0) {
    WriteAll(descriptor, &result, sizeof(result));
    ::close(descriptor);
    _exit(exitCode);
}

[[noreturn]] void RunCellChild(int descriptor, const Options& options,
                               int scenarios, int parallelism,
                               std::uint64_t theoreticalBytes) {
    ChildPreamble preamble;
    ChildResult result;

    cudaError_t cudaStatus = cudaSetDevice(options.deviceId);
    if (cudaStatus == cudaSuccess) cudaStatus = cudaFree(nullptr);  // create context
    cudaDeviceProp properties{};
    if (cudaStatus == cudaSuccess) {
        cudaStatus = cudaGetDeviceProperties(&properties, options.deviceId);
    }
    std::size_t freeBefore = 0;
    std::size_t deviceTotal = 0;
    if (cudaStatus == cudaSuccess) {
        cudaStatus = cudaMemGetInfo(&freeBefore, &deviceTotal);
    }

    if (cudaStatus != cudaSuccess) {
        CopyText(preamble.deviceName, sizeof(preamble.deviceName), "unavailable");
        WriteAll(descriptor, &preamble, sizeof(preamble));
        CopyText(result.backend, sizeof(result.backend), "none");
        CopyText(result.status, sizeof(result.status), "cuda_unavailable");
        FinishChild(descriptor, result);
    }

    preamble.cudaReady = 1;
    preamble.freeBeforeBytes = static_cast<std::uint64_t>(freeBefore);
    preamble.deviceTotalBytes = static_cast<std::uint64_t>(deviceTotal);
    CopyText(preamble.deviceName, sizeof(preamble.deviceName), properties.name);
    if (!WriteAll(descriptor, &preamble, sizeof(preamble))) _exit(3);

    // Leave a small allowance for stream metadata and allocator granularity,
    // neither of which is represented in theoreticalBytes.
    const std::uint64_t headroom =
        std::min(kAllocationHeadroomBytes, theoreticalBytes);
    if (theoreticalBytes == std::numeric_limits<std::uint64_t>::max() ||
        theoreticalBytes > preamble.freeBeforeBytes ||
        headroom > preamble.freeBeforeBytes - theoreticalBytes) {
        result.oom = 1;
        result.freeAfterBytes = preamble.freeBeforeBytes;
        CopyText(result.backend, sizeof(result.backend), "cuda");
        CopyText(result.status, sizeof(result.status), "oom_preflight");
        FinishChild(descriptor, result);
    }

    try {
        std::unique_ptr<float[]> demand = BuildDemandMatrix(options.nodeNum, scenarios);
        RouteBatch batch = BuildRouteBatch(options.nodeNum, options.routeNum);
        std::unique_ptr<ScenarioCapacityEvaluator> evaluator =
            MakeScenarioCapacityEvaluator(demand.get(), options.nodeNum, scenarios,
                                          options.vehicleCapacity,
                                          EvaluatorBackend::kCuda);
        CopyText(result.backend, sizeof(result.backend), evaluator->Backend());
        if (std::string(evaluator->Backend()) != "cuda") {
            CopyText(result.status, sizeof(result.status), "backend_fallback");
            FinishChild(descriptor, result);
        }

        std::atomic<int> ready{0};
        std::atomic<bool> start{false};
        std::atomic<bool> workerError{false};
        std::atomic<bool> workerHostOom{false};
        std::vector<std::thread> workers;
        workers.reserve(parallelism);
        for (int worker = 0; worker < parallelism; ++worker) {
            workers.emplace_back([&]() {
                ready.fetch_add(1, std::memory_order_release);
                while (!start.load(std::memory_order_acquire)) {
                    std::this_thread::yield();
                }
                try {
                    if (options.mode == BenchmarkMode::kFull) {
                        std::vector<double> loads;
                        std::vector<double> nodeAccViolations;
                        evaluator->Evaluate(batch, loads, nodeAccViolations);
                        const std::size_t expectedLoads =
                            static_cast<std::size_t>(batch.TotalNodes()) *
                            scenarios;
                        if (loads.size() != expectedLoads ||
                            nodeAccViolations.size() !=
                                static_cast<std::size_t>(batch.TotalNodes())) {
                            workerError.store(true, std::memory_order_release);
                        }
                    } else {
                        std::vector<double> routeViolations;
                        evaluator->EvaluateRouteViolations(batch,
                                                           routeViolations);
                        if (routeViolations.size() !=
                            static_cast<std::size_t>(batch.RouteNum())) {
                            workerError.store(true, std::memory_order_release);
                        }
                    }
                } catch (const std::bad_alloc&) {
                    workerHostOom.store(true, std::memory_order_release);
                } catch (...) {
                    workerError.store(true, std::memory_order_release);
                }
            });
        }
        while (ready.load(std::memory_order_acquire) != parallelism) {
            std::this_thread::yield();
        }
        start.store(true, std::memory_order_release);
        for (std::thread& worker : workers) worker.join();

        if (workerHostOom.load(std::memory_order_acquire)) {
            result.oom = 1;
            CopyText(result.status, sizeof(result.status), "host_oom");
            FinishChild(descriptor, result, 6);
        }
        if (workerError.load(std::memory_order_acquire)) {
            CopyText(result.status, sizeof(result.status), "worker_error");
            FinishChild(descriptor, result, 4);
        }
        cudaStatus = cudaDeviceSynchronize();
        std::size_t freeAfter = 0;
        std::size_t totalAfter = 0;
        if (cudaStatus == cudaSuccess) {
            cudaStatus = cudaMemGetInfo(&freeAfter, &totalAfter);
        }
        if (cudaStatus != cudaSuccess) {
            CopyText(result.status, sizeof(result.status), "cuda_measure_error");
            FinishChild(descriptor, result, 5);
        }

        result.freeAfterBytes = static_cast<std::uint64_t>(freeAfter);
        result.persistentBytes = preamble.freeBeforeBytes > result.freeAfterBytes
                                     ? preamble.freeBeforeBytes - result.freeAfterBytes
                                     : 0;
        CopyText(result.status, sizeof(result.status), "ok");
        FinishChild(descriptor, result);
    } catch (const std::bad_alloc&) {
        result.oom = 1;
        CopyText(result.backend, sizeof(result.backend), "cuda");
        CopyText(result.status, sizeof(result.status), "host_oom");
        FinishChild(descriptor, result, 6);
    } catch (const std::exception&) {
        CopyText(result.backend, sizeof(result.backend), "cuda");
        CopyText(result.status, sizeof(result.status), "exception");
        FinishChild(descriptor, result, 7);
    } catch (...) {
        CopyText(result.backend, sizeof(result.backend), "cuda");
        CopyText(result.status, sizeof(result.status), "unknown_exception");
        FinishChild(descriptor, result, 8);
    }
}

CellObservation ObserveCell(const Options& options, int scenarios,
                            int parallelism) {
    CellObservation observation;
    const int totalNodes = options.nodeNum - 1;
    observation.theoreticalBytes = TheoreticalPersistentBytes(
        options.nodeNum, totalNodes, options.routeNum, scenarios, parallelism,
        options.mode);

    int descriptors[2];
    if (::pipe(descriptors) != 0) {
        observation.status = "pipe_error";
        return observation;
    }
    std::fflush(nullptr);
    const pid_t child = ::fork();
    if (child < 0) {
        ::close(descriptors[0]);
        ::close(descriptors[1]);
        observation.status = "fork_error";
        return observation;
    }
    if (child == 0) {
        ::close(descriptors[0]);
        RunCellChild(descriptors[1], options, scenarios, parallelism,
                     observation.theoreticalBytes);
    }

    ::close(descriptors[1]);
    ChildPreamble preamble;
    ChildResult result;
    const bool hasPreamble = ReadAll(descriptors[0], &preamble, sizeof(preamble)) &&
                             preamble.magic == kPreambleMagic;
    const bool hasResult = ReadAll(descriptors[0], &result, sizeof(result)) &&
                           result.magic == kResultMagic;
    ::close(descriptors[0]);

    int waitStatus = 0;
    while (::waitpid(child, &waitStatus, 0) < 0 && errno == EINTR) {
    }

    if (hasPreamble) {
        observation.freeBeforeBytes = preamble.freeBeforeBytes;
        observation.deviceTotalBytes = preamble.deviceTotalBytes;
        observation.deviceName = preamble.deviceName;
    }
    if (hasResult) {
        observation.freeAfterBytes = result.freeAfterBytes;
        observation.persistentBytes = result.persistentBytes;
        observation.backend = result.backend;
        observation.status = result.status;
        observation.oom = result.oom != 0;
        return observation;
    }

    observation.backend = hasPreamble && preamble.cudaReady ? "cuda" : "unknown";
    if (WIFSIGNALED(waitStatus)) {
        const int signalNumber = WTERMSIG(waitStatus);
        if (signalNumber == SIGABRT) {
            observation.status = "oom_runtime";
            observation.oom = true;
        } else if (signalNumber == SIGKILL) {
            observation.status = "process_killed";
            observation.oom = true;
        } else {
            observation.status = "signal_" + std::to_string(signalNumber);
        }
    } else if (WIFEXITED(waitStatus)) {
        observation.status = "child_exit_" + std::to_string(WEXITSTATUS(waitStatus));
    }
    return observation;
}

std::string CsvEscape(const std::string& value) {
    if (value.find_first_of(",\"\r\n") == std::string::npos) return value;
    std::string escaped = "\"";
    for (char character : value) {
        if (character == '\"') escaped += '\"';
        escaped += character;
    }
    escaped += '\"';
    return escaped;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const Options options = ParseOptions(argc, argv);
        const int totalNodes = options.nodeNum - 1;
        std::cout
            << "scenario_count,parallelism,mode,node_num,total_nodes,route_count,"
               "device_id,backend,status,oom,persistent_allocated_bytes,"
               "theoretical_bytes,free_before_bytes,free_after_bytes,"
               "device_total_bytes,device_name\n";
        for (int scenarios : options.scenarios) {
            for (int parallelism : options.parallelism) {
                const CellObservation result =
                    ObserveCell(options, scenarios, parallelism);
                std::cout << scenarios << ',' << parallelism << ','
                          << ModeName(options.mode) << ','
                          << options.nodeNum << ',' << totalNodes << ','
                          << options.routeNum << ',' << options.deviceId << ','
                          << result.backend << ',' << result.status << ','
                          << (result.oom ? 1 : 0) << ','
                          << result.persistentBytes << ','
                          << result.theoreticalBytes << ','
                          << result.freeBeforeBytes << ','
                          << result.freeAfterBytes << ','
                          << result.deviceTotalBytes << ','
                          << CsvEscape(result.deviceName) << '\n';
                std::cout.flush();
            }
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "gpu_memory_benchmark: " << error.what() << '\n';
        return 2;
    }
}
