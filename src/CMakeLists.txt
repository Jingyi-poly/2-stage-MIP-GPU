cmake_minimum_required(VERSION 3.18)

project(AMDAHL VERSION 0.0 LANGUAGES CXX)
set(AMDAHL_VERSION_PATCH 1)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(USE_GUROBI "Build the exact (Gurobi-based) solver" OFF)
option(USE_CUDA "Build the CUDA scenario-capacity evaluator backend" OFF)
option(USE_SCIP "Build the free SCIP stochastic PDPTW benchmark" OFF)

set(GUROBI_ROOT_DIR "/opt/gurobi1100/linux64" CACHE STRING "Specific the root of Gurobi")
set(CUDA_ARCH "75" CACHE STRING "CUDA architecture (e.g. 75 for RTX 20-series)")

set(sources 
        # aclnn/aclnn.cpp
        analysis/problem_analysis.cc
        analysis/solution_analysis.cc
        domain/eval/CpuScenarioCapacityEvaluator.cpp
        domain/eval/MatrixModelEvaluator.cpp
        domain/model/entity/VrpRoute.cpp
        domain/model/VrpSolution.cpp
        domain/solver/memetic/crossover/EdgeAssemblyCrossover.cpp
        domain/solver/memetic/crossover/LcsDp.cpp
        domain/solver/memetic/crossover/RouteSubset.cpp
        domain/solver/memetic/crossover/SrexCrossover.cpp
        domain/solver/memetic/insert/Insert.cpp
        domain/solver/memetic/removal/Removal.cpp
        domain/solver/memetic/LargeNeighborhoodSearch.cpp
        domain/solver/memetic/LocalSearch.cpp
        domain/solver/memetic/Memetic.cpp
        domain/solver/memetic/Perturb.cpp
        io/pdptw_reader.cc
        io/request_pairing_reader.cc
        io/sol_reader.cc
        io/sol_writer.cc
        io/valid_arcs_reader.cc
        io/vrp_sol_writer.cc
        model/solution.cc
        util/convert_utils.cc
        util/random_utils.cc
        util/sol_similarity.cc
)

if (${USE_GUROBI})
    add_compile_definitions(USE_GUROBI)
    set(sources ${sources} domain/solver/exact/TwoIndexFormulation.cpp)
endif (${USE_GUROBI})

configure_file(${AMDAHL_SOURCE_DIR}/config.h.in ${AMDAHL_BINARY_DIR}/config.h)

add_library(amdahl_core STATIC ${sources})
target_include_directories(amdahl_core PUBLIC ${AMDAHL_SOURCE_DIR} ${AMDAHL_BINARY_DIR})

find_package(OpenMP REQUIRED)
target_link_libraries(amdahl_core PUBLIC OpenMP::OpenMP_CXX)

if (${USE_CUDA})
    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)
    target_sources(amdahl_core PRIVATE domain/eval/CudaScenarioCapacityEvaluator.cu)
    target_compile_definitions(amdahl_core PUBLIC USE_CUDA)
    set_target_properties(amdahl_core PROPERTIES
            CUDA_STANDARD 17
            CUDA_STANDARD_REQUIRED ON
            CUDA_ARCHITECTURES "${CUDA_ARCH}")
    target_link_libraries(amdahl_core PUBLIC CUDA::cudart)
endif (${USE_CUDA})

if (${USE_SCIP})
    find_path(SCIP_INCLUDE_DIR scip/scip.h PATHS /usr/include)
    find_library(SCIP_LIBRARY NAMES scip PATHS /usr/lib)
    if (NOT SCIP_INCLUDE_DIR OR NOT SCIP_LIBRARY)
        message(FATAL_ERROR "USE_SCIP=ON requires SCIP headers and libscip (expected under /usr/include and /usr/lib)")
    endif ()
    add_executable(pdptw_scip_benchmark pdptw_scip_benchmark.cc)
    target_include_directories(pdptw_scip_benchmark PRIVATE ${SCIP_INCLUDE_DIR})
    target_link_libraries(pdptw_scip_benchmark PRIVATE amdahl_core ${SCIP_LIBRARY})
endif (${USE_SCIP})

if (${USE_GUROBI})
    target_include_directories(amdahl_core PUBLIC ${GUROBI_ROOT_DIR}/include/)
    target_link_libraries(amdahl_core PUBLIC ${GUROBI_ROOT_DIR}/src/build/libgurobi_c++.a)
    target_link_libraries(amdahl_core PUBLIC ${GUROBI_ROOT_DIR}/lib/libgurobi110.so)
endif (${USE_GUROBI})

add_executable(amdahl main.cc)
target_link_libraries(amdahl PRIVATE amdahl_core)

add_executable(scenario_eval_benchmark scenario_eval_benchmark.cc)
target_link_libraries(scenario_eval_benchmark PRIVATE amdahl_core)

add_executable(insertion_delta_benchmark insertion_delta_benchmark.cc)
target_link_libraries(insertion_delta_benchmark PRIVATE amdahl_core)

add_executable(reversal_delta_benchmark reversal_delta_benchmark.cc)
target_link_libraries(reversal_delta_benchmark PRIVATE amdahl_core)

add_executable(paper_experiments paper_experiments.cc)
target_link_libraries(paper_experiments PRIVATE amdahl_core)

add_executable(matrix_model_eval_test matrix_model_eval_test.cc)
target_link_libraries(matrix_model_eval_test PRIVATE amdahl_core)

add_executable(matrix_model_benchmark matrix_model_benchmark.cc)
target_link_libraries(matrix_model_benchmark PRIVATE amdahl_core)

if (${USE_CUDA})
    add_executable(gpu_memory_benchmark gpu_memory_benchmark.cc)
    target_link_libraries(gpu_memory_benchmark PRIVATE amdahl_core CUDA::cudart)
endif (${USE_CUDA})

# # cmake ../ -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
# # Ascend / aclnn (NPU) backend - left as a future evaluator backend.
