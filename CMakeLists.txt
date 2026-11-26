cmake_minimum_required(VERSION 3.18)
project(HGS_CVRP LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_CUDA "Build the CUDA Split evaluator and solver" OFF)
option(ENABLE_GUROBI "Build the Gurobi baselines" OFF)
option(ENABLE_SCIP "Build the SCIP PCTSP solvers" ON)
option(ENABLE_GPU_TESTS "Run PCTSP tests that require an NVIDIA device" OFF)
find_package(OpenMP REQUIRED COMPONENTS CXX)

if(ENABLE_SCIP)
    find_package(SCIP CONFIG REQUIRED)
    if(TARGET libscip)
        set(SCIP_TARGET libscip)
    elseif(TARGET SCIP::libscip)
        set(SCIP_TARGET SCIP::libscip)
    else()
        message(FATAL_ERROR "SCIP does not export a supported libscip target")
    endif()
endif()

if(MSVC)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

set(split_sources
    Program/AlgorithmParameters.cpp
    Program/C_Interface.cpp
    Program/Genetic.cpp
    Program/GeneticHGS.cpp
    Program/Individual.cpp
    Program/InstanceCVRPLIB.cpp
    Program/LocalSearch.cpp
    Program/Params.cpp
    Program/Population.cpp
    Program/Split.cpp
)

add_library(objlib OBJECT ${split_sources})
set_target_properties(objlib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(objlib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/Program")
target_link_libraries(objlib PUBLIC OpenMP::OpenMP_CXX)

add_library(lib SHARED)
target_link_libraries(lib PUBLIC objlib)

if(ENABLE_CUDA)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    find_package(CUDAToolkit REQUIRED)

    add_library(objlib_cuda OBJECT ${split_sources} Program/SplitCUDA.cu)
    set_target_properties(objlib_cuda PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_include_directories(objlib_cuda PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/Program")
    target_link_libraries(objlib_cuda PUBLIC OpenMP::OpenMP_CXX CUDA::cudart)
    if(CMAKE_CUDA_COMPILER_ID STREQUAL "NVIDIA" AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(objlib_cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-fopenmp>)
    endif()
    add_library(lib_cuda SHARED)
    target_link_libraries(lib_cuda PUBLIC objlib_cuda)
    set_target_properties(lib_cuda PROPERTIES OUTPUT_NAME hgscvrp)
endif()

if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    return()
endif()

add_library(lib_static STATIC)
target_link_libraries(lib_static PUBLIC objlib)
set_target_properties(lib_static PROPERTIES OUTPUT_NAME hgscvrp_static)

add_executable(bin Program/main2.cpp)
target_link_libraries(bin PRIVATE lib_static)
set_target_properties(bin PROPERTIES OUTPUT_NAME hgs)
add_executable(cross_validate Program/cross_validate.cpp)
target_link_libraries(cross_validate PRIVATE lib_static)

if(ENABLE_CUDA)
    add_library(lib_static_cuda STATIC)
    target_link_libraries(lib_static_cuda PUBLIC objlib_cuda)
    set_target_properties(lib_static_cuda PROPERTIES OUTPUT_NAME hgscvrp_static_cuda)
    add_executable(bin_cuda Program/main_cuda.cpp)
    target_link_libraries(bin_cuda PRIVATE lib_static_cuda)
    set_target_properties(bin_cuda PROPERTIES OUTPUT_NAME hgs_cuda)
    install(TARGETS lib_cuda bin_cuda LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
endif()

if(ENABLE_GUROBI)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
    find_package(Gurobi REQUIRED)
    add_executable(pctsp_exact Program/main_pctsp.cpp Program/PCTSPExact.cpp)
    target_link_libraries(pctsp_exact PRIVATE lib_static Gurobi::CXX)
    if(ENABLE_CUDA)
        add_executable(pctsp_exact_cuda Program/main_pctsp_cuda.cpp Program/PCTSPExact.cpp)
        target_link_libraries(pctsp_exact_cuda PRIVATE lib_static_cuda Gurobi::CXX)
    endif()
endif()

if(ENABLE_SCIP)
    set(pctsp_level_sources
        Program/main_pctsp_level_scip.cpp
        Program/PCTSPLevelBBSCIP.cpp
        Program/PCTSPExactSCIP.cpp)
    add_executable(pctsp_level_exact ${pctsp_level_sources})
    target_link_libraries(pctsp_level_exact PRIVATE lib_static ${SCIP_TARGET})
    add_executable(pctsp_scip_outer Program/main_pctsp_scip.cpp Program/PCTSPExactSCIP.cpp)
    target_link_libraries(pctsp_scip_outer PRIVATE lib_static ${SCIP_TARGET})
    install(TARGETS pctsp_level_exact pctsp_scip_outer RUNTIME DESTINATION bin)

    if(ENABLE_CUDA)
        add_executable(pctsp_level_exact_cuda ${pctsp_level_sources})
        target_compile_definitions(pctsp_level_exact_cuda PRIVATE PCTSP_LEVEL_USE_CUDA=1)
        target_link_libraries(pctsp_level_exact_cuda PRIVATE lib_static_cuda ${SCIP_TARGET})
        add_executable(pctsp_scip_outer_cuda Program/main_pctsp_scip_cuda.cpp Program/PCTSPExactSCIP.cpp)
        target_link_libraries(pctsp_scip_outer_cuda PRIVATE lib_static_cuda ${SCIP_TARGET})
        install(TARGETS pctsp_level_exact_cuda pctsp_scip_outer_cuda RUNTIME DESTINATION bin)
    endif()
endif()

include(CTest)
if(BUILD_TESTING)
    function(add_hgs_smoke instance max_cost clients round)
        add_test(NAME "bin_test_${instance}"
            COMMAND "${CMAKE_COMMAND}"
                "-DSOLVER=$<TARGET_FILE:bin>"
                "-DINSTANCE=${CMAKE_CURRENT_SOURCE_DIR}/Instances/CVRP/${instance}.vrp"
                "-DSOLUTION=${CMAKE_CURRENT_BINARY_DIR}/test-results/${instance}.sol"
                "-DMAX_COST=${max_cost}"
                "-DCLIENTS=${clients}"
                "-DROUND=${round}"
                -P "${CMAKE_CURRENT_SOURCE_DIR}/Test/TestExecutable.cmake")
    endfunction()
    add_hgs_smoke(X-n101-k25 29000 100 1)
    add_hgs_smoke(X-n110-k13 16000 109 1)
    add_hgs_smoke(CMT6 590 50 0)
    add_hgs_smoke(CMT7 960 75 0)

    add_subdirectory(Test/Test-c)
    add_test(NAME lib_test_c COMMAND lib_test_c)
    set_tests_properties(lib_test_c PROPERTIES ENVIRONMENT "OMP_NUM_THREADS=1")

    if(ENABLE_SCIP)
        add_executable(test_pctsp_level_bb Test/test_pctsp_level_bb_bruteforce.cpp
            Program/PCTSPLevelBBSCIP.cpp Program/PCTSPExactSCIP.cpp)
        target_link_libraries(test_pctsp_level_bb PRIVATE lib_static ${SCIP_TARGET})
        add_executable(test_pctsp_scip_outer Test/test_pctsp_scip_optional_bruteforce.cpp
            Program/PCTSPExactSCIP.cpp)
        target_link_libraries(test_pctsp_scip_outer PRIVATE lib_static ${SCIP_TARGET})
        set(pctsp_fixture "${CMAKE_CURRENT_SOURCE_DIR}/Test/tiny-n5-k2.vrp")
        add_test(NAME pctsp_level_bb_bruteforce COMMAND test_pctsp_level_bb "${pctsp_fixture}")
        add_test(NAME pctsp_scip_outer_bruteforce COMMAND test_pctsp_scip_outer "${pctsp_fixture}")
        set_tests_properties(pctsp_level_bb_bruteforce pctsp_scip_outer_bruteforce
            PROPERTIES TIMEOUT 120 ENVIRONMENT "OMP_NUM_THREADS=2")

        set(pctsp_smoke_args -scenarioSeed 11 -penaltySeed 1729 -algoSeed 1
            -nextrascen 3 -optionalVisit 1 -skipPenScale 1.2 -timeLim 30 -nthreads 2 -gpuBatchSize 8 -log 0)
        add_test(NAME pctsp_level_exact_tiny COMMAND pctsp_level_exact "${pctsp_fixture}"
            "${CMAKE_CURRENT_BINARY_DIR}/tiny-level-exact.sol" ${pctsp_smoke_args})
        set_tests_properties(pctsp_level_exact_tiny PROPERTIES
            PASS_REGULAR_EXPRESSION "proven optimal:[ ]+yes" TIMEOUT 60)
        if(ENABLE_CUDA AND ENABLE_GPU_TESTS)
            add_test(NAME pctsp_level_exact_cuda_tiny COMMAND pctsp_level_exact_cuda "${pctsp_fixture}"
                "${CMAKE_CURRENT_BINARY_DIR}/tiny-level-exact-cuda.sol" ${pctsp_smoke_args} -deviceId 0)
            set_tests_properties(pctsp_level_exact_cuda_tiny PROPERTIES
                PASS_REGULAR_EXPRESSION "proven optimal:[ ]+yes" TIMEOUT 60)
        endif()
    endif()

    if(ENABLE_CUDA)
        add_executable(test_cuda_split Test/TestSplitCUDA.cpp)
        target_link_libraries(test_cuda_split PRIVATE lib_static_cuda)
        add_test(NAME fixed_tour_cpu_cuda
            COMMAND test_cuda_split
                "${CMAKE_CURRENT_SOURCE_DIR}/Instances/CVRP/X-n106-k14.vrp"
                "${CMAKE_CURRENT_SOURCE_DIR}/Instances/CVRP/X-n106-k14.vrp.cvrp_chromT.txt")
        set_tests_properties(fixed_tour_cpu_cuda PROPERTIES SKIP_RETURN_CODE 77)
    endif()
endif()

install(TARGETS lib bin cross_validate LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(FILES Program/AlgorithmParameters.h Program/C_Interface.h DESTINATION include)
