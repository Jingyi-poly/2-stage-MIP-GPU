cmake_minimum_required(VERSION 3.18)
project(cvrp_exact LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
find_package(OpenMP REQUIRED COMPONENTS CXX)
find_package(Gurobi REQUIRED)
set(HGS_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Program")

add_executable(cvrp_exact cvrp_exact.cpp
    ${HGS_SRC_DIR}/Params.cpp
    ${HGS_SRC_DIR}/Individual.cpp
    ${HGS_SRC_DIR}/Split.cpp
    ${HGS_SRC_DIR}/InstanceCVRPLIB.cpp
    ${HGS_SRC_DIR}/AlgorithmParameters.cpp
    ${HGS_SRC_DIR}/LocalSearch.cpp
    ${HGS_SRC_DIR}/Population.cpp
    ${HGS_SRC_DIR}/Genetic.cpp
    ${HGS_SRC_DIR}/GeneticHGS.cpp
    ${HGS_SRC_DIR}/C_Interface.cpp)
target_include_directories(cvrp_exact PRIVATE "${HGS_SRC_DIR}")
target_link_libraries(cvrp_exact PRIVATE OpenMP::OpenMP_CXX Gurobi::CXX)
