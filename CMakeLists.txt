cmake_minimum_required(VERSION 3.16)
project(irp LANGUAGES CXX)

option(IRP_ENABLE_GRPC "Enable the gRPC client for the GPU evaluator" OFF)

find_package(Threads REQUIRED)

add_executable(irp
    Client.cpp
    Vehicle.cpp
    commandline.cpp
    Genetic.cpp
    Individu.cpp
    LocalSearch.cpp
    main.cpp
    Noeud.cpp
    Rng.cpp
    Params.cpp
    ParamsList.cpp
    Population.cpp
    Route.cpp
    Mutations.cpp
    LinearPiece.cpp
    PLFunction.cpp
    LotSizingSolver.cpp
    OUPolicySolver.cpp
    ThreadPool.cpp
    ScenarioUtils.cpp
)
target_compile_features(irp PRIVATE cxx_std_17)
target_compile_definitions(irp PRIVATE IL_STD)
target_compile_options(irp PRIVATE
    "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-fno-strict-aliasing>"
)
target_link_libraries(irp PRIVATE Threads::Threads)

if(IRP_ENABLE_GRPC)
    find_package(Protobuf REQUIRED)
    find_package(gRPC CONFIG REQUIRED)

    set(proto_dir "${CMAKE_CURRENT_SOURCE_DIR}/rpc")
    set(proto_file "${proto_dir}/irp.proto")
    set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/rpc")
    set(generated_files
        "${generated_dir}/irp.pb.cc"
        "${generated_dir}/irp.pb.h"
        "${generated_dir}/irp.grpc.pb.cc"
        "${generated_dir}/irp.grpc.pb.h"
    )
    add_custom_command(
        OUTPUT ${generated_files}
        COMMAND ${CMAKE_COMMAND} -E make_directory "${generated_dir}"
        COMMAND protobuf::protoc
        ARGS "--proto_path=${proto_dir}"
             "--cpp_out=${generated_dir}"
             "--grpc_out=${generated_dir}"
             "--plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>"
             "${proto_file}"
        DEPENDS "${proto_file}" protobuf::protoc gRPC::grpc_cpp_plugin
        VERBATIM
    )
    target_sources(irp PRIVATE ${generated_files})
    target_include_directories(irp PRIVATE "${generated_dir}")
    target_compile_definitions(irp PRIVATE IRP_ENABLE_GRPC)
    target_link_libraries(irp PRIVATE gRPC::grpc++ protobuf::libprotobuf)
endif()

include(CTest)
if(BUILD_TESTING)
    foreach(scenarios IN ITEMS 1 2)
        add_test(NAME dsirp_cpu_${scenarios}_scenario
            COMMAND "${CMAKE_COMMAND}"
                "-DIRP=$<TARGET_FILE:irp>"
                "-DINSTANCE=${CMAKE_CURRENT_SOURCE_DIR}/Data/Small/Istanze0105h3/0.dat"
                "-DSCENARIOS=${scenarios}"
                "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/results/smoke-${scenarios}.sol"
                -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/cpu_smoke.cmake"
        )
        set_tests_properties(dsirp_cpu_${scenarios}_scenario PROPERTIES TIMEOUT 90)
    endforeach()
endif()
