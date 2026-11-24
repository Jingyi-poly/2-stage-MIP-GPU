set(GUROBI_HOME "$ENV{GUROBI_HOME}" CACHE PATH "Directory containing Gurobi include/ and lib/")
find_path(Gurobi_INCLUDE_DIR NAMES gurobi_c++.h HINTS "${GUROBI_HOME}" PATH_SUFFIXES include)

if(Gurobi_INCLUDE_DIR AND EXISTS "${Gurobi_INCLUDE_DIR}/gurobi_c.h")
    file(STRINGS "${Gurobi_INCLUDE_DIR}/gurobi_c.h" _gurobi_version_macros
        REGEX "^#[ \t]*define[ \t]+GRB_VERSION_(MAJOR|MINOR)[ \t]+[0-9]+")
    if(_gurobi_version_macros MATCHES "GRB_VERSION_MAJOR[ \t]+([0-9]+)")
        set(_gurobi_major "${CMAKE_MATCH_1}")
    endif()
    if(_gurobi_version_macros MATCHES "GRB_VERSION_MINOR[ \t]+([0-9]+)")
        set(_gurobi_minor "${CMAKE_MATCH_1}")
    endif()
    if(DEFINED _gurobi_major AND DEFINED _gurobi_minor)
        set(_gurobi_core_name "gurobi${_gurobi_major}${_gurobi_minor}")
    endif()
endif()

find_library(Gurobi_CORE_LIBRARY NAMES ${_gurobi_core_name} gurobi
    HINTS "${GUROBI_HOME}" PATH_SUFFIXES lib)
find_library(Gurobi_CXX_LIBRARY NAMES gurobi_c++ gurobi_c++md2017
    HINTS "${GUROBI_HOME}" PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Gurobi
    REQUIRED_VARS Gurobi_INCLUDE_DIR Gurobi_CORE_LIBRARY Gurobi_CXX_LIBRARY
    REASON_FAILURE_MESSAGE "Set GUROBI_HOME to the Gurobi SDK directory containing include/ and lib/.")

if(Gurobi_FOUND AND NOT TARGET Gurobi::CXX)
    add_library(Gurobi::CXX UNKNOWN IMPORTED)
    set_target_properties(Gurobi::CXX PROPERTIES
        IMPORTED_LOCATION "${Gurobi_CXX_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${Gurobi_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${Gurobi_CORE_LIBRARY}")
endif()

mark_as_advanced(Gurobi_INCLUDE_DIR Gurobi_CORE_LIBRARY Gurobi_CXX_LIBRARY)
