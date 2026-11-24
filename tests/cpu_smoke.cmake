file(REMOVE "${OUTPUT}")
execute_process(
    COMMAND "${IRP}" "${INSTANCE}" -type 38 -veh 1 -stock 100 -seed 989 -demandseed 989
            -scenario "${SCENARIOS}" -iter 1 -threads 1 -sol "${OUTPUT}"
    RESULT_VARIABLE status
    OUTPUT_VARIABLE stdout
    ERROR_VARIABLE stderr
)
if(NOT status STREQUAL "0" OR NOT EXISTS "${OUTPUT}")
    message(FATAL_ERROR "CPU run failed (${status})\n${stdout}\n${stderr}")
endif()

file(STRINGS "${OUTPUT}" summary LIMIT_COUNT 1)
string(REGEX MATCHALL "writing the best solution: fitness" solutions "${stdout}")
list(LENGTH solutions count)
if(NOT summary MATCHES "^Total Cost: [0-9]" OR NOT count EQUAL SCENARIOS)
    message(FATAL_ERROR "Expected a finite cost and ${SCENARIOS} scenario solutions\n${stdout}")
endif()
