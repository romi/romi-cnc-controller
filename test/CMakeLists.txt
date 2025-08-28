cmake_minimum_required(VERSION 3.10)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/mock
  ${CMAKE_CURRENT_SOURCE_DIR}/../RomiCNCController)

###

set(SRCS_BLOCKS
  src/tests_main.cpp
  src/Block_tests.cpp)

add_executable(romi_cnc_controller_unit_tests_blocks ${SRCS_BLOCKS})
target_link_libraries(romi_cnc_controller_unit_tests_blocks romi-cnc-controller gtest gmock)
add_test(NAME romi_cnc_controller_unit_tests_blocks
  COMMAND romi_cnc_controller_unit_tests_blocks)

###
#
#Message (${CMAKE_BINARY_DIR})
#
#if(BUILD_COVERAGE)
#    SETUP_TARGET_FOR_COVERAGE_LCOV(
#            NAME libsteering_unit_tests_coverage
#            EXECUTABLE ctest -V ${n_cores} -R "libsteering"
#            #EXCLUDE '*wrapper*' "src/Linux.cpp" "include/*" '${CMAKE_BINARY_DIR}/libr/*'
#            DEPENDENCIES
#                libsteering_unit_tests_movingaverage
#                libsteering_unit_tests_controller
#                libsteering_unit_tests_steppercontroller
#    )
#endif()


