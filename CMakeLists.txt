cmake_minimum_required(VERSION 3.16)

project(MySimulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(agv_sim_lib
    src/vehicle.cpp
    src/simulation.cpp
    src/csv_handling.cpp
    src/pure_pursuit_controller.cpp
    src/path_geometry.cpp
)

target_include_directories(agv_sim_lib PUBLIC include)

add_executable(simulator
    src/main.cpp
)

target_link_libraries(simulator PRIVATE agv_sim_lib)

enable_testing()

find_package(GTest REQUIRED)

add_executable(test_kinematic_bicycle_model
    tests/test_kinematic_bicycle_model.cpp
)

target_link_libraries(test_kinematic_bicycle_model
    PRIVATE
    agv_sim_lib
    GTest::gtest_main
)

include(GoogleTest)

gtest_discover_tests(test_kinematic_bicycle_model)

add_executable(test_pure_pursuit_controller
    tests/test_pure_pursuit_controller.cpp
)

target_link_libraries(test_pure_pursuit_controller
    PRIVATE
    agv_sim_lib
    GTest::gtest_main
)

gtest_discover_tests(test_pure_pursuit_controller)