cmake_minimum_required(VERSION 3.16)

project(MySimulation LANGUAGES CXX)

add_executable(simulator
    src/main.cpp
    src/vehicle.cpp
    src/simulation.cpp
    src/csv_handling.cpp
    src/pure_pursuit_controller.cpp
)

target_include_directories(simulator PRIVATE include)