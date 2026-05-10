cmake_minimum_required(VERSION 3.16)

project(MySimulation LANGUAGES CXX)

add_executable(simulator
    src/main.cpp
)

target_include_directories(simulator PRIVATE include)