cmake_minimum_required(VERSION 3.5)
project(cbrp-lr)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIL_STD -std=c++0x ")

set(CMAKE_BUILD_TYPE Release) 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(/opt/gurobi1101/linux64/include /opt/boost_1_85_0)

link_directories(/opt/gurobi1101/linux64/lib /opt/boost_1_85_0)

set(
    sources
    main.cpp
    headers/Graph.h
    src/Graph.cpp
    headers/Include.h
    src/Arc.cpp
    headers/Arc.h
    headers/Lagrangean.h
    src/Lagrangean.cpp
    headers/ERCSPP.hpp
    headers/Scenario.h
    src/Scenario.cpp
    headers/StochasticModel.h
    src/StochasticModel.cpp
    headers/DeterministicModel.h
    src/DeterministicModel.cpp
    headers/WarmStart.h
    src/WarmStart.cpp
)

add_executable(cbrp-lr ${sources})

target_link_libraries(cbrp-lr gurobi_c++ gurobi110 pthread m)

