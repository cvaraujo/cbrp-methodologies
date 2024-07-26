cmake_minimum_required(VERSION 3.5)
project(test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIL_STD -std=c++0x ")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(/opt/gurobi1101/linux64/include /opt/boost_1_85_0)

link_directories(/opt/gurobi1101/linux64/lib /opt/boost_1_85_0)

set(
    sources
    test_common.cpp
    ../src/classes/Arc.hpp
    ../src/classes/Graph.hpp
    ../src/classes/Graph.cpp
)

add_executable(test ${sources})

target_link_libraries(test gurobi_c++ gurobi110 pthread m)

