cmake_minimum_required(VERSION 3.5)
project(cbrp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIL_STD -std=c++17 -DSilence")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(/opt/gurobi1101/linux64/include /opt/boost_1_85_0 /opt/lemon-1.3.1)

link_directories(/opt/gurobi1101/linux64/lib /opt/boost_1_85_0 /opt/lemon-1.3.1/lemon)

set(
    sources 
    main.cpp
    src/classes/Arc.hpp
    src/classes/Parameters.hpp
    src/classes/Graph.hpp
    src/classes/Graph.cpp
    src/classes/Input.hpp
    src/classes/Input.cpp
    src/classes/Solution.hpp
    src/common/ShortestPath.hpp
    src/common/ShortestPath.cpp
    src/common/BlockConnection.hpp
    src/common/BlockConnection.cpp
    src/common/Knapsack.hpp
    src/exact/DeterministicModel.hpp
    src/exact/DeterministicModel.cpp
    src/exact/DeterministicModelWalk.hpp
    src/exact/DeterministicModelWalk.cpp
    src/exact/StochasticModel.hpp
    src/exact/StochasticModel.cpp
    src/exact/StochasticModelWalk.hpp
    src/exact/StochasticModelWalk.cpp
    src/heuristic/GreedyHeuristic.hpp
    src/heuristic/GreedyHeuristic.cpp
    src/heuristic/LocalSearch.hpp
    src/heuristic/LocalSearch.cpp
)

add_executable(cbrp ${sources})

target_link_libraries(cbrp gurobi_c++ gurobi110 pthread m emon)

