cmake_minimum_required(VERSION 3.5)
project(cbrp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIL_STD -std=c++17 -g")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(/workspaces/gurobi1103/armlinux64/include /workspaces/boost_1_87_0 /workspaces/lemon-1.3.1)

link_directories(/workspaces/gurobi1103/armlinux64/lib /workspaces/boost_1_87_0 /workspaces/lemon-1.3.1/lemon)

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
    src/common/BoostLibrary.hpp
    src/common/BoostLibrary.cpp
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
    src/heuristic/Lagrangean.hpp
    src/heuristic/Lagrangean.cpp
    src/heuristic/LocalSearch.hpp
    src/heuristic/LocalSearch.cpp
)

add_executable(cbrp ${sources})

target_link_libraries(cbrp gurobi_c++ gurobi110 pthread m emon)

