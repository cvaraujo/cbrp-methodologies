cmake_minimum_required(VERSION 3.10)
project(cbrp)

# Configurações do compilador
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -DIL_STD -O3 -ffast-math -funroll-loops -march=native -DNDEBUG")

# Diretórios de includes
include_directories(
    /opt/gurobi1201/linux64/include
    /opt/boost_1_88_0
    /opt/lemon-1.3.1
)

# Diretórios de bibliotecas
link_directories(
    /opt/gurobi1201/linux64/lib
    /opt/boost_1_88_0
    /opt/lemon-1.3.1/lemon
)

# Fontes comuns
set(SOURCES_MAIN
    src/classes/Arc.hpp
    src/classes/Parameters.hpp
    src/classes/Route.hpp
    src/classes/Route.cpp
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
)

# Fontes dos modelos
set(SOURCES_DETERMINISTIC_MODEL
    src/exact/DeterministicModel.hpp
    src/exact/DeterministicModel.cpp
    src/exact/DeterministicModelWalk.hpp
    src/exact/DeterministicModelWalk.cpp
)

set(SOURCES_STOCHASTIC_MODEL
    src/exact/StochasticModel.hpp
    src/exact/StochasticModel.cpp
    src/exact/StochasticModelWalk.hpp
    src/exact/StochasticModelWalk.cpp
    main.cpp
)

# Heurísticas e métodos adicionais
set(SOURCES_DETERMINISTIC_HEURISTIC
    src/heuristic/stochastic/StartSolution.hpp
    src/heuristic/GreedyHeuristic.hpp
    src/heuristic/GreedyHeuristic.cpp
    main.cpp
)

set(SOURCES_DETERMINISTIC_GREEDY_HEURISTIC
    src/heuristic/GreedyHeuristic.hpp
    src/heuristic/GreedyHeuristic.cpp
    main-det-greedy-heuristic.cpp
)

set(SOURCES_DETERMINISTIC_LAGRANGEAN
    src/heuristic/stochastic/StartSolution.hpp
    src/heuristic/GreedyHeuristic.hpp
    src/heuristic/GreedyHeuristic.cpp
    src/exact/DeterministicModelWalkBarrierMethod.hpp
    src/exact/DeterministicModelWalkBarrierMethod.cpp
    src/heuristic/Lagrangean.hpp
    src/heuristic/Lagrangean.cpp
    main-lagrangean.cpp
)

set(SOURCES_STOCHASTIC_HEURISTIC
    src/heuristic/metaheuristics/SimulatedAnnealing.hpp
    src/heuristic/stochastic/Utils.hpp
    src/heuristic/stochastic/LocalSearch.hpp
    src/heuristic/stochastic/LocalSearch.cpp
    src/heuristic/stochastic/StartSolution.hpp
    src/heuristic/GreedyHeuristic.hpp
    src/heuristic/GreedyHeuristic.cpp
)

set(SOURCES_SIMHEURISTIC
    src/common/Postgree.hpp
    src/simheuristic/simheuristic.hpp
    main-simheuristic.cpp
)

# ======================
# Criação dos executáveis
# ======================

# Deterministic Model
add_executable(cbrp-det
    ${SOURCES_MAIN}
    ${SOURCES_DETERMINISTIC_MODEL}
)

# Stochastic Model
add_executable(cbrp-stoc
    ${SOURCES_MAIN}
    ${SOURCES_STOCHASTIC_MODEL}
)

# Deterministic Heuristic
add_executable(cbrp-det-heur
    ${SOURCES_MAIN}
    ${SOURCES_DETERMINISTIC_HEURISTIC}
)

add_executable(cbrp-det-greedy
    ${SOURCES_MAIN}
    ${SOURCES_DETERMINISTIC_GREEDY_HEURISTIC}
    main-det-greedy-heuristic.cpp
)

# Deterministic Lagrangean
add_executable(cbrp-det-lagr
    ${SOURCES_MAIN}
    ${SOURCES_DETERMINISTIC_MODEL}
    ${SOURCES_DETERMINISTIC_LAGRANGEAN}
)

# Stochastic Heuristic
add_executable(cbrp-stoc-heur
    ${SOURCES_MAIN}
    ${SOURCES_STOCHASTIC_HEURISTIC}
)

# Simheuristic
add_executable(cbrp-simheur
    ${SOURCES_MAIN}
    ${SOURCES_STOCHASTIC_HEURISTIC}
    ${SOURCES_SIMHEURISTIC}
)


target_link_libraries(cbrp-det gurobi_c++ gurobi120 pthread m emon pqxx pq zmq)
target_link_libraries(cbrp-det-greedy gurobi_c++ gurobi120 pthread m emon pqxx pq zmq)
target_link_libraries(cbrp-stoc gurobi_c++ gurobi120 pthread m emon pqxx pq zmq)
target_link_libraries(cbrp-det-lagr gurobi_c++ gurobi120 pthread m emon pqxx pq zmq)

target_link_libraries(cbrp-det-heur pthread m)
target_link_libraries(cbrp-stoc-heur pthread m)
target_link_libraries(cbrp-simheur pthread m)