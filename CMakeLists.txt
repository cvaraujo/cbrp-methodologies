cmake_minimum_required(VERSION 3.5)
project(cbrp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIL_STD -std=c++0x")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(/opt/gurobi1101/linux64/include /opt/boost_1_85_0)

link_directories(/opt/gurobi1101/linux64/lib /opt/boost_1_85_0)

set(
    sources
    main.cpp
)

add_executable(cbrp ${sources})

target_link_libraries(cbrp gurobi_c++ gurobi110 pthread m)

