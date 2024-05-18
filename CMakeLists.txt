cmake_minimum_required(VERSION 3.5)
project(cbrp-lr)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIL_STD -std=c++0x ")

include_directories(/opt/boost_1_84_0)

link_directories(/opt/boost_1_84_0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
)

add_executable(cbrp-lr ${sources})

target_link_libraries(cbrp-lr pthread m)
