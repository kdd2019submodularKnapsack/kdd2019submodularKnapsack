cmake_minimum_required(VERSION 2.8.4)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -O2 --std=c++17 -fopenmp -pthread") # -g -pg
project (submodular-knapsack)
include_directories("headers")
include_directories("lib")

#add_subdirectory(lib)
#
#add_library(eigens lib)
#
#target_link_libraries(submodular-knapsack eigens)

#file(GLOB_RECURSE lib_SRC
#        lib/eigen3/*.cpp
#        lib/redsvd/util.cpp
#        )

add_executable(submodular-knapsack
#        ${lib_SRC}

        src/movie_data.cpp
        src/utils.cpp
        src/main.cpp
        headers/inputs/DelegateKnapsack.h headers/algorithms/robust/GeneralRobust.h headers/algorithms/robust/MakeRobust.h headers/inputs/CoverageKnapsack.h headers/testWithRemovals.h headers/inputs/NonrobustKiller.h headers/Dataset.h)