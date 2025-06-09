cmake_minimum_required(VERSION 3.15)
project(E-CVRP)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "CMake build type: ${CMAKE_BUILD_TYPE}")

# Compiler flags based on build type
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

# Find OpenMP
find_package(OpenMP REQUIRED)

# Source dependencies (optional extension point)
set(DEPENDENCIES
        include/case.hpp
        src/case.cpp
        include/command_line.hpp
        src/command_line.cpp
        include/parameters.hpp
        include/preprocessor.hpp
        src/preprocessor.cpp
        include/heuristic_interface.hpp
        include/stats_interface.hpp
        src/stats_interface.cpp
        include/initializer.hpp
        src/initializer.cpp
        include/individual.hpp
        src/individual.cpp
        include/lahc.hpp
        src/lahc.cpp
        include/follower.hpp
        src/follower.cpp
        include/leader_lahc.hpp
        src/leader_lahc.cpp)

# Main executable
add_executable(Run main.cpp ${DEPENDENCIES})
target_include_directories(Run PRIVATE include external/include)

# Link OpenMP
if(OpenMP_CXX_FOUND)
    target_link_libraries(Run PRIVATE OpenMP::OpenMP_CXX)
endif()

# Debug-only: Add tests with GoogleTest
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_subdirectory(external/googletest)
    enable_testing()

    add_executable(Tests tests/test_main.cpp ${DEPENDENCIES}
            tests/case_test.cpp
            tests/command_line_test.cpp)
    target_include_directories(Tests PRIVATE include external/include)
    target_link_libraries(Tests PRIVATE gtest gtest_main OpenMP::OpenMP_CXX)

    include(GoogleTest)
    gtest_discover_tests(Tests)
endif()
