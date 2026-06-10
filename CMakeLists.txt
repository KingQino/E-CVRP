cmake_minimum_required(VERSION 3.15)
project(E-CVRP-Learning)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_UPPER)

message(STATUS "CMake build type: ${CMAKE_BUILD_TYPE}")

# Compiler flags based on build type
if(CMAKE_BUILD_TYPE_UPPER STREQUAL "RELEASE")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
elseif(CMAKE_BUILD_TYPE_UPPER STREQUAL "DEBUG")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

# Find OpenMP
if(APPLE AND CMAKE_C_COMPILER_ID STREQUAL "AppleClang")
    set(OpenMP_ROOT "/opt/homebrew/opt/libomp")
    set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp")
    set(OpenMP_C_LIB_NAMES "omp")
    set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp")
    set(OpenMP_CXX_LIB_NAMES "omp")
    set(OpenMP_omp_LIBRARY "${OpenMP_ROOT}/lib/libomp.dylib")
    set(OpenMP_CXX_INCLUDE_DIR "/opt/homebrew/opt/libomp/include")
    set(OpenMP_C_INCLUDE_DIR  "/opt/homebrew/opt/libomp/include")
endif()

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
        include/two_stage_alns.hpp
        src/two_stage_alns.cpp
        include/follower.hpp
        src/follower.cpp
        include/leader.hpp
        src/leader.cpp
        include/leader_single.hpp
        src/leader_single.cpp
)

# Main executable
add_executable(Run main.cpp ${DEPENDENCIES})
target_include_directories(Run PRIVATE include external/include)
if(APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    target_include_directories(Run PRIVATE /opt/homebrew/opt/libomp/include)
endif()

# Link OpenMP
if(OpenMP_CXX_FOUND)
    target_link_libraries(Run PRIVATE OpenMP::OpenMP_CXX)
endif()

# Debug-only: Add tests with GoogleTest when available
if(CMAKE_BUILD_TYPE_UPPER STREQUAL "DEBUG")
    enable_testing()

    set(TEST_LIBRARIES)
    if(EXISTS "${CMAKE_SOURCE_DIR}/external/googletest/CMakeLists.txt")
        add_subdirectory(external/googletest)
        set(TEST_LIBRARIES gtest gtest_main)
    else()
        find_package(GTest QUIET)
        if(GTest_FOUND)
            set(TEST_LIBRARIES GTest::gtest GTest::gtest_main)
        else()
            message(WARNING "GoogleTest not found; skipping Tests target. Run `git submodule update --init external/googletest` to enable tests.")
        endif()
    endif()

    if(TEST_LIBRARIES)
        add_executable(Tests tests/test_main.cpp ${DEPENDENCIES}
                tests/case_test.cpp
                tests/two_stage_alns_test.cpp
                tests/follower_test.cpp
                tests/command_line_test.cpp)
        target_include_directories(Tests PRIVATE include external/include)
        target_link_libraries(Tests PRIVATE ${TEST_LIBRARIES} OpenMP::OpenMP_CXX)

        include(GoogleTest)
        gtest_discover_tests(Tests)
    endif()
endif()
