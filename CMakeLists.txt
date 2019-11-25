# CMake version.
cmake_minimum_required(VERSION 3.10)

# Project.
project(preform VERSION 0.0.1 LANGUAGES CXX)

# Set release.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Set compile flags.
set(CMAKE_CXX_FLAGS "-Wall -Wextra -march=native -mtune=native")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Require threads.
find_package(Threads REQUIRED)

# Include directory for all examples and tests.
include_directories(include)

# Recurse.
add_subdirectory(src)
