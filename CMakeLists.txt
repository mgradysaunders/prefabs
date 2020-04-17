# CMake version.
cmake_minimum_required(VERSION 3.10)

# Project.
project(preform VERSION 1.0.0 LANGUAGES CXX)

# Set release.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Set compile flags.
set(CMAKE_CXX_FLAGS "-Wall -Wextra -march=native -mtune=native")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Install headers.
install(
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/"
    DESTINATION include
    FILES_MATCHING 
    PATTERN "*.hpp" 
    PATTERN "*.inl"
    )

# TODO 

# Require threads.
find_package(Threads REQUIRED)

# Include directory for all examples and tests.
include_directories(include)

# Recurse.
add_subdirectory(src)
