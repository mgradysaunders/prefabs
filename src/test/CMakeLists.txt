# Add executables.
add_executable(aabbtree aabbtree.cpp)
add_executable(byte_order byte_order.cpp)
add_executable(float_atomic float_atomic.cpp)
add_executable(float_interval float_interval.cpp)
add_executable(half half.cpp)
add_executable(microsurface microsurface.cpp)
add_executable(quat quat.cpp)
add_executable(random random.cpp)
add_executable(running_stat running_stat.cpp)

# Set runtime output directory for all.
set_target_properties(
    aabbtree
    byte_order
    float_atomic
    float_interval
    half
    microsurface
    quat
    random
    running_stat
    PROPERTIES 
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/test"
    )

# Set C++14.
set_target_properties(
    byte_order
    float_atomic
    half
    random
    running_stat
    PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED True
    )

# Set C++17.
set_target_properties(
    aabbtree
    float_interval
    microsurface
    quat
    PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED True
    )

# Link threads.
target_link_libraries(
    aabbtree "${CMAKE_THREAD_LIBS_INIT}")
target_link_libraries(
    float_atomic "${CMAKE_THREAD_LIBS_INIT}")

