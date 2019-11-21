# Add executables.
add_executable(image2_mipmap image2_mipmap.cpp)
add_executable(image2_resample image2_resample.cpp)
add_executable(microsurface_preview microsurface_preview.cpp)
add_executable(periodic_simplex_noise2 periodic_simplex_noise2.cpp)
add_executable(periodic_worley_noise2 periodic_worley_noise2.cpp)
add_executable(simplex_noise2 simplex_noise2.cpp)
add_executable(simplex_noise3 simplex_noise3.cpp)
add_executable(terminal_progress_bar terminal_progress_bar.cpp)
add_executable(worley_noise2 worley_noise2.cpp)
add_executable(worley_noise3 worley_noise3.cpp)

# Set runtime output directory for all.
set_target_properties(
    image2_mipmap
    image2_resample
    microsurface_preview
    periodic_simplex_noise2
    periodic_worley_noise2
    simplex_noise2
    simplex_noise3
    terminal_progress_bar
    worley_noise2
    worley_noise3
    PROPERTIES 
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/example"
    )

# Set C++11.
set_target_properties(
    terminal_progress_bar
    PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED True
    )

# Set C++17.
set_target_properties(
    image2_mipmap
    image2_resample
    microsurface_preview
    periodic_simplex_noise2
    periodic_worley_noise2
    simplex_noise2
    simplex_noise3
    worley_noise2
    worley_noise3
    PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED True
    )

# Link threads.
# Only uses std::this_thread::sleep_for(), link may be unnecessary?
target_link_libraries(
    terminal_progress_bar ${CMAKE_THREAD_LIBS_INIT})


