include_directories(../core)

# specify output directories
make_directory ("${CMAKE_BINARY_DIR}/bin/")
set (EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin") 

# raycasting test
set(test_raycasting_files test_raycasting.cpp)
add_executable(test_raycasting ${test_raycasting_files})
target_link_libraries(test_raycasting core)

