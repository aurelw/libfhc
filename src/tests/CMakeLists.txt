include_directories(../core)

# specify output directories
make_directory ("${CMAKE_BINARY_DIR}/bin/")
set (EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin") 

# raycasting test
set(test_raycasting_files test_raycasting.cpp utils.cpp)
add_executable(test_raycasting ${test_raycasting_files})
target_link_libraries(test_raycasting core)

# cropping test
set(test_cropping_files test_cropping.cpp utils.cpp)
add_executable(test_cropping ${test_cropping_files})
target_link_libraries(test_cropping core)

