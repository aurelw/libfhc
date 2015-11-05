
set (core_files simpleraycaster.cpp fasthullcollision.cpp) 

add_library(core ${core_files})
target_link_libraries(core ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
