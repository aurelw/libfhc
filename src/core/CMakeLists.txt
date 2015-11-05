
set (core_files simpleraycaster.cpp fasthullcollision.cpp) 

add_library(core ${core_files})
target_link_libraries(core ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_library(fhc SHARED ${core_files})
target_link_libraries(fhc ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
install(TARGETS fhc DESTINATION lib)
install(FILES "fasthullcollision.hpp" "simpleraycaster.hpp" DESTINATION include/fhc)

