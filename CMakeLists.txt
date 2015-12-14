cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(libfhc)

set (CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)

option (WITH_LLVM "Build with clang and llvm." OFF)
option (WITH_C++11 "Be lame and downgrade to C++11 (g++)." OFF)
option (BUILD_CORE "Build core" ON)
option (BUILD_TESTS "Build tests" OFF)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

if (WITH_C++11) 

    add_definitions("-std=c++11")
    add_definitions("-DWITH_CPP11") 

else (WITH_C++11)

    add_definitions("-std=c++14")
    if (WITH_LLVM)
        #FIXME compile with libc++
        #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
        set(CMAKE_CXX_COMPILER "/usr/bin/clang++-3.6")
    else (WITH_LLVM)
    endif(WITH_LLVM)

endif (WITH_C++11)

add_definitions("-lpthread")

# Boost
find_package(Boost COMPONENTS signals filesystem serialization program_options system REQUIRED)
include_directories(${Boost_INCLUDE_DIR} )

# OpenCV
find_package( OpenCV 2.3 REQUIRED )

# PCL 1.8
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

configure_file("${PROJECT_SOURCE_DIR}/src/core/builddefinitions.hpp.in"
               "${PROJECT_SOURCE_DIR}/src/core/builddefinitions.hpp")

add_subdirectory(src)

