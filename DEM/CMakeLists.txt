cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

set(CMAKE_CXX_COMPILER g++)
set(CMAKE_BUILD_TYPE Debug)

project(ply_to_dem)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ply_to_dem ply_to_dem.cpp)

target_link_libraries(ply_to_dem ${PCL_LIBRARIES})