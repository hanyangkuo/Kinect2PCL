cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(kinect_pcl)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (kinect_pcl kinect_pcl.cpp)
target_link_libraries (kinect_pcl ${PCL_LIBRARIES})