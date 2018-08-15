cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_hdl_viewer_simple)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_definitions(-std=gnu++11)

add_executable(pcl_hdl_viewer_simple main.cpp)
target_link_libraries(pcl_hdl_viewer_simple ${PCL_LIBRARIES})
