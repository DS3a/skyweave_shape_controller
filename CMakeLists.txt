cmake_minimum_required(VERSION 3.10)
project(skyweave_shape_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

add_library(skyweave_link_tracker SHARED src/skyweave_link_tracker.cpp)

target_link_libraries(skyweave_link_tracker ${GAZEBO_LIBRARIES})
