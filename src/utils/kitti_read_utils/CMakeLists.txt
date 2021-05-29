cmake_minimum_required(VERSION 3.10)
PROJECT(kitti_read_utils)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic")

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES
    kitti_utils
  CATKIN_DEPENDS
#  DEPENDS
)

find_package(Boost REQUIRED COMPONENTS filesystem system thread serialization regex timer)

include_directories(${Boost_INCLUDE_DIRS} /usr/include/eigen3 src/)
link_libraries(${Boost_LIBRARIES})

add_library(kitti_utils src/kitti_utils.cpp
                        src/string_utils.cpp
                        src/Laserscan.cpp)

