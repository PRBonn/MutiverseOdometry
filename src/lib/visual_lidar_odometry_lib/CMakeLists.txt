cmake_minimum_required(VERSION 3.10)

project(visual_lidar_odometry_core)

#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic")

#set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
#set(CMAKE_CXX_FLAGS_RELEASE "-O0")


add_definitions(-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_BUILD_TYPE "RELEASE")


find_package(catkin REQUIRED
  COMPONENTS
)
find_package(common_vlo)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  visual_lidar_odometry_core
  CATKIN_DEPENDS
#  DEPENDS
)
find_package(visual_odometry_core REQUIRED)
find_package(lidar_odometry_core REQUIRED)

find_package( OpenCV 4.3.0 REQUIRED )
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package (Eigen3 REQUIRED)

find_package( libpointmatcher REQUIRED )
message(STATUS "LibPointMatcher library status:")
message(STATUS "    version: ${libpointmatcher_VERSION}")
message(STATUS "    libraries: ${libpointmatcher_LIBRARIES}")
message(STATUS "    include path: ${libpointmatcher_INCLUDE_DIRS}")

find_package(Open3D)
message("#################################################################################")
message(STATUS "Open3D library status: ${Open3D_FOUND}")
message(STATUS "    version: ${Open3D_VERSION}")
message(STATUS "    libraries: ${Open3D_LIBRARIES}")
message(STATUS "    3rd party libraries: ${Open3D_LIBRARY_DIRS}")
message(STATUS "    include path: ${Open3D_INCLUDE_DIRS}")
message(STATUS "    include path: ${Open3D_CXX_FLAGS}")
message("#################################################################################")

include_directories(${catkin_INCLUDE_DIRS}
                    ${visual_odometry_core_INCLUDE_DIRS}
                    ${lidar_odometry_core_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${libpointmatcher_LIBRARIES}
                    ${common_vlo_INCLUDE_DIRS}
                     include)

add_library(visual_lidar_odometry_core "src/HuangsVLO/HuangVLO.cpp"
                                       "src/HuangsVLO/LidarOdometer1DOF.cpp"
                                       "src/FeatureFusionVLO/FeatureFusionVLO.cpp"
                                       "src/FeatureFusionVLO/FeatureFusionMatcher.cpp"
                                       "src/VLO6dof/VLO6dof.cpp"
                                       "src/VisualLidarFrame.cpp"
                                       "src/VisualLidarFusionFrame.cpp"
                                       "src/ComboVLO/ComboVLO.cpp"
                                       "src/VOwithScaleVLO/VOwithScaleVLO.cpp"
                                       "src/ColorBasedVLO/ColorBasedVLO.cpp")
target_link_libraries(visual_lidar_odometry_core  ${catkin_LIBRARIES}
                                                  ${rviz_bridge_viewer_LIBRARIES}
                                                  ${EIGEN3_LIBS}
                                                  ${visual_odometry_core_LIBRARIES}
                                                  ${lidar_odometry_core_LIBRARIES}
                                                  ${OpenCV_LIBS}
                                                  ${libpointmatcher_LIBRARIES}
                                                  ${common_vlo_LIBRARIES})




