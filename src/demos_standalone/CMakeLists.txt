cmake_minimum_required(VERSION 3.10)
project(standalone)



add_definitions(-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_BUILD_TYPE "RELEASE")


set(CATKIN_PACKAGE_DEPENDENCIES
    roscpp
    geometry_msgs
    pcl_ros
    tf2_ros
    tf2
    eigen_conversions
    nav_msgs
    pcl_conversions
    roscpp
    sensor_msgs
    tf
    tf_conversions
    rosbag_storage

)

find_package(catkin REQUIRED COMPONENTS  ${CATKIN_PACKAGE_DEPENDENCIES})
find_package(kitti_read_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(common_vlo REQUIRED)
find_package(open3d_conversions REQUIRED)
find_package(visual_odometry_core REQUIRED)
find_package(lidar_odometry_core REQUIRED)
find_package(visual_lidar_odometry_core REQUIRED)
find_package(yaml-cpp REQUIRED )

find_package( OpenCV 4.3.0 REQUIRED )
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package(Open3D)
message("#################################################################################")
message(STATUS "Open3D library status: ${Open3D_FOUND}")
message(STATUS "    version: ${Open3D_VERSION}")
message(STATUS "    libraries: ${Open3D_LIBRARIES}")
message(STATUS "    3rd party libraries: ${Open3D_LIBRARY_DIRS}")
message(STATUS "    include path: ${Open3D_INCLUDE_DIRS}")
message(STATUS "    include path: ${Open3D_CXX_FLAGS}")
message("#################################################################################")


include_directories(
    include

    ${catkin_INCLUDE_DIRS}
    ${visual_odometry_core_INCLUDE_DIRS}
    ${lidar_odometry_core_INCLUDE_DIRS}
    ${visual_lidar_odometry_core_INCLUDE_DIRS}
    ${common_vlo_INCLUDE_DIRS}
    ${kitti_read_utils_INCLUDE_DIRS}
     ${OpenCV_INCLUDE_DIRS})

set(common_libraries ${catkin_LIBRARIES}

    ${EIGEN3_LIBS}
    ${visual_odometry_core_LIBRARIES}

    ${lidar_odometry_core_LIBRARIES}

    ${Open3D_LIBRARIES}

    yaml-cpp
    ${common_vlo_LIBRARIES}

    ${lidar_odometry_lego_loam_LIBRARIES}
    ${OpenCV_LIBS})

link_directories(${Open3D_LIBRARY_DIRS})

add_executable(run_VLO_ComboVLO_pipeline run_VLO_ComboVLO_pipeline.cpp)
target_link_libraries(run_VLO_ComboVLO_pipeline
    ${catkin_LIBRARIES}
    yaml-cpp
    ${Open3D_LIBRARIES}
    ${lidar_odometry_core_LIBRARIES}
    ${visual_lidar_odometry_core_LIBRARIES}
    ${visual_odometry_core_LIBRARIES}
    ${common_vlo_LIBRARIES}
    ${OpenCV_LIBS}
    ${kitti_read_utils_LIBRARIES}

    )


