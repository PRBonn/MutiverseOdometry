cmake_minimum_required(VERSION 3.10)

project(lidar_odometry_core)

#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic")
#set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
#set(CMAKE_CXX_FLAGS_RELEASE "-O0")
#set(CMAKE_BUILD_TYPE "RELEASE")

add_definitions(-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED
  COMPONENTS
  pcl_conversions
  pcl_ros
  open3d_conversions
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(CUDA)
if(CUDA_FOUND)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()

find_package(common_vlo REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES point2plane ndt_omp fast_gicp gicp #lidar_odometry_lego_loam
  CATKIN_DEPENDS open3d_conversions #cloud_msgs
#  DEPENDS
)

find_package (Eigen3 REQUIRED)
find_package(Open3D)


include_directories(${catkin_INCLUDE_DIRS}
                    ${common_vlo_INCLUDE_DIRS}
                    include
                    ${open3d_conversions_DIRS}
                    )

link_directories(${Open3D_LIBRARY_DIRS})

add_library(point2plane
    "src/ILidarOdometer.cpp"
    "src/LidarOdometerFrameToFrame.cpp"
    "src/lidar_odometry/LidarFrame.cpp"
    "src/lidar_odometry/point_cloud_analysis.cpp"
    )
target_link_libraries(point2plane
    ${libpointmatcher_LIBRARIES}
    ${Open3D_LIBRARIES}
    ${catkin_LIBRARIES}
    ${open3d_conversions_LIBRARIES}
    )

add_library(ndt_omp
  src/ILidarOdometer.cpp
  src/NdtLO/pclomp/voxel_grid_covariance_omp.cpp
  src/NdtLO/pclomp/ndt_omp.cpp
  src/NdtLO/pclomp/gicp_omp.cpp
  src/NdtLO/NdtLO.cpp
)
target_link_libraries(ndt_omp ${catkin_LIBRARIES})

add_library(fast_gicp
  src/GicpLO/fast_gicp/gicp/fast_gicp.cpp
  src/GicpLO/fast_gicp/gicp/fast_gicp_st.cpp
  src/GicpLO/fast_gicp/gicp/fast_vgicp.cpp
  src/GicpLO/fast_gicp/gicp/experimental/fast_gicp_mp.cpp

)

target_link_libraries(fast_gicp
  ${catkin_LIBRARIES}
)

add_library(gicp
  src/ILidarOdometer.cpp
  src/GicpLO/GicpLO.cpp
  src/GicpLO/fast_gicp/gicp/fast_gicp.cpp
  src/GicpLO/fast_gicp/gicp/fast_gicp_st.cpp
)
target_link_libraries(gicp ${catkin_LIBRARIES})


target_include_directories(fast_gicp PUBLIC
  include
  thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

#add_executable(gicp_align src/GicpLO/align.cpp)
#add_dependencies(gicp_align ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(gicp_align
#  ${catkin_LIBRARIES}
#  fast_gicp
#)

add_executable(gicp_kitti src/GicpLO/kitti.cpp)
add_dependencies(gicp_kitti ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gicp_kitti
  ${catkin_LIBRARIES}
  fast_gicp
)



#add_executable(align
#  src/align.cpp
#)
#add_dependencies(align
#  ndt_omp
#)
#target_link_libraries(align
#  ${catkin_LIBRARIES}
##  ${PCL_LIBRARIES}
#  ndt_omp
#)



############
## INSTAL ##
############

#install(
#  TARGETS
#    ndt_omp
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

## install headers
#install(DIRECTORY include/pclomp
#        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

#add_library(teaserlo
#    "src/TeaserLO/TeaserLO.cpp"
#    "src/TeaserLO/TeaserFeaturesLO.cpp"
#    )
#target_link_libraries(teaserlo
#    teaserpp::teaser_registration
#    ${catkin_LIBRARIES}
#    ${Open3D_LIBRARIES}
#    )

#find_package(teaserpp REQUIRED)
#message(STATUS "teaserpp library status: ${teaserpp_FOUND}")
#message(STATUS "    version: ${teaserpp_VERSION}")
#message(STATUS "    libraries: ${teaserpp_LIBRARIES}")
#message(STATUS "    3rd party libraries: ${teaserpp_LIBRARY_DIRS}")
#message(STATUS "    include path: ${teaserpp_INCLUDE_DIRS}")
#message(STATUS "    include path: ${teaserpp_CXX_FLAGS}")
#message("#################################################################################")
#    find_package(cloud_msgs)
#    find_package(GTSAM REQUIRED QUIET)
#find_package(OpenMP)
#if (OPENMP_FOUND)
#    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()
#message("#################################################################################")

#add_library(lidar_odometry_lego_loam  "src/LeGO-LOAM/imageProjection.cpp"
#                                      "src/LeGO-LOAM/featureAssociation.cpp"
#                                      "src/LeGO-LOAM/mapOptmization.cpp"
#                                      "src/LeGO-LOAM/transformFusion.cpp")

#target_link_libraries(lidar_odometry_lego_loam ${catkin_LIBRARIES}  gtsam)



