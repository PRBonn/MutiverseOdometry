cmake_minimum_required(VERSION 3.10)

project(visual_odometry_core)

#set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
#set(CMAKE_CXX_FLAGS_RELEASE "-O0")

#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic")


add_definitions(-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_BUILD_TYPE "RELEASE")


find_package(catkin REQUIRED
  COMPONENTS
)
find_package(common_vlo REQUIRED)
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    visual_odometry_core geometry utils visual_odometry
  CATKIN_DEPENDS
#  DEPENDS
)

find_package( OpenCV 4.3.0 REQUIRED )
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package (Eigen3 REQUIRED)

find_package( Boost 1.65.1 REQUIRED COMPONENTS )
include_directories( ${Boost_INCLUDE_DIRS} )
message(STATUS "Boost library status:")
message(STATUS "    version: ${Boost_VERSION}")
message(STATUS "    libraries: ${Boost_LIBRARIES}")
message(STATUS "    include path: ${Boost_INCLUDE_DIRS}")

find_package(Open3D)
message("#################################################################################")
message(STATUS "Open3D library status: ${Open3D_FOUND}")
message(STATUS "    version: ${Open3D_VERSION}")
message(STATUS "    libraries: ${Open3D_LIBRARIES}")
message(STATUS "    3rd party libraries: ${Open3D_LIBRARY_DIRS}")
message(STATUS "    include path: ${Open3D_INCLUDE_DIRS}")
message(STATUS "    include path: ${Open3D_CXX_FLAGS}")
message("#################################################################################")



include_directories( ${OpenCV_INCLUDE_DIRS}
                     ${catkin_INCLUDE_DIRS}
                     ${common_vlo_INCLUDE_DIRS}
                     include)
add_library(geometry SHARED "src/geometry/Camera.cpp")
target_link_libraries(geometry ${OpenCV_LIBS})

add_library(utils SHARED "src/utils/conversions.cpp")
target_link_libraries(utils ${OpenCV_LIBS} geometry)



add_library(visual_odometry "src/visual_odometry/VisualFrame.cpp"
                            "src/visual_odometry/Map.cpp"
                            "src/visual_odometry/MapPoint.cpp"
                            "src/visual_odometry/FeatureDetector.cpp"
                            "src/visual_odometry/FeatureMatcher.cpp")

target_link_libraries(visual_odometry ${OpenCV_LIBS} geometry utils)

link_directories(${Open3D_LIBRARY_DIRS})
add_library(visual_odometry_core "src/VO_FrameToFrame_5point.cpp"
  #  "src/VO_PnP.cpp"
    "src/IVisualOdometer.cpp")
target_link_libraries(visual_odometry_core ${OpenCV_LIBS} visual_odometry geometry ${Open3D_LIBRARIES} ${common_vlo_LIBRARIES})




