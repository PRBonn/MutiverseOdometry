cmake_minimum_required(VERSION 3.10)

project(common_vlo)

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


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES common_vlo
  CATKIN_DEPENDS
#  DEPENDS
)
message("#################################################################################")
find_package( OpenCV 4.3.0 REQUIRED )
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message("#################################################################################")

find_package(Open3D)

message("#################################################################################")
message(STATUS "Open3D library status: ${Open3D_FOUND}")
message(STATUS "    version: ${Open3D_VERSION}")
message(STATUS "    libraries: ${Open3D_LIBRARIES}")
message(STATUS "    3rd party libraries: ${Open3D_LIBRARY_DIRS}")
message(STATUS "    include path: ${Open3D_INCLUDE_DIRS}")
message(STATUS "    include path: ${Open3D_CXX_FLAGS}")
message("#################################################################################")
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
message("#################################################################################")


include_directories(${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    include)
link_directories(${Open3D_LIBRARY_DIRS})
add_library(common_vlo "src/SanityChecker/SanityChecker.cpp"
                       "src/IOdometer.cpp"
                       "src/SanityChecker/LineDetector.cpp"
                       "src/geometry/geometry.cpp"
                       "src/utils_vlo.cpp")
target_link_libraries(common_vlo  ${Open3D_LIBRARIES})
