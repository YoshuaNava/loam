cmake_minimum_required(VERSION 2.8.3)
project(loam)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

find_package(catkin REQUIRED COMPONENTS
  pcl_catkin
  ecl_time
  
  QUIET
)

find_package(Eigen3 REQUIRED QUIET)
# find_package(PCL REQUIRED QUIET)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	#${EIGEN3_INCLUDE_DIR}
  # ${PCL_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS pcl_catkin ecl_time
  DEPENDS EIGEN3 #PCL
  INCLUDE_DIRS include
  LIBRARIES loam loam_velodyne loam_continuous
)

add_definitions( -march=native )

add_subdirectory(src/loam)
add_subdirectory(src/loam_velodyne)
# add_subdirectory(src/loam_continuous)

