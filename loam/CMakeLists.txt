cmake_minimum_required(VERSION 2.8.3)
project(loam)

set(CMAKE_CXX_STANDARD 11)

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
  LIBRARIES loam_velodyne
)

add_subdirectory(src/loam_velodyne)

