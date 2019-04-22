cmake_minimum_required(VERSION 2.8.3)
project(loam_ros)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_catkin
  sensor_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
  ecl_time
  loam
  loam_msgs
  
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
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs std_srvs pcl_catkin ecl_time loam loam_msgs
  DEPENDS EIGEN3 #PCL
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)


add_library(${PROJECT_NAME} SHARED
            src/loam_ros/common.cpp
            src/loam_ros/TransformMaintenanceRos.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(multiScanRegistration src/multi_scan_registration_node.cpp)
target_link_libraries(multiScanRegistration ${catkin_LIBRARIES} ${PROJECT_NAME} )

add_executable(laserOdometry src/laser_odometry_node.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PROJECT_NAME} )

add_executable(laserMapping src/laser_mapping_node.cpp)
target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PROJECT_NAME} )

add_executable(transformMaintenance src/transform_maintenance_node.cpp)
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PROJECT_NAME} )

