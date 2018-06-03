cmake_minimum_required(VERSION 2.8.3)
project(loam_msgs)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  
  QUIET
)

add_service_files(
  FILES
  PoseUpdate.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_generation 
)