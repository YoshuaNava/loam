add_library(loam_velodyne_ros
            # ScanRegistration.cpp
            # MultiScanRegistration.cpp
            # LaserOdometry.cpp
            # LaserMapping.cpp
            TransformMaintenanceRos.cpp)
target_link_libraries(loam_velodyne_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES})