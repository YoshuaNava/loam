add_library(loam_ros
            ScanRegistrationRos.cpp
            # MultiScanRegistration.cpp
            # LaserOdometry.cpp
            # LaserMapping.cpp
            TransformMaintenanceRos.cpp)
target_link_libraries(loam_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES})