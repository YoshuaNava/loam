add_library(loam_velodyne
            ScanRegistration.cpp
            MultiScanRegistration.cpp
            LaserOdometry.cpp
            LaserMapping.cpp
            TransformMaintenance.cpp)
target_link_libraries(loam_velodyne ${catkin_LIBRARIES} ${PCL_LIBRARIES})
