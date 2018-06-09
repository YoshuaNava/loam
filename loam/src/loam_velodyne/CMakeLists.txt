add_library(loam_velodyne SHARED
            MultiScanRegistration.cpp
            LaserOdometry.cpp
            LaserMapping.cpp
)
target_link_libraries(loam_velodyne ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam)
