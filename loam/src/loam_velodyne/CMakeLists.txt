add_library(loam_velodyne SHARED
            MultiScanRegistration.cpp
            VeloLaserOdometry.cpp
            VeloLaserMapping.cpp
)
target_link_libraries(loam_velodyne ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam)
