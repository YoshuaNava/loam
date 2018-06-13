add_library(loam_continuous SHARED
            CtScanRegistration.cpp
            CtLaserOdometry.cpp
            # LaserMapping.cpp
            # TransformMaintenance.cpp
)
target_link_libraries(loam_continuous ${catkin_LIBRARIES} loam)
