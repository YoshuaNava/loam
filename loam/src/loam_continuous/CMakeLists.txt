add_library(loam_continuous SHARED
            CtScanRegistration.cpp
            # LaserOdometry.cpp
            # LaserMapping.cpp
            # TransformMaintenance.cpp
)
target_link_libraries(loam_continuous ${catkin_LIBRARIES} loam)
