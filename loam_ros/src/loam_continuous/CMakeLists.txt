add_library(loam_continuous_ros
            CtScanRegistrationRos.cpp
            # MultiScanRegistration.cpp
            # LaserOdometry.cpp
            # LaserMapping.cpp
            # TransformMaintenanceRos.cpp
)
target_link_libraries(loam_continuous_ros ${catkin_LIBRARIES})
