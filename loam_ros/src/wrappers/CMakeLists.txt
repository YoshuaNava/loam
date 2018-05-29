add_library(${PROJECT_NAME}
            # ScanRegistration.cpp
            # MultiScanRegistration.cpp
            # LaserOdometry.cpp
            # LaserMapping.cpp
            TransformMaintenanceRos.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
