add_library(loam SHARED
            ScanRegistration.cpp
            # LaserOdometry.cpp
            # LaserMapping.cpp
            TransformMaintenance.cpp
)
target_link_libraries(loam ${catkin_LIBRARIES} ${PCL_LIBRARIES})
