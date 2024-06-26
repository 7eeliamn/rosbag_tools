cmake_minimum_required(VERSION 2.8.3)
project(bagextracter)

add_compile_options(-std=c++11)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin
    REQUIRED COMPONENTS
            roscpp
	    rosbag
            sensor_msgs
	    std_msgs
	    cv_bridge
	    pcl_ros
	    pcl_conversions
            visualization_msgs
)
find_package(OpenCV REQUIRED)

#add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    CATKIN_DEPENDS
            roscpp
            sensor_msgs
            nav_msgs
            geometry_msgs
            visualization_msgs
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS} ${EIGEN3_LIBRARY_DIR})

add_executable(pcdSaver src/pcdSaver.cpp)
add_dependencies(pcdSaver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PCL_LIBRARIES} ${EIGEN3_LIBRARY_DIR})
target_link_libraries(pcdSaver ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARY_DIR})

add_executable(imageSaver src/imageSaver.cpp)
add_dependencies(imageSaver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${EIGEN3_LIBRARY_DIR})
target_link_libraries(imageSaver ${catkin_LIBRARIES} ${EIGEN3_LIBRARY_DIR} ${OpenCV_LIBRARIES})

add_executable(bagReader src/bagReader.cpp)
add_dependencies(bagReader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${EIGEN3_LIBRARY_DIR})
target_link_libraries(bagReader ${catkin_LIBRARIES} ${EIGEN3_LIBRARY_DIR} ${OpenCV_LIBRARIES})

