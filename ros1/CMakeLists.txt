cmake_minimum_required(VERSION 3.10)
set(PYTHON_EXECUTABLE /usr/bin/python3)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  cv_bridge
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensors_calib_ros1 
  ${CMAKE_CURRENT_LIST_DIR}/sensors_calib_ros1.cpp
)
target_link_libraries(sensors_calib_ros1 
  PUBLIC
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

