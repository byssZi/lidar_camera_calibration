cmake_minimum_required(VERSION 3.10)
set(PYTHON_EXECUTABLE /usr/bin/python3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
#list(APPEND CMAKE_PREFIX_PATH "/home/nuc/dev_ws/install/msg_all/share/msg_all/cmake")
#find_package(msg_all REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(
	include
	#/home/nuc/dev_ws/install/msg_all/include/msg_all
)

link_directories(
	#/home/nuc/dev_ws/install/msg_all/lib
)

add_executable(sensors_calib_ros2 ${CMAKE_CURRENT_LIST_DIR}/sensors_calib_ros2.cpp)
#ament_target_dependencies(sensors_calib_ros2 rclcpp sensor_msgs cv_bridge msg_all)
ament_target_dependencies(sensors_calib_ros2 rclcpp sensor_msgs cv_bridge)
target_link_libraries(sensors_calib_ros2
    ${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

ament_package()
