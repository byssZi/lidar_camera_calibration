cmake_minimum_required(VERSION 3.10)

project(sensors_calib)
set (CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(LIBRARY_NAME ${PROJECT_NAME})

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include(Findrapidjson)
if(NOT RAPIDJSON_FOUND)
  message(STATUS "rapidjson not found")
  return()
endif()

add_compile_options(
  "$<$<CONFIG:DEBUG>:-DDEBUG>"
)

add_subdirectory(src)

add_subdirectory(manual_calib)

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif(BUILD_EXAMPLES)

if(BUILD_APPSROS1)
  add_subdirectory(ros1)
endif(BUILD_APPSROS1)

if(BUILD_APPSROS2)
  add_subdirectory(ros2)
endif(BUILD_APPSROS2)
